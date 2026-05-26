#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "qcar2_msgs_2/msg/lateral_guidance.hpp"
#include "qcar2_msgs_2/msg/ph_quintic_path.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_control_2/ph_runtime.hpp"

namespace ph = qcar2_control_2::ph;

static constexpr double kEpsilon = 1e-9;

class VFGGuidancePHNode : public rclcpp::Node
{
public:
  VFGGuidancePHNode()
  : Node("vfg_guidance_ph_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 100.0);
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.2);

    k_e_ = this->declare_parameter<double>("k_e", 1.0);

    lookahead_min_distance_ = this->declare_parameter<double>("lookahead_min_distance", 0.2);
    lookahead_max_distance_ = this->declare_parameter<double>("lookahead_max_distance", 1.5);
    lookahead_speed_min_    = this->declare_parameter<double>("lookahead_speed_min",    0.6);
    lookahead_speed_max_    = this->declare_parameter<double>("lookahead_speed_max",    1.2);
    lookahead_sample_count_ = this->declare_parameter<int>("lookahead_sample_count", 20);

    ff_lookahead_base_m_   = this->declare_parameter<double>("ff_lookahead_base_m", 0.12);
    ff_lookahead_tau_s_    = this->declare_parameter<double>("ff_lookahead_tau_s", 0.16);

    local_search_half_window_m_ = this->declare_parameter<double>("local_search_half_window_m", 0.8);
    local_search_seed_count_ = this->declare_parameter<int>("local_search_seed_count", 11);
    global_seeds_per_segment_ = this->declare_parameter<int>("global_seeds_per_segment", 9);
    newton_max_iterations_ = this->declare_parameter<int>("newton_max_iterations", 12);
    newton_tolerance_ = this->declare_parameter<double>("newton_tolerance", 1e-6);
    closest_global_fallback_dist_m_ =
      this->declare_parameter<double>("closest_global_fallback_dist_m", 1.2);

    publish_markers_ = this->declare_parameter<bool>("publish_markers", false);
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    sub_path_ = this->create_subscription<qcar2_msgs_2::msg::PhQuinticPath>(
      "/planning/local_path_ph", rclcpp::QoS(1),
      std::bind(&VFGGuidancePHNode::pathCallback, this, std::placeholders::_1));

    sub_motor_cmd_ = this->create_subscription<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", rclcpp::QoS(10),
      std::bind(&VFGGuidancePHNode::motorCmdCallback, this, std::placeholders::_1));

    pub_guidance_ = this->create_publisher<qcar2_msgs_2::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10));

    if (publish_markers_) {
      pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vfg/debug_markers", rclcpp::QoS(10));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, update_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VFGGuidancePHNode::update, this));

    RCLCPP_INFO(this->get_logger(), "VFG PH guidance node started");
  }

private:
  void pathCallback(const qcar2_msgs_2::msg::PhQuinticPath::SharedPtr msg)
  {
    has_path_ = true;
    invalid_reason_.clear();

    if (!runtime_.updateFromMsg(*msg, invalid_reason_)) {
      has_valid_path_ = false;
      return;
    }

    has_valid_path_ = true;
  }

  void motorCmdCallback(const qcar2_interfaces::msg::MotorCommands::SharedPtr msg)
  {
    if (msg->motor_names.size() >= 2 && msg->values.size() >= 2) {
      for (size_t i = 0; i < msg->motor_names.size(); ++i) {
        if (msg->motor_names[i] == "motor_throttle") {
          current_speed_cmd_ = std::fabs(msg->values[i]);
          have_speed_cmd_ = true;
          return;
        }
      }
    }
  }

  void publishStatus(uint8_t status, const std::string & msg)
  {
    qcar2_msgs_2::msg::LateralGuidance out;
    out.header.stamp = this->now();
    out.header.frame_id = runtime_.frameId();
    out.curve_id = runtime_.trajId();
    out.status = status;
    out.status_msg = msg;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    out.curvature_1pm = nan;
    out.lookahead_distance_m = nan;
    out.lookahead_max_curvature_1pm = nan;
    out.ff_lookahead_curvature_1pm = nan;
    pub_guidance_->publish(out);

    if (publish_markers_) {
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(clear);
      pub_markers_->publish(ma);
    }
  }

  void update()
  {
    if (!has_path_) {
      publishStatus(qcar2_msgs_2::msg::LateralGuidance::STATUS_NO_PATH, "No PH path received");
      return;
    }

    if (!has_valid_path_) {
      publishStatus(
        qcar2_msgs_2::msg::LateralGuidance::STATUS_INVALID_PATH,
        invalid_reason_.empty() ? "Invalid PH path" : invalid_reason_);
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        runtime_.frameId(), base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      publishStatus(qcar2_msgs_2::msg::LateralGuidance::STATUS_TF_FAIL, ex.what());
      return;
    }

    const ph::Vec2 p_vehicle{tf.transform.translation.x, tf.transform.translation.y};
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    const double psi_vehicle = tf2::getYaw(q);

    ph::ClosestResult closest;
    if (!runtime_.findClosest(
        p_vehicle,
        local_search_half_window_m_,
        local_search_seed_count_,
        global_seeds_per_segment_,
        newton_max_iterations_,
        newton_tolerance_,
        closest_global_fallback_dist_m_,
        closest)) {
      publishStatus(qcar2_msgs_2::msg::LateralGuidance::STATUS_INVALID_PATH, "Closest-point failed");
      return;
    }

    const auto e_star = runtime_.evalAtS(closest.s);
    const ph::Vec2 v2 = e_star.d1;
    const double v2_norm = ph::norm(v2);
    if (v2_norm < kEpsilon || !std::isfinite(v2_norm)) {
      publishStatus(qcar2_msgs_2::msg::LateralGuidance::STATUS_NUMERIC_FAIL, "Zero tangent");
      return;
    }

    const ph::Vec2 v2_hat = ph::normalize(v2);
    const ph::Vec2 n_hat{-v2_hat.y, v2_hat.x};

    const ph::Vec2 v1 = closest.p - p_vehicle;
    const double v1_norm = ph::norm(v1);
    ph::Vec2 v1_hat{0.0, 0.0};
    if (v1_norm > kEpsilon && std::isfinite(v1_norm)) {
      v1_hat = {v1.x / v1_norm, v1.y / v1_norm};
    }
    const double cross_track_error = ph::dot(v1, n_hat);

    // w1 = tanh(k_e * |e|)
    const double w1 = std::tanh(k_e_ * std::fabs(cross_track_error));
    const double w2 = std::sqrt(std::max(0.0, 1.0 - w1 * w1));

    const ph::Vec2 v_vfg = w1 * v1_hat + w2 * v2_hat;
    const double v_vfg_norm = ph::norm(v_vfg);
    if (v_vfg_norm < kEpsilon || !std::isfinite(v_vfg_norm)) {
      publishStatus(qcar2_msgs_2::msg::LateralGuidance::STATUS_NUMERIC_FAIL, "Invalid VFG vector");
      return;
    }

    const double psi_vfg = std::atan2(v_vfg.y, v_vfg.x);
    const double psi_path_tangent = std::atan2(v2_hat.y, v2_hat.x);
    const double heading_error = ph::wrapToPi(psi_vfg - psi_vehicle);

    const double curvature = runtime_.curvatureFrom(e_star.d1, e_star.d2);

    // Linear endpoint mapping: lookahead_speed_min → lookahead_min_distance,
    //                          lookahead_speed_max → lookahead_max_distance.
    // Outside the speed window the distance is clamped to the endpoint values.
    double lookahead_distance = lookahead_min_distance_;
    if (have_speed_cmd_) {
      const double span = lookahead_speed_max_ - lookahead_speed_min_;
      double t = 0.0;
      if (span > kEpsilon) {
        t = (current_speed_cmd_ - lookahead_speed_min_) / span;
      }
      t = ph::clamp(t, 0.0, 1.0);
      lookahead_distance = lookahead_min_distance_ +
        t * (lookahead_max_distance_ - lookahead_min_distance_);
    }
    double actual_lookahead = 0.0;
    double lookahead_max_curvature = std::numeric_limits<double>::quiet_NaN();
    runtime_.computeLookaheadMaxCurvature(
      closest.s, lookahead_distance, lookahead_sample_count_,
      actual_lookahead, lookahead_max_curvature);
    lookahead_distance = actual_lookahead;

    double ff_lookahead_curvature = std::numeric_limits<double>::quiet_NaN();
    const double ff_lookahead_distance = ff_lookahead_base_m_ + ff_lookahead_tau_s_ * current_speed_cmd_;
    if (runtime_.totalLength() > kEpsilon && ff_lookahead_distance > kEpsilon) {
      const double s_ff = std::min(runtime_.totalLength(), closest.s + ff_lookahead_distance);
      ff_lookahead_curvature = runtime_.curvatureAtS(s_ff);
    }

    qcar2_msgs_2::msg::LateralGuidance out;
    out.header.stamp = this->now();
    out.header.frame_id = runtime_.frameId();
    out.curve_id = runtime_.trajId();
    out.heading_error_rad = heading_error;
    out.u_star = runtime_.ratioFromS(closest.s);
    out.closest_point.x = closest.p.x;
    out.closest_point.y = closest.p.y;
    out.closest_point.z = 0.0;
    out.psi_vehicle_rad = psi_vehicle;
    out.psi_path_tangent_rad = psi_path_tangent;
    out.psi_vfg_rad = psi_vfg;
    out.cross_track_error_m = cross_track_error;
    out.curvature_1pm = curvature;
    out.lookahead_distance_m = lookahead_distance;
    out.lookahead_max_curvature_1pm = lookahead_max_curvature;
    out.ff_lookahead_curvature_1pm = ff_lookahead_curvature;
    out.w1_to_path = w1;
    out.w2_tangent = w2;
    out.v1_to_path_norm = v1_norm;
    out.v_vfg_norm = v_vfg_norm;
    out.status = qcar2_msgs_2::msg::LateralGuidance::STATUS_OK;
    out.status_msg = "OK";
    pub_guidance_->publish(out);

    if (publish_markers_) {
      publishMarkers(p_vehicle, closest.p, v2_hat, n_hat, v1, v2, v_vfg);
    }
  }

  void publishMarkers(
    const ph::Vec2 & p,
    const ph::Vec2 & q,
    const ph::Vec2 & v2_hat,
    const ph::Vec2 & n_hat,
    const ph::Vec2 & v1,
    const ph::Vec2 & v2,
    const ph::Vec2 & v_vfg)
  {
    visualization_msgs::msg::MarkerArray ma;
    const rclcpp::Time stamp = this->now();

    auto makeArrow = [&](int id, const ph::Vec2 & dir, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = runtime_.frameId();
      m.header.stamp = stamp;
      m.ns = "vfg_ph";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.02;
      m.scale.y = 0.05;
      m.scale.z = 0.08;
      m.color.a = 1.0f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;

      geometry_msgs::msg::Point p0;
      p0.x = p.x;
      p0.y = p.y;
      p0.z = 0.05;

      const ph::Vec2 u = ph::normalize(dir);
      geometry_msgs::msg::Point p1;
      p1.x = p.x + 0.5 * u.x;
      p1.y = p.y + 0.5 * u.y;
      p1.z = 0.05;

      m.points = {p0, p1};
      return m;
    };

    auto makeSphere = [&](int id, const ph::Vec2 & pos, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = runtime_.frameId();
      m.header.stamp = stamp;
      m.ns = "vfg_ph";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.08;
      m.scale.y = 0.08;
      m.scale.z = 0.08;
      m.color.a = 1.0f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.pose.position.x = pos.x;
      m.pose.position.y = pos.y;
      m.pose.position.z = 0.05;
      return m;
    };

    ma.markers.push_back(makeSphere(0, q, 1.0f, 0.5f, 0.1f));
    ma.markers.push_back(makeArrow(1, v2_hat, 0.2f, 1.0f, 0.2f));
    ma.markers.push_back(makeArrow(2, n_hat, 1.0f, 0.2f, 0.2f));
    ma.markers.push_back(makeArrow(3, v1, 0.2f, 0.6f, 1.0f));
    ma.markers.push_back(makeArrow(4, v2, 0.7f, 0.7f, 0.7f));
    ma.markers.push_back(makeArrow(5, v_vfg, 0.1f, 0.3f, 1.0f));

    pub_markers_->publish(ma);
  }

  double update_rate_hz_{100.0};
  double tf_timeout_sec_{0.2};
  double k_e_{1.0};
  double lookahead_min_distance_{0.2};
  double lookahead_max_distance_{1.5};
  double lookahead_speed_min_{0.6};
  double lookahead_speed_max_{1.2};
  int lookahead_sample_count_{20};
  double ff_lookahead_base_m_{0.2};
  double ff_lookahead_tau_s_{0.16};

  double local_search_half_window_m_{0.8};
  int local_search_seed_count_{11};
  int global_seeds_per_segment_{9};
  int newton_max_iterations_{12};
  double newton_tolerance_{1e-6};
  double closest_global_fallback_dist_m_{1.2};

  bool publish_markers_{false};
  std::string base_frame_{"base_link"};

  bool has_path_{false};
  bool has_valid_path_{false};
  std::string invalid_reason_;

  ph::PhPathRuntime runtime_;

  double current_speed_cmd_{0.0};
  bool have_speed_cmd_{false};

  rclcpp::Subscription<qcar2_msgs_2::msg::PhQuinticPath>::SharedPtr sub_path_;
  rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr sub_motor_cmd_;
  rclcpp::Publisher<qcar2_msgs_2::msg::LateralGuidance>::SharedPtr pub_guidance_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFGGuidancePHNode>());
  rclcpp::shutdown();
  return 0;
}

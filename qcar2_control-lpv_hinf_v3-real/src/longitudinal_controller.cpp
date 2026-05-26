#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "std_msgs/msg/float64.hpp"
#include "qcar2_msgs_2/msg/lateral_guidance.hpp"
#include "qcar2_msgs_2/msg/ph_quintic_path.hpp"
#include "qcar2_control_2/ph_runtime.hpp"

namespace ph = qcar2_control_2::ph;

static constexpr double kEpsilon = 1e-6;
static constexpr double kMaxDt   = 1.0;

class LongitudinalControllerNode : public rclcpp::Node
{
public:
  LongitudinalControllerNode()
  : Node("longitudinal_controller_node")
  {
    control_rate_hz_      = this->declare_parameter<double>("control_rate_hz", 200.0);
    guidance_timeout_sec_ = this->declare_parameter<double>("guidance_timeout_sec", 0.5);
    throttle_test_value_mps_ = this->declare_parameter<double>("throttle_test_value_mps", 0.0);
    stop_on_penultimate_  = this->declare_parameter<bool>("stop_on_penultimate", true);
    stop_distance_m_      = this->declare_parameter<double>("stop_distance_m", 0.1);
    stop_sample_count_    = this->declare_parameter<int>("stop_sample_count", 200);

    speed_control_enabled_ = this->declare_parameter<bool>("speed_control_enabled", true);
    speed_max_             = this->declare_parameter<double>("speed_max", 1.5);
    speed_min_             = this->declare_parameter<double>("speed_min", 0.3);
    curvature_low_         = this->declare_parameter<double>("curvature_low", 0.3);
    curvature_high_        = this->declare_parameter<double>("curvature_high", 0.7);
    max_acceleration_      = this->declare_parameter<double>("max_acceleration", 0.2);
    max_deceleration_      = this->declare_parameter<double>("max_deceleration", 0.4);
    speed_smooth_alpha_    = this->declare_parameter<double>("speed_smooth_alpha", 0.8);

    sub_guidance_ = this->create_subscription<qcar2_msgs_2::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10),
      std::bind(&LongitudinalControllerNode::guidanceCallback, this, std::placeholders::_1));

    sub_ph_path_ = this->create_subscription<qcar2_msgs_2::msg::PhQuinticPath>(
      "/planning/local_path_ph", rclcpp::QoS(1),
      std::bind(&LongitudinalControllerNode::phPathCallback, this, std::placeholders::_1));

    pub_speed_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/target_speed", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, control_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LongitudinalControllerNode::update, this));

    RCLCPP_INFO(this->get_logger(), "Longitudinal controller started");
  }

private:
  void guidanceCallback(const qcar2_msgs_2::msg::LateralGuidance::SharedPtr msg)
  {
    latest_guidance_ = *msg;
    have_guidance_ = true;
    last_guidance_time_ = this->now();

    if (!have_curve_id_ || msg->curve_id != last_curve_id_) {
      last_curve_id_ = msg->curve_id;
      have_curve_id_ = true;
      stopped_ = false;
    }
  }

  void phPathCallback(const qcar2_msgs_2::msg::PhQuinticPath::SharedPtr msg)
  {
    if (!stop_on_penultimate_) {
      have_stop_u_ = false;
      return;
    }

    std::string reason;
    if (!ph_runtime_.updateFromMsg(*msg, reason)) {
      have_stop_u_ = false;
      return;
    }

    stop_curve_id_ = static_cast<uint32_t>(msg->traj_id);
    const int count = std::max(20, stop_sample_count_);
    stop_u_ = static_cast<double>(count - 2) / static_cast<double>(count - 1);
    stop_curve_point_ph_ = ph_runtime_.pointAtRatio(stop_u_);
    have_stop_u_ = true;
  }

  bool guidanceTimedOut(const rclcpp::Time & now) const
  {
    if (!have_guidance_) {
      return true;
    }
    return (now - last_guidance_time_).seconds() > guidance_timeout_sec_;
  }

  // Curvature-to-speed mapping using a cubic smoothstep.
  // kappa <= curvature_low_: keep speed_max_.
  // kappa >= curvature_high_: keep speed_min_.
  double computeSpeedFromCurvature(double curvature) const
  {
    const double kappa = std::fabs(curvature);
    if (kappa <= curvature_low_) {
      return speed_max_;
    }
    if (kappa >= curvature_high_) {
      return speed_min_;
    }
    const double t = (kappa - curvature_low_) / (curvature_high_ - curvature_low_);
    const double smooth_t = t * t * (3.0 - 2.0 * t);
    return speed_max_ - (speed_max_ - speed_min_) * smooth_t;
  }

  void update()
  {
    const rclcpp::Time now = this->now();
    double dt_control = 0.0;
    if (have_last_control_time_) {
      dt_control = (now - last_control_time_).seconds();
      if (dt_control < kEpsilon || dt_control > kMaxDt) {
        dt_control = 0.0;
      }
    }
    last_control_time_ = now;
    have_last_control_time_ = true;

    // Guidance lost or path invalid: smoothly ramp speed down to zero.
    if (guidanceTimedOut(now) ||
        !have_guidance_ ||
        latest_guidance_.status != qcar2_msgs_2::msg::LateralGuidance::STATUS_OK) {
      publishSpeed(dt_control, 0.0);
      return;
    }

    // Already latched in stop state: keep publishing zero.
    if (stopped_) {
      publishZeroSpeed();
      return;
    }

    // Stop detection: check whether the vehicle has reached the stop point.
    if (stop_on_penultimate_ && have_stop_u_ &&
        latest_guidance_.curve_id == stop_curve_id_) {
      const ph::Vec2 q{latest_guidance_.closest_point.x, latest_guidance_.closest_point.y};
      const double dist = ph::norm(q - stop_curve_point_ph_);
      const bool should_stop =
        (latest_guidance_.u_star >= stop_u_ && dist <= stop_distance_m_);

      if (should_stop) {
        stopped_ = true;
        publishZeroSpeed();
        RCLCPP_INFO(this->get_logger(), "Vehicle stopped at target location");
        return;
      }
    }

    // Compute longitudinal target speed.
    double target_speed = throttle_test_value_mps_;
    if (speed_control_enabled_ && std::isfinite(latest_guidance_.lookahead_max_curvature_1pm)) {
      target_speed = computeSpeedFromCurvature(latest_guidance_.lookahead_max_curvature_1pm);
    }

    publishSpeed(dt_control, target_speed);
  }

  // Apply accel/decel limiting and EMA smoothing, then publish.
  void publishSpeed(double dt, double target_speed)
  {
    double speed_cmd = last_speed_cmd_;
    if (dt > kEpsilon) {
      const double delta = target_speed - last_speed_cmd_;
      if (delta > 0) {
        const double max_d = max_acceleration_ * dt;
        speed_cmd = (delta > max_d) ? (last_speed_cmd_ + max_d) : target_speed;
      } else {
        const double max_d = max_deceleration_ * dt;
        speed_cmd = (-delta > max_d) ? (last_speed_cmd_ - max_d) : target_speed;
      }
      last_speed_cmd_ = speed_cmd;
    }

    smoothed_speed_ = speed_smooth_alpha_ * speed_cmd +
      (1.0 - speed_smooth_alpha_) * smoothed_speed_;

    std_msgs::msg::Float64 msg;
    msg.data = smoothed_speed_;
    pub_speed_->publish(msg);
  }

  // Immediately publish zero and reset speed smoothing state.
  void publishZeroSpeed()
  {
    last_speed_cmd_ = 0.0;
    smoothed_speed_ = 0.0;

    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    pub_speed_->publish(msg);
  }

  // Parameters
  double control_rate_hz_{200.0};
  double guidance_timeout_sec_{0.5};
  double throttle_test_value_mps_{0.0};
  bool   stop_on_penultimate_{true};
  double stop_distance_m_{0.1};
  int    stop_sample_count_{200};
  bool   speed_control_enabled_{true};
  double speed_max_{0.3};
  double speed_min_{0.3};
  double curvature_low_{0.3};
  double curvature_high_{0.7};
  double max_acceleration_{0.2};
  double max_deceleration_{0.4};
  double speed_smooth_alpha_{0.8};

  // Speed control state
  double last_speed_cmd_{0.0};
  double smoothed_speed_{0.0};
  bool   stopped_{false};

  // PH stop data
  ph::PhPathRuntime ph_runtime_;
  ph::Vec2          stop_curve_point_ph_{0.0, 0.0};

  // Stop state
  uint32_t stop_curve_id_{0};
  bool     have_stop_u_{false};
  double   stop_u_{1.0};

  // Guidance state
  qcar2_msgs_2::msg::LateralGuidance latest_guidance_;
  bool             have_guidance_{false};
  rclcpp::Time     last_guidance_time_{0, 0, RCL_ROS_TIME};
  bool             have_curve_id_{false};
  uint32_t         last_curve_id_{0};

  // Timer state
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool         have_last_control_time_{false};

  // ROS handles
  rclcpp::Subscription<qcar2_msgs_2::msg::LateralGuidance>::SharedPtr sub_guidance_;
  rclcpp::Subscription<qcar2_msgs_2::msg::PhQuinticPath>::SharedPtr   sub_ph_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pub_speed_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongitudinalControllerNode>());
  rclcpp::shutdown();
  return 0;
}

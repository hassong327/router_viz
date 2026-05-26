#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "qcar2_msgs_2/msg/lateral_guidance.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_control_2/lpv_hinf_runtime.hpp"

namespace hinf = qcar2_control_2::lpv_hinf;

static constexpr double kEpsilon     = 1e-6;
static constexpr double kMaxDt       = 1.0;
static constexpr double kMaxSteerRad = 0.52;

class HinfLateralControllerNode : public rclcpp::Node
{
public:
  HinfLateralControllerNode()
  : Node("h_inf_lateral_controller_node")
  {
    // Declare parameters.
    control_rate_hz_        = this->declare_parameter<double>("control_rate_hz", 200.0);
    guidance_timeout_sec_   = this->declare_parameter<double>("guidance_timeout_sec", 0.5);
    controller_dt_          = this->declare_parameter<double>("controller_dt", 0.005);
    K_ff_                   = this->declare_parameter<double>("K_ff", 0.7);
    rho_scale_              = this->declare_parameter<double>("rho_scale", 1.0);
    output_gain_            = this->declare_parameter<double>("output_gain", 0.35);
    delta_max_              = this->declare_parameter<double>("delta_max", 0.52);
    wheelbase_              = this->declare_parameter<double>("wheelbase", 0.256);
    steer_rate_limit_radps_ = this->declare_parameter<double>("steer_rate_limit_radps", 10.0);
    speed_output_limit_     = this->declare_parameter<double>("speed_output_limit", 1.5);
    steer_trim_rad_         = this->declare_parameter<double>("steer_trim_rad", 0.0);

    // v7 cascade outer PI on lateral error (matlab Method 3 / params.sim.{Kp,Ki}_outer).
    // 0 disables the cascade (pure v4 behavior).
    Kp_outer_               = this->declare_parameter<double>("Kp_outer", 0.0);
    Ki_outer_               = this->declare_parameter<double>("Ki_outer", 0.0);
    gear_ratio_         = this->declare_parameter<double>("gear_ratio",         0.0954);
    wheel_radius_       = this->declare_parameter<double>("wheel_radius",       0.033);
    encoder_cpr_        = this->declare_parameter<double>("encoder_cpr",        2880.0);
    raw_encoder_counts_ = this->declare_parameter<bool>  ("raw_encoder_counts", true);

    // JSON path: empty string → use package share default.
    std::string json_path = this->declare_parameter<std::string>(
      "controller_json_path", "");
    if (json_path.empty()) {
      json_path = ament_index_cpp::get_package_share_directory("qcar2_control_2")
                  + "/data/lpv_hinf_qcar2_v3.1.json";
    }

    // Load controller.
    std::string error_msg;
    if (!lpv_hinf_.loadFromJson(json_path, controller_dt_, error_msg)) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to load H-inf controller: %s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    // Configure tuning parameters.
    lpv_hinf_.K_ff        = K_ff_;
    lpv_hinf_.rho_scale   = rho_scale_;
    lpv_hinf_.output_gain = output_gain_;
    lpv_hinf_.delta_max   = delta_max_;
    lpv_hinf_.L           = wheelbase_;

    RCLCPP_INFO(this->get_logger(),
                "Loaded LPV H-inf controller (%d vertices) from %s",
                lpv_hinf_.numVertices(), json_path.c_str());

    // Subscriptions.
    sub_guidance_ = this->create_subscription<qcar2_msgs_2::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10),
      std::bind(&HinfLateralControllerNode::guidanceCallback,
                this, std::placeholders::_1));

    sub_speed_ = this->create_subscription<std_msgs::msg::Float64>(
      "/control/target_speed", rclcpp::QoS(10),
      std::bind(&HinfLateralControllerNode::speedCallback,
                this, std::placeholders::_1));

    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/qcar2_joint", rclcpp::QoS(10),
      std::bind(&HinfLateralControllerNode::jointCallback,
                this, std::placeholders::_1));

    // Publisher.
    pub_cmd_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", rclcpp::QoS(10));

    // Timer.
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1e-3, control_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HinfLateralControllerNode::update, this));

    // Pre-initialise reusable command message.
    cmd_msg_.motor_names = {"steering_angle", "motor_throttle"};
    cmd_msg_.values.resize(2);

    // Shutdown callback: zero commands before exit.
    this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        timer_->cancel();
        publishCmd(0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      });

    RCLCPP_INFO(this->get_logger(), "H-inf lateral controller started");
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
      need_reset_ = true;
    }
  }

  void speedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_target_speed_ = msg->data;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.empty()) return;
    const double v0 = msg->velocity[0];
    if (raw_encoder_counts_) {
      // velocity[0] = encoder counts/sec (QLabs raw)
      latest_measured_speed_ = (v0 / encoder_cpr_) * (2.0 * M_PI) * gear_ratio_ * wheel_radius_;
    } else {
      // velocity[0] = motor shaft angular velocity in rad/s (ROS standard)
      latest_measured_speed_ = v0 * gear_ratio_ * wheel_radius_;
    }
    have_joint_speed_ = true;
  }

  void resetState()
  {
    lpv_hinf_.reset();
    last_steer_cmd_ = 0.0;
    x_int_outer_    = 0.0;     // v7 cascade outer integrator
    need_reset_ = false;
  }

  bool guidanceTimedOut(const rclcpp::Time & now) const
  {
    if (!have_guidance_) {
      return true;
    }
    return (now - last_guidance_time_).seconds() > guidance_timeout_sec_;
  }

  void update()
  {
    const rclcpp::Time now = this->now();

    // dt for rate limiting.
    double dt_control = 0.0;
    if (have_last_control_time_) {
      dt_control = (now - last_control_time_).seconds();
      if (dt_control < kEpsilon || dt_control > kMaxDt) {
        dt_control = 0.0;
      }
    }
    last_control_time_ = now;
    have_last_control_time_ = true;

    // Guidance loss or invalid status → reset and stop.
    if (guidanceTimedOut(now) ||
        !have_guidance_ ||
        latest_guidance_.status != qcar2_msgs_2::msg::LateralGuidance::STATUS_OK) {
      resetState();
      publishCmd(0.0, 0.0);
      return;
    }

    // Longitudinal controller has stopped the vehicle.
    if (std::fabs(latest_target_speed_) < kEpsilon) {
      publishCmd(0.0, 0.0);
      return;
    }

    // Reset controller state on new curve.
    if (need_reset_) {
      resetState();
    }

    const double target_speed = latest_target_speed_;
    const double e_psi_raw = latest_guidance_.heading_error_rad;

    // --- v7 cascade outer PI on lateral error ---
    // /vfg/lateral_guidance::cross_track_error_m is POSITIVE when the
    // vehicle sits to the RIGHT of the path tangent (dot(v1, n_hat)
    // with v1 = closest - vehicle and n_hat the left-perpendicular).
    // The matlab simulation uses the OPPOSITE convention (e_y > 0 =
    // vehicle LEFT). We flip the sign here so Kp_outer / Ki_outer carry
    // the same physical meaning as in qcar2_params.m.
    double e_y = -latest_guidance_.cross_track_error_m;
    if (!std::isfinite(e_y)) {
      e_y = 0.0;
    }
    // Inject the outer-loop bias into e_psi BEFORE the H-inf compute.
    // Equivalent to substituting psi_des → psi_des - Kp*e_y - Ki*int(e_y)
    // because e_psi = psi_des - psi.
    const double e_psi = e_psi_raw
                       - Kp_outer_ * e_y
                       - Ki_outer_ * x_int_outer_;

    // Curvature values (guard against NaN).
    double kappa_sched = latest_guidance_.curvature_1pm;
    if (!std::isfinite(kappa_sched)) {
      kappa_sched = 0.0;
    }
    double kappa_ff = latest_guidance_.ff_lookahead_curvature_1pm;
    if (!std::isfinite(kappa_ff)) {
      kappa_ff = 0.0;
    }

    // rho scheduling: use measured speed from /qcar2_joint; fall back to target_speed if unavailable.
    const double speed_for_rho = have_joint_speed_ ? latest_measured_speed_ : target_speed;

    // H-infinity controller compute (2-channel meas, v4/v7 schema).
    double steer_raw = lpv_hinf_.compute(
      e_psi, kappa_sched, speed_for_rho, kappa_ff);

    // Steering rate limiting.
    double steer_cmd = std::max(-kMaxSteerRad, std::min(kMaxSteerRad, steer_raw));
    if (dt_control > kEpsilon && steer_rate_limit_radps_ > kEpsilon) {
      const double max_delta = steer_rate_limit_radps_ * dt_control;
      const double delta = steer_cmd - last_steer_cmd_;
      if (std::fabs(delta) > max_delta) {
        steer_cmd = last_steer_cmd_ + std::copysign(max_delta, delta);
      }
    }

    last_steer_cmd_ = steer_cmd;
    publishCmd(steer_cmd, target_speed);

    // --- v7 outer-loop integral update with conditional anti-windup ---
    // Pause integration when the steering command is saturated in the
    // direction that further integration would deepen. With e_y > 0
    // (vehicle LEFT), the bias drives the controller to turn RIGHT, so
    // steer_cmd goes NEGATIVE. Windup happens when:
    //   saturated at -delta_max AND e_y > 0  (wants more negative)
    //   saturated at +delta_max AND e_y < 0  (wants more positive)
    if (dt_control > kEpsilon) {
      const bool sat_pos = (steer_cmd >  delta_max_ - kEpsilon);
      const bool sat_neg = (steer_cmd < -delta_max_ + kEpsilon);
      const bool aw_skip = (sat_pos && e_y < 0.0) || (sat_neg && e_y > 0.0);
      if (!aw_skip) {
        x_int_outer_ += dt_control * e_y;
      }
    }
  }

  void publishCmd(double steer, double speed)
  {
    const double steer_out = steer + steer_trim_rad_;
    cmd_msg_.values[0] = std::max(-kMaxSteerRad, std::min(kMaxSteerRad, steer_out));
    cmd_msg_.values[1] = std::max(-speed_output_limit_, std::min(speed_output_limit_, speed));
    pub_cmd_->publish(cmd_msg_);
  }

  // Parameters.
  double control_rate_hz_{200.0};
  double guidance_timeout_sec_{0.5};
  double controller_dt_{0.005};
  double K_ff_{0.0};
  double rho_scale_{1.0};
  double output_gain_{1.0};
  double delta_max_{0.52};
  double wheelbase_{0.256};
  double steer_rate_limit_radps_{10.0};
  double speed_output_limit_{1.0};
  double steer_trim_rad_{0.0};
  double gear_ratio_{0.0954};
  double wheel_radius_{0.033};
  double encoder_cpr_{2880.0};
  bool   raw_encoder_counts_{false};

  // v7 cascade outer PI on lateral error e_y.
  double Kp_outer_{0.0};
  double Ki_outer_{0.0};

  // Controller.
  hinf::LpvHinfRuntime lpv_hinf_;

  // Speed from longitudinal controller.
  double latest_target_speed_{0.0};

  // Measured speed from /qcar2_joint.
  double latest_measured_speed_{0.0};
  bool   have_joint_speed_{false};

  // Steering state.
  double last_steer_cmd_{0.0};

  // v7 cascade outer-loop integral state (integral of e_y).
  double x_int_outer_{0.0};

  // Guidance state.
  qcar2_msgs_2::msg::LateralGuidance latest_guidance_;
  bool             have_guidance_{false};
  rclcpp::Time     last_guidance_time_{0, 0, RCL_ROS_TIME};
  bool             have_curve_id_{false};
  uint32_t         last_curve_id_{0};
  bool             need_reset_{false};

  // Timer state.
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool         have_last_control_time_{false};

  // ROS handles.
  rclcpp::Subscription<qcar2_msgs_2::msg::LateralGuidance>::SharedPtr  sub_guidance_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr            sub_speed_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      sub_joint_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  qcar2_interfaces::msg::MotorCommands cmd_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HinfLateralControllerNode>());
  rclcpp::shutdown();
  return 0;
}

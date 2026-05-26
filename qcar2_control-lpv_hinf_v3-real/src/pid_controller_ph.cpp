#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>

#include "std_msgs/msg/float64.hpp"
#include "qcar2_msgs_2/msg/lateral_guidance.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_control_2/ph_runtime.hpp"

namespace ph = qcar2_control_2::ph;

static constexpr double kEpsilon     = 1e-6;
static constexpr double kMaxDt       = 1.0;
static constexpr double kMaxSteerRad = 0.6;

class PidLateralControllerPHNode : public rclcpp::Node
{
public:
  PidLateralControllerPHNode()
  : Node("pid_lateral_controller_ph_node")
  {
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    kp_ = this->declare_parameter<double>("kp", 0.8);
    ki_ = this->declare_parameter<double>("ki", 0.0);
    kd_ = this->declare_parameter<double>("kd", 0.12);
    kff_ = this->declare_parameter<double>("kff", 0.85);
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.257);
    d_filter_alpha_ = this->declare_parameter<double>("d_filter_alpha", 0.25);
    i_min_ = this->declare_parameter<double>("i_min", -0.5);
    i_max_ = this->declare_parameter<double>("i_max", 0.5);
    steer_rate_limit_radps_ = this->declare_parameter<double>("steer_rate_limit_radps", 10.0);
    guidance_timeout_sec_ = this->declare_parameter<double>("guidance_timeout_sec", 0.5);
    steer_offset_ = this->declare_parameter<double>("steer_offset", -0.13);  // rad, hardware trim

    gain_schedule_enabled_ = this->declare_parameter<bool>("gain_schedule_enabled", true);
    gain_ref_speed_ = this->declare_parameter<double>("gain_ref_speed", 0.3);
    gain_min_speed_ = this->declare_parameter<double>("gain_min_speed", 0.2);

    // Output smoothing for steering (EMA: Exponential Moving Average).
    steer_smooth_alpha_ = this->declare_parameter<double>("steer_smooth_alpha", 0.9);

    sub_guidance_ = this->create_subscription<qcar2_msgs_2::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10),
      std::bind(&PidLateralControllerPHNode::guidanceCallback, this, std::placeholders::_1));

    sub_speed_ = this->create_subscription<std_msgs::msg::Float64>(
      "/control/target_speed", rclcpp::QoS(10),
      std::bind(&PidLateralControllerPHNode::speedCallback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, control_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PidLateralControllerPHNode::update, this));

    cmd_msg_.motor_names = {"steering_angle", "motor_throttle"};
    cmd_msg_.values.resize(2);

    this->get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
      timer_->cancel();
      publishCmd(0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    RCLCPP_INFO(this->get_logger(), "PID PH lateral controller started");
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
      reset_pid_ = true;
    }
  }

  void speedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_target_speed_ = msg->data;
  }

  void resetPidState()
  {
    integrator_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    last_steer_cmd_ = 0.0;
    reset_pid_ = false;
    have_prev_guidance_stamp_ = false;
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
    double dt_control = 0.0;
    if (have_last_control_time_) {
      dt_control = (now - last_control_time_).seconds();
      if (dt_control < kEpsilon || dt_control > kMaxDt) {
        dt_control = 0.0;
      }
    }
    last_control_time_ = now;
    have_last_control_time_ = true;

    if (guidanceTimedOut(now) ||
        !have_guidance_ ||
        latest_guidance_.status != qcar2_msgs_2::msg::LateralGuidance::STATUS_OK) {
      resetPidState();
      publishCmd(0.0, 0.0);
      return;
    }

    // When longitudinal controller has stopped the vehicle, zero out steering too.
    if (std::fabs(latest_target_speed_) < kEpsilon) {
      smoothed_steer_ = 0.0;
      publishCmd(0.0, 0.0);
      return;
    }

    const double e = latest_guidance_.heading_error_rad;
    double dt_pid = dt_control;

    if ((latest_guidance_.header.stamp.sec != 0) ||
        (latest_guidance_.header.stamp.nanosec != 0)) {
      rclcpp::Time stamp(latest_guidance_.header.stamp);
      if (have_prev_guidance_stamp_ && stamp > prev_guidance_stamp_) {
        const double dt = (stamp - prev_guidance_stamp_).seconds();
        if (dt > kEpsilon && dt < kMaxDt) {
          dt_pid = dt;
        }
      }
      prev_guidance_stamp_ = stamp;
      have_prev_guidance_stamp_ = true;
    }

    if (reset_pid_) {
      integrator_ = 0.0;
      prev_error_ = e;
      filtered_derivative_ = 0.0;
      reset_pid_ = false;
    }

    const double target_speed = latest_target_speed_;

    double gain_scale = 1.0;
    if (gain_schedule_enabled_) {
      const double current_speed = std::fabs(target_speed);
      const double effective_speed = std::max(gain_min_speed_, current_speed);
      gain_scale = gain_ref_speed_ / effective_speed;
      gain_scale = ph::clamp(gain_scale, 0.1, 1.1);
    }

    const double p_term = gain_scale * kp_ * e;

    double d_term = 0.0;
    if (dt_pid > kEpsilon) {
      integrator_ += e * dt_pid;
      integrator_ = ph::clamp(integrator_, i_min_, i_max_);

      const double raw_derivative = (e - prev_error_) / dt_pid;
      filtered_derivative_ = d_filter_alpha_ * raw_derivative +
        (1.0 - d_filter_alpha_) * filtered_derivative_;
      d_term = gain_scale * kd_ * filtered_derivative_;
    }
    const double i_term = gain_scale * ki_ * integrator_;

    double ff_term = 0.0;
    if (std::isfinite(latest_guidance_.ff_lookahead_curvature_1pm)) {
      ff_term = kff_ * wheelbase_ * latest_guidance_.ff_lookahead_curvature_1pm;
    }

    const double steer_raw = p_term + i_term + d_term + ff_term;

    double steer_cmd = steer_raw;
    if (dt_control > kEpsilon && steer_rate_limit_radps_ > kEpsilon) {
      const double max_delta = steer_rate_limit_radps_ * dt_control;
      const double delta = steer_cmd - last_steer_cmd_;
      if (std::fabs(delta) > max_delta) {
        steer_cmd = last_steer_cmd_ + std::copysign(max_delta, delta);
      }
    }
    last_steer_cmd_ = steer_cmd;
    prev_error_ = e;

    publishCmd(steer_cmd, target_speed);
  }

  void publishCmd(double steer, double speed)
  {
    smoothed_steer_ = steer_smooth_alpha_ * steer + (1.0 - steer_smooth_alpha_) * smoothed_steer_;
    cmd_msg_.values[0] = smoothed_steer_ + steer_offset_;
    cmd_msg_.values[1] = ph::clamp(speed, -1.0, 1.0);
    pub_cmd_->publish(cmd_msg_);
  }

  double control_rate_hz_{200.0};
  double kp_{0.8};
  double ki_{0.0};
  double kd_{0.12};
  double kff_{0.85};
  double wheelbase_{0.257};
  double d_filter_alpha_{0.25};
  double i_min_{-0.5};
  double i_max_{0.5};
  double steer_rate_limit_radps_{10.0};
  double guidance_timeout_sec_{0.5};

  bool   gain_schedule_enabled_{true};
  double gain_ref_speed_{0.3};
  double gain_min_speed_{0.2};

  // Steering output smoothing.
  double steer_smooth_alpha_{0.9};
  double smoothed_steer_{0.0};
  double steer_offset_{-0.13};

  // Speed from longitudinal controller.
  double latest_target_speed_{0.0};

  rclcpp::Subscription<qcar2_msgs_2::msg::LateralGuidance>::SharedPtr sub_guidance_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           sub_speed_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  qcar2_msgs_2::msg::LateralGuidance latest_guidance_;
  bool             have_guidance_{false};
  rclcpp::Time     last_guidance_time_{0, 0, RCL_ROS_TIME};

  bool     have_curve_id_{false};
  uint32_t last_curve_id_{0};
  bool     reset_pid_{false};

  double integrator_{0.0};
  double prev_error_{0.0};
  double filtered_derivative_{0.0};
  double last_steer_cmd_{0.0};

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool         have_last_control_time_{false};

  rclcpp::Time prev_guidance_stamp_{0, 0, RCL_ROS_TIME};
  bool         have_prev_guidance_stamp_{false};

  qcar2_interfaces::msg::MotorCommands cmd_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidLateralControllerPHNode>());
  rclcpp::shutdown();
  return 0;
}

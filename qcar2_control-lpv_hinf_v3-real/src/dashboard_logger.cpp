#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <qcar2_msgs_2/msg/dashboard_sample.hpp>
#include <qcar2_msgs_2/msg/lateral_guidance.hpp>
#include <qcar2_interfaces/msg/motor_commands.hpp>

using namespace std::chrono_literals;

class DashboardLogger : public rclcpp::Node
{
public:
  DashboardLogger() : Node("dashboard_logger")
  {
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("tf_timeout_sec", 0.1);
    declare_parameter<double>("update_rate_hz", 30.0);

    declare_parameter<double>("wheel_radius", 0.033);
    declare_parameter<double>("gear_ratio", 0.0954);
    declare_parameter<double>("encoder_cpr", 2880.0);

    declare_parameter<std::string>("save_directory", "./acc_plots");
    declare_parameter<bool>("csv_log_enabled", true);

    declare_parameter<std::string>("path_topic", "/planning/local_path_path");
    declare_parameter<std::string>("motor_cmd_topic", "/qcar2_motor_speed_cmd");
    declare_parameter<std::string>("joint_topic", "/qcar2_joint");
    declare_parameter<std::string>("guidance_topic", "/vfg/lateral_guidance");
    declare_parameter<std::string>("sample_topic", "/dashboard/sample");
    declare_parameter<std::string>("end_signal_topic", "stop");
    declare_parameter<std::string>("end_signal_message", "stop3");

    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    tf_timeout_ = get_parameter("tf_timeout_sec").as_double();

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    gear_ratio_ = get_parameter("gear_ratio").as_double();
    encoder_cpr_ = get_parameter("encoder_cpr").as_double();

    save_directory_ = get_parameter("save_directory").as_string();
    csv_log_enabled_ = get_parameter("csv_log_enabled").as_bool();

    const auto path_topic = get_parameter("path_topic").as_string();
    const auto motor_cmd_topic = get_parameter("motor_cmd_topic").as_string();
    const auto joint_topic = get_parameter("joint_topic").as_string();
    const auto guidance_topic = get_parameter("guidance_topic").as_string();
    const auto sample_topic = get_parameter("sample_topic").as_string();
    const auto end_signal_topic = get_parameter("end_signal_topic").as_string();
    end_signal_message_ = get_parameter("end_signal_message").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr) {
        path_received_ = true;
      });

    sub_motor_cmd_ = create_subscription<qcar2_interfaces::msg::MotorCommands>(
      motor_cmd_topic, 10,
      [this](const qcar2_interfaces::msg::MotorCommands::SharedPtr msg) {
        if (msg->motor_names.size() >= 2 && msg->values.size() >= 2) {
          latest_steering_rad_ = msg->values[0];
          latest_target_speed_ = msg->values[1];
          motor_cmd_ok_ = true;
        }
      });

    sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->velocity.empty()) {
          const double enc = msg->velocity[0];
          latest_actual_speed_ =
            (enc / encoder_cpr_) * gear_ratio_ * (2.0 * M_PI) * wheel_radius_;
        }
      });

    sub_guidance_ = create_subscription<qcar2_msgs_2::msg::LateralGuidance>(
      guidance_topic, 10,
      [this](const qcar2_msgs_2::msg::LateralGuidance::SharedPtr msg) {
        latest_cte_m_ = msg->cross_track_error_m;
        guidance_ok_ = true;
      });

    sub_end_signal_ = create_subscription<std_msgs::msg::String>(
      end_signal_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!completed_ && msg->data == end_signal_message_) {
          completed_ = true;
          RCLCPP_INFO(get_logger(),
                      "End-of-run signal received ('%s') — marking completed",
                      msg->data.c_str());
        }
      });

    pub_sample_ = create_publisher<qcar2_msgs_2::msg::DashboardSample>(sample_topic, 10);

    const double rate = get_parameter("update_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { on_timer(); });

    if (csv_log_enabled_) {
      open_csv();
    }

    RCLCPP_INFO(get_logger(), "dashboard_logger started (rate=%.1f Hz)", rate);
  }

  ~DashboardLogger() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  void open_csv()
  {
    try {
      std::filesystem::create_directories(save_directory_);
      const std::time_t t = std::time(nullptr);
      std::tm tm{};
      localtime_r(&t, &tm);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
      const std::string filepath = save_directory_ + "/run_" + buf + ".csv";
      csv_.open(filepath, std::ios::out);
      if (csv_.is_open()) {
        csv_ << "stamp_sec,elapsed_sec,distance_m,x_map,y_map,"
                "target_speed_mps,actual_speed_mps,steering_rad,cte_m,completed\n";
        RCLCPP_INFO(get_logger(), "CSV log: %s", filepath.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to open CSV: %s", filepath.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "CSV setup failed: %s", e.what());
    }
  }

  void on_timer()
  {
    const auto now = this->now();

    bool tf_ok_this_tick = false;
    double x = 0.0;
    double y = 0.0;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        frame_id_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      tf_ok_this_tick = true;
      tf_ok_ = true;
    } catch (const std::exception &) {
      // TF unavailable — still publish so viewer can show waiting state
    }

    if (tf_ok_this_tick) {
      if (!start_time_.has_value()) {
        start_time_ = now;
      }
      if (prev_pos_.has_value()) {
        const double dx = x - prev_pos_->first;
        const double dy = y - prev_pos_->second;
        distance_travelled_ += std::hypot(dx, dy);
      }
      prev_pos_ = std::make_pair(x, y);
    }

    const double elapsed =
      start_time_.has_value() ? (now - *start_time_).seconds() : 0.0;

    qcar2_msgs_2::msg::DashboardSample msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.elapsed_sec = elapsed;
    msg.distance_travelled_m = distance_travelled_;
    msg.vehicle_x_map = x;
    msg.vehicle_y_map = y;
    msg.target_speed_mps = latest_target_speed_;
    msg.actual_speed_mps = latest_actual_speed_;
    msg.steering_rad = latest_steering_rad_;
    msg.cross_track_error_m = latest_cte_m_;
    msg.tf_ok = tf_ok_;
    msg.motor_cmd_ok = motor_cmd_ok_;
    msg.guidance_ok = guidance_ok_;
    msg.completed = completed_;
    pub_sample_->publish(msg);

    if (tf_ok_this_tick && csv_.is_open()) {
      csv_ << std::fixed
           << now.seconds() << ","
           << elapsed << ","
           << distance_travelled_ << ","
           << x << "," << y << ","
           << latest_target_speed_ << ","
           << latest_actual_speed_ << ","
           << latest_steering_rad_ << ","
           << latest_cte_m_ << ","
           << (completed_ ? 1 : 0) << "\n";
      if (++csv_lines_since_flush_ >= 50) {
        csv_.flush();
        csv_lines_since_flush_ = 0;
      }
    }
  }

  // Parameters
  std::string frame_id_;
  std::string base_frame_;
  double tf_timeout_{0.1};
  double wheel_radius_{0.033};
  double gear_ratio_{0.0954};
  double encoder_cpr_{2880.0};
  std::string save_directory_;
  bool csv_log_enabled_{true};
  std::string end_signal_message_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr sub_motor_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<qcar2_msgs_2::msg::LateralGuidance>::SharedPtr sub_guidance_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_end_signal_;

  // Publisher
  rclcpp::Publisher<qcar2_msgs_2::msg::DashboardSample>::SharedPtr pub_sample_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Latest values
  double latest_target_speed_{0.0};
  double latest_actual_speed_{0.0};
  double latest_steering_rad_{0.0};
  double latest_cte_m_{0.0};
  bool path_received_{false};

  // Topic readiness flags (sticky — once true, stays true)
  bool tf_ok_{false};
  bool motor_cmd_ok_{false};
  bool guidance_ok_{false};

  // State
  std::optional<rclcpp::Time> start_time_;
  std::optional<std::pair<double, double>> prev_pos_;
  double distance_travelled_{0.0};
  bool completed_{false};

  // CSV
  std::ofstream csv_;
  int csv_lines_since_flush_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DashboardLogger>());
  rclcpp::shutdown();
  return 0;
}

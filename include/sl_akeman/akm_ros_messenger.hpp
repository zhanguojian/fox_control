#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "akm_control/msg/akm_status.hpp"
#include "sl_akeman/akeman_robot.hpp"

namespace fox {

class AkmRosMessenger {
 public:
  AkmRosMessenger(std::shared_ptr<AkmRobot> robot, rclcpp::Node *node)
      : robot_(std::move(robot)), node_(node) {}

  void Configure(double wheelbase, double track, double max_inner_steer,
                 double max_linear_speed, double command_timeout,
                 std::string base_frame, bool simulated) {
    wheelbase_ = wheelbase;
    track_ = track;
    max_inner_steer_ = max_inner_steer;
    max_linear_speed_ = max_linear_speed;
    command_timeout_ = command_timeout;
    base_frame_ = std::move(base_frame);
    simulated_ = simulated;
  }

  void Setup() {
    status_pub_ =
        node_->create_publisher<akm_control::msg::AKMStatus>("akm_status", 10);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&AkmRosMessenger::OnVelocityCommand, this,
                  std::placeholders::_1));
    last_command_ = node_->now();
  }

  void Update() {
    const auto now = node_->now();

    double linear_velocity = 0.0;
    double central_steering = 0.0;
    AkmCoreState state{};
    AkmActuatorState actuator{};

    ApplyCommandWatchdog(now);
    if (simulated_) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      linear_velocity = commanded_linear_;
      central_steering = commanded_central_steering_;
    } else {
      state = robot_->GetRobotState();
      actuator = robot_->GetActuatorState();
      linear_velocity = state.motion_state.linear_velocity;
      central_steering =
          ConvertInnerToCentral(state.motion_state.steering_angle);
    }

    PublishStatus(now, state, actuator, linear_velocity, central_steering);
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      commanded_linear_ = 0.0;
      commanded_central_steering_ = 0.0;
    }
    if (!simulated_ && robot_) {
      robot_->SetMotionCommand(0.0, 0.0);
    }
  }

 private:
  void OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double linear =
        std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
    double central_steering = 0.0;
    if (std::abs(linear) > 1e-4) {
      central_steering =
          std::atan(wheelbase_ * msg->angular.z / linear);
    }
    const double inner_steering = ConvertCentralToInner(central_steering);

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      commanded_linear_ = linear;
      commanded_central_steering_ = central_steering;
      last_command_ = node_->now();
      watchdog_stopped_ = false;
    }

    if (!simulated_) {
      robot_->SetMotionCommand(linear, inner_steering);
    }
  }

  void ApplyCommandWatchdog(const rclcpp::Time &now) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!watchdog_stopped_ &&
        (now - last_command_).seconds() > command_timeout_) {
      commanded_linear_ = 0.0;
      commanded_central_steering_ = 0.0;
      if (!simulated_) {
        robot_->SetMotionCommand(0.0, 0.0);
      }
      watchdog_stopped_ = true;
      RCLCPP_WARN(node_->get_logger(), "速度指令超时，底盘已停止");
    }
  }

  double ConvertCentralToInner(double angle) const {
    if (std::abs(angle) < 1e-6) {
      return 0.0;
    }
    const double sign = std::copysign(1.0, angle);
    const double absolute = std::abs(angle);
    const double denominator =
        2.0 * wheelbase_ * std::cos(absolute) -
        track_ * std::sin(absolute);
    const double inner =
        std::atan2(2.0 * wheelbase_ * std::sin(absolute), denominator);
    return sign * std::clamp(inner, 0.0, max_inner_steer_);
  }

  double ConvertInnerToCentral(double angle) const {
    if (std::abs(angle) < 1e-6) {
      return 0.0;
    }
    const double sign = std::copysign(1.0, angle);
    const double radius =
        wheelbase_ / std::tan(std::abs(angle)) + track_ / 2.0;
    return sign * std::atan2(wheelbase_, radius);
  }

  void PublishStatus(const rclcpp::Time &now, const AkmCoreState &state,
                     const AkmActuatorState &actuator, double linear,
                     double steering) {
    akm_control::msg::AKMStatus message;
    message.header.stamp = now;
    message.header.frame_id = base_frame_;
    message.linear_velocity = linear;
    message.steering_angle = steering;
    message.vehicle_state = state.system_state.vehicle_state;
    message.control_mode = state.system_state.control_mode;
    message.error_code = state.system_state.error_code;
    message.battery_voltage = state.system_state.battery_voltage;

    for (std::size_t i = 0; i < message.actuator_states.size(); ++i) {
      const auto &high = actuator.actuator_hs_state[i];
      const auto &low = actuator.actuator_ls_state[i];
      auto &target = message.actuator_states[i];
      target.motor_id = static_cast<uint8_t>(i);
      target.rpm = high.rpm;
      target.current = high.current;
      target.pulse_count = high.pulse_count;
      target.driver_voltage = low.driver_voltage;
      target.driver_temperature = low.driver_temp;
      target.motor_temperature = static_cast<int8_t>(low.motor_temp);
      target.driver_state = low.driver_state;
    }
    status_pub_->publish(message);
  }

  std::shared_ptr<AkmRobot> robot_;
  rclcpp::Node *node_;
  rclcpp::Publisher<akm_control::msg::AKMStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::mutex command_mutex_;
  rclcpp::Time last_command_;
  double commanded_linear_ = 0.0;
  double commanded_central_steering_ = 0.0;
  bool watchdog_stopped_ = false;

  double wheelbase_ = 0.55;
  double track_ = 0.52;
  double max_inner_steer_ = 0.75;
  double max_linear_speed_ = 1.5;
  double command_timeout_ = 0.5;
  std::string base_frame_ = "base_link";
  bool simulated_ = false;
};

}  // namespace fox

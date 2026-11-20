/*
 * Akm_robot.hpp
 *
 * Created on: nov 19, 2026 10:59
 * Description:
 *
 * Copyright (c) 
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "../robot_common_interface.hpp"
#include "akeman_interface.hpp"

namespace fox {
class AkmRobot : public RobotCommonInterface, public AkmInterface {
 public:
  explicit AkmRobot(
      ProtocolVersion protocol = ProtocolVersion::SONGLING);
  ~AkmRobot() override = default;

  bool Connect(std::string can_name) override;

  std::string RequestVersion(int timeout_sec = 3) override;

  void EnableCommandedMode() override;

  void ActivateBrake() override;
  void ReleaseBrake() override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;

  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // get robot state
  AkmCoreState GetRobotState() override;
  AkmActuatorState GetActuatorState() override;
  AkmCommonSensorState GetCommonSensorState() override;

 private:
  std::unique_ptr<RobotCommonInterface> robot_;
};

class AkmBaseRos : public rclcpp::Node {
 public:
  AkmBaseRos(std::string node_name, std::string config_path);

  bool Initialize();
  void Run();
  void Stop();

 private:
  
  std::string port_name_;
  std::string base_frame_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  double wheelbase_ = 0.55;
  double track_ = 0.52;
  double max_steer_angle_ = 0.75;
  double max_linear_speed_ = 1.5;
  double command_timeout_ = 0.5;
  bool config_loaded_ = false;
  
  std::shared_ptr<AkmRobot> robot_;

  std::atomic<bool> keep_running_;

  bool LoadParameters(const std::string &config_path);
};











}  // namespace fox


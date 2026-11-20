#include "sl_akeman/akeman_robot.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "sl_akeman/akeman_base.hpp"
#include "sl_akeman/akm_ros_messenger.hpp"

namespace fox {

AkmRobot::AkmRobot(ProtocolVersion protocol) {
  if (protocol != ProtocolVersion::SONGLING) {
    throw std::invalid_argument("不支持的底盘协议");
  }
  robot_ = std::make_unique<AkmBase>();
}

bool AkmRobot::Connect(std::string can_name) {
  return robot_->Connect(std::move(can_name));
}

std::string AkmRobot::RequestVersion(int timeout_sec) {
  return robot_->RequestVersion(timeout_sec);
}

void AkmRobot::EnableCommandedMode() {
  robot_->EnableCommandedMode();
}

void AkmRobot::ResetRobotState() {
  robot_->ResetRobotState();
}

ProtocolVersion AkmRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void AkmRobot::SetMotionCommand(double linear_vel, double steering_angle) {
  dynamic_cast<AkmInterface &>(*robot_)
      .SetMotionCommand(linear_vel, steering_angle);
}

void AkmRobot::ActivateBrake() {
  dynamic_cast<AkmInterface &>(*robot_).ActivateBrake();
}

void AkmRobot::ReleaseBrake() {
  dynamic_cast<AkmInterface &>(*robot_).ReleaseBrake();
}

AkmCoreState AkmRobot::GetRobotState() {
  return dynamic_cast<AkmInterface &>(*robot_).GetRobotState();
}

AkmActuatorState AkmRobot::GetActuatorState() {
  return dynamic_cast<AkmInterface &>(*robot_).GetActuatorState();
}

AkmCommonSensorState AkmRobot::GetCommonSensorState() {
  return dynamic_cast<AkmInterface &>(*robot_).GetCommonSensorState();
}

AkmBaseRos::AkmBaseRos(std::string node_name, std::string config_path)
    : rclcpp::Node(std::move(node_name)), keep_running_(false) {
  config_loaded_ = LoadParameters(config_path);
}

bool AkmBaseRos::LoadParameters(const std::string &config_path) {
  try {
    const YAML::Node document = YAML::LoadFile(config_path);
    const YAML::Node config = document["akeman"]["ros__parameters"];
    if (!config || !config.IsMap()) {
      RCLCPP_ERROR(get_logger(), "配置文件缺少 akeman.ros__parameters");
      return false;
    }

    port_name_ = config["port_name"].as<std::string>("can0");
    base_frame_ = config["base_frame"].as<std::string>("base_link");
    simulated_robot_ = config["simulated_robot"].as<bool>(false);
    sim_control_rate_ = config["control_rate"].as<int>(50);
    wheelbase_ = config["wheelbase"].as<double>(0.55);
    track_ = config["track"].as<double>(0.52);
    max_steer_angle_ = config["max_steer_angle"].as<double>(0.75);
    max_linear_speed_ = config["max_linear_speed"].as<double>(1.5);
    command_timeout_ = config["command_timeout"].as<double>(0.5);
  } catch (const YAML::Exception &error) {
    RCLCPP_ERROR(get_logger(), "读取配置文件失败：%s", error.what());
    return false;
  }

  // 配置文件提供默认值，命令行 ROS 参数仍可按需覆盖。
  declare_parameter("port_name", port_name_);
  declare_parameter("base_frame", base_frame_);
  declare_parameter("simulated_robot", simulated_robot_);
  declare_parameter("control_rate", sim_control_rate_);
  declare_parameter("wheelbase", wheelbase_);
  declare_parameter("track", track_);
  declare_parameter("max_steer_angle", max_steer_angle_);
  declare_parameter("max_linear_speed", max_linear_speed_);
  declare_parameter("command_timeout", command_timeout_);

  port_name_ = get_parameter("port_name").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  simulated_robot_ = get_parameter("simulated_robot").as_bool();
  sim_control_rate_ = get_parameter("control_rate").as_int();
  wheelbase_ = get_parameter("wheelbase").as_double();
  track_ = get_parameter("track").as_double();
  max_steer_angle_ = get_parameter("max_steer_angle").as_double();
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  command_timeout_ = get_parameter("command_timeout").as_double();

  RCLCPP_INFO(get_logger(), "已读取配置文件：%s", config_path.c_str());
  return true;
}

bool AkmBaseRos::Initialize() {
  if (!config_loaded_) {
    return false;
  }
  if (sim_control_rate_ <= 0) {
    RCLCPP_ERROR(get_logger(), "control_rate 必须大于 0");
    return false;
  }

  robot_ = std::make_shared<AkmRobot>();
  if (!simulated_robot_ && !robot_->Connect(port_name_)) {
    RCLCPP_ERROR(get_logger(), "无法连接 CAN 接口 %s", port_name_.c_str());
    robot_.reset();
    return false;
  }

  if (!simulated_robot_) {
    robot_->EnableCommandedMode();
  }
  RCLCPP_INFO(get_logger(), "AKM 底盘初始化完成，模式：%s",
              simulated_robot_ ? "仿真" : "实车");
  return true;
}

void AkmBaseRos::Run() {
  if (!robot_) {
    RCLCPP_ERROR(get_logger(), "底盘尚未初始化");
    return;
  }

  AkmRosMessenger messenger(robot_, this);
  messenger.Configure(
      wheelbase_, track_, max_steer_angle_, max_linear_speed_,
      command_timeout_, base_frame_, simulated_robot_);
  messenger.Setup();

  rclcpp::WallRate rate(sim_control_rate_);
  keep_running_ = true;
  while (rclcpp::ok() && keep_running_) {
    rclcpp::spin_some(shared_from_this());
    if (!rclcpp::ok() || !keep_running_) {
      break;
    }
    messenger.Update();
    rate.sleep();
  }
  messenger.Stop();
}

void AkmBaseRos::Stop() {
  keep_running_ = false;
  if (robot_) {
    robot_->SetMotionCommand(0.0, 0.0);
  }
}

}  // namespace fox

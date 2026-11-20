/*
 * Akm_robot.cpp
 *
 * Created on: nov 20, 2026 23:30
 * Description:
 *
 * Copyright (c) 2026
 */

#if 0  // 旧版草稿保留用于迁移记录，当前实现位于 akm_robot.cpp。
#include "ugv_sdk/mobile_robot/Akm_robot.hpp"
#include "ugv_sdk/details/robot_base/Akm_base.hpp"

namespace fox {
AkmRobot::AkmRobot(ProtocolVersion protocol) {
  // if (protocol == ProtocolVersion::AGX_V1) {
  //   robot_ = new AkmBaseV1();
  // } else if (protocol == ProtocolVersion::AGX_V2) {
  if(protocol == SONGLING)
  {
    robot_ = new AkmBase();
  }
}

AkmRobot::~AkmRobot() {
  if (robot_) delete robot_;
}

void AkmRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

// std::string AkmRobot::RequestVersion(int timeout_sec) {
//     return robot_->RequestVersion(timeout_sec);
// }

bool AkmRobot::Connect(std::string can_name) {
  return robot_->Connect(can_name);
}

void AkmRobot::ResetRobotState() { robot_->ResetRobotState(); }

// ProtocolVersion AkmRobot::GetParserProtocolVersion() {
//   return robot_->GetParserProtocolVersion();
// }

void AkmRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  Akm->SetMotionCommand(linear_vel, angular_vel);
}

void AkmRobot::ActivateBrake() {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  Akm->ActivateBrake();
}

void AkmRobot::ReleaseBrake() {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  Akm->ReleaseBrake();
}

AkmCoreState AkmRobot::GetRobotState() {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  return Akm->GetRobotState();
}

AkmActuatorState AkmRobot::GetActuatorState() {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  return Akm->GetActuatorState();
}
AkmCommonSensorState AkmRobot::GetCommonSensorState() {
  auto Akm = dynamic_cast<AkmInterface*>(robot_);
  return Akm->GetCommonSensorState();
}

HunterBaseRos::HunterBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));   //声明参数

  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));

  this->declare_parameter("robot_model", rclcpp::ParameterValue("akeman"));

  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  LoadParameters();
}

void HunterBaseRos::LoadParameters() 
{
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");//获取参数

  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");


  this->get_parameter_or<std::string>("robot_model", robot_model_, "hunter2");
  

  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool HunterBaseRos::Initialize() {

  std::cout << "Robot base: AKeMan" << std::endl;

  if (protocol == ProtocolVersion::SONGLING) 
  {
    std::cout << "Detected protocol: SONGLING" << std::endl;
    robot_ = std::unique_ptr<AKMRobot>(new AKMRobot(ProtocolVersion::SONGLING));
  } else if {
    std::cout << "Detected protocol: UNKONWN" << std::endl;
    return false;
  }
  return true;
}

void HunterBaseRos::Stop() { keep_running_ = false; }


void HunterBaseRos::Run() 
{

  // instantiate a ROS messenger
    std::unique_ptr<HunterMessenger<AKMRobot>> messenger =
        std::unique_ptr<AKMMessenger<HunterRobot>>(
            new AKMMessenger<AKMRobot>(robot_, this));

    if(robot_model_=="AKM")
    {
      messenger->SetTrack(AKMParams::track);
      messenger->SetWeelbase(AKMParams::wheelbase);
      messenger->SetMaxSteerAngleCentral(AKMParams::max_steer_angle_central);
      messenger->SetMaxSteerAngle(AKMParams::max_steer_angle);
      
    }
   
    messenger->SetBaseFrame(base_frame_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        // std::cout << "EnableCommandedMode" << std::endl;
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    rclcpp::Rate rate(50);
    keep_running_ = true;
    while (keep_running_) {
      messenger->PublishStateToROS();
      // robot_->EnableCommandedMode();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    // }
  }
}


}  // namespace fox
#endif
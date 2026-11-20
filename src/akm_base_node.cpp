/*
 * akm_base_node.cpp
 *
 * Created on: nov 20, 2026 16:20
 * Description:
 *
 * Copyright 
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sl_akeman/akeman_robot.hpp"

using namespace fox;

namespace {

    const char* home = std::getenv("HOME");
    std::string config_path = std::string(home) + "/fox/control/config/akm_params.yaml";
    const char* kConfigPath = config_path.c_str();

}  // namespace

std::shared_ptr<AkmBaseRos> robot;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  robot = std::make_shared<AkmBaseRos>("akeman", kConfigPath);
  if (robot->Initialize()) {
    robot->Run();
  } else {
    robot.reset();
    rclcpp::shutdown();
    return 1;
  }

  robot.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
/*
 * fox_base.hpp
 *
 * Created on: nov 19, 2026 23:23
 * Description:
    继承底盘对akman底盘细化实现
 * Copyright (c)
 */

#pragma once

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "akeman_interface.hpp"
#include "../robot_base.hpp"
#include "../protocol/sl_protocol_parser.hpp"

namespace fox {

class AkmBase : public RobotBase<ProtocolParser>, public AkmInterface {
 public:
  AkmBase() = default;
  ~AkmBase() override = default;

  bool Connect(std::string can_name) override {
    return RobotBase::Connect(std::move(can_name));
  }

  void SetMotionCommand(double linear_vel, double steering_angle) override {
    SendMotionCommand(linear_vel, 0.0, 0.0, steering_angle);
  }

  AkmCoreState GetRobotState() override {
    const auto state = GetRobotCoreStateMsgGroup();
    AkmCoreState result{};
    result.time_stamp = state.time_stamp;
    result.system_state = state.system_state;
    result.motion_state = state.motion_state;
    result.rc_state = state.rc_state;
    return result;
  }

  AkmActuatorState GetActuatorState() override {
    const auto actuator = GetActuatorStateMsgGroup();
    AkmActuatorState result{};
    result.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 3; ++i) {
      result.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      result.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    }
    return result;
  }

  AkmCommonSensorState GetCommonSensorState() override {
    const auto state = GetCommonSensorStateMsgGroup();
    AkmCommonSensorState result{};
    result.time_stamp = state.time_stamp;
    result.bms_basic_state = state.bms_basic_state;
    result.bms_extend_state = state.bms_extend_state;
    return result;
  }

  void ActivateBrake() override {
    SetBrakeMode(BRAKE_MODE_LOCK);
  }

  void ReleaseBrake() override {
    SetBrakeMode(BRAKE_MODE_UNLOCK);
  }
};
}  // namespace fox


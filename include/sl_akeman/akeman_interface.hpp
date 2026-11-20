/*
 * Akm_interface.hpp
 *
 * Created on: nov 19, 2026 
 * Description:
 * Akm 状态怎样组织：核心状态、电机状态、电池状态
 * Akm 应提供哪些操作：设置速度和转向、控制刹车、读取状态
 * Copyright (c) 
 */

#pragma once

#include <string>

#include "protocol/message_struct.h"
#include "../robot_common_interface.hpp"

namespace fox {

// core state 把底盘核心状态放在一起.状态组使用的更新时间;系统状态、电压、错误码,等;实际速度、转向角等;遥控器开关和摇杆状态
struct AkmCoreState {
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  RcStateMessage rc_state;
};

//保存执行器状态 , 高频状态 和低频状态,
struct AkmActuatorState {
  SdkTimePoint time_stamp;

  // actuator state
  ActuatorHSStateMessage actuator_hs_state[3];
  ActuatorLSStateMessage actuator_ls_state[3];
  // - for v1 robots only
  // ActuatorStateMessageV1 actuator_state[3];
};

//把电池基本状态与扩展状态组合起来 .	SOC、SOH、电压、电流、温度和电池告警和警告状态字节
struct AkmCommonSensorState {
  SdkTimePoint time_stamp;

  BmsBasicMessage bms_basic_state;
  BmsExtendedMessage bms_extend_state;
};

struct AkmInterface {
  virtual ~AkmInterface() = default;

  //设置运动命令：这里明确写的是转向角
  virtual void SetMotionCommand(double linear_vel, double steering_angle) = 0;

  //刹车接口
  virtual void ActivateBrake() = 0;
  virtual void ReleaseBrake() = 0;

  // get robot state
  virtual AkmCoreState GetRobotState() = 0;
  virtual AkmActuatorState GetActuatorState() = 0;
  virtual AkmCommonSensorState GetCommonSensorState() = 0;
};
}  // namespace fox


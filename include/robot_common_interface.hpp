/*
 * robot_interface.hpp
 *
 * Created on: Nov 19, 2026 11:48
 * Description:
 * SDK 中的机器人对象，应当提供连接、切换控制模式、查询版本等公共功能。
 * Copyright (c) 2026 
 */

#pragma once

#include <string>
#include <chrono>
#include <stdexcept>

#include "protocol/message_struct.h"
#include "protocol/parser_base.h"

#define MAX_ACTUATOR_NUM 8

namespace fox {
using SdkClock = std::chrono::steady_clock;
using SdkTimePoint = std::chrono::time_point<SdkClock>;

class RobotCommonInterface {
 public:
  virtual ~RobotCommonInterface() = default;

  // functions to be implemented by class Base
  virtual void EnableCommandedMode() = 0;
  virtual std::string RequestVersion(int timeout_sec) = 0;

  // functions to be implemented by each robot class
  virtual bool Connect(std::string can_name) = 0;

  virtual void ResetRobotState() = 0;

  virtual ProtocolVersion GetParserProtocolVersion() = 0;
};
}  // namespace fox


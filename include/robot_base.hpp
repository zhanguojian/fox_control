/*
 * robot_base.hpp
 *
 * Created on: nov 19, 2026 17:14
 * Description:
 * robotBase 负责把这些模块组织起来：发送时构造消息并调用编码器，接收时调用解码器并保存状态
 * Copyright (c) 
 */


#pragma once
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <chrono>

#include "async_port/asybc_can.hpp"
#include "robot_common_interface.hpp"
#include "protocol/parser_base.h"

namespace fox {
  
// 核心状态组;   系统状态。运动状态。灯光状态。运动模式。遥控器状态。
struct CoreStateMsgGroup {
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  MotionModeStateMessage motion_mode_state;
  RcStateMessage rc_state;
};

//执行器状态组：保存执行器状态 , 高频状态 和低频状态,
struct ActuatorStateMsgGroup {
  SdkTimePoint time_stamp;

  ActuatorHSStateMessage actuator_hs_state[MAX_ACTUATOR_NUM]{};
  ActuatorLSStateMessage actuator_ls_state[MAX_ACTUATOR_NUM]{};

  MotorAngleMessage motor_angles;  // ranger only
  MotorSpeedMessage motor_speeds;  // ranger only
};

//传感器状态组：保存电池基本信息和扩展状态。
struct CommonSensorStateMsgGroup {
  SdkTimePoint time_stamp;
  BmsBasicMessage bms_basic_state;
  BmsExtendedMessage bms_extend_state;
};

template <typename ParserType>
class RobotBase : public RobotCommonInterface {
 public:
  RobotBase() {
    static_assert(std::is_base_of<
                      ParserBase<ProtocolVersion::SONGLING>,
                      ParserType>::value,
                  "ParserType 必须继承松灵协议解析器基类");
  };
  virtual ~RobotBase() { DisconnectPort(); }

  // do not allow copy or assignment
  RobotBase(const RobotBase &) = delete;
  RobotBase &operator=(const RobotBase &) = delete;


  //把接收函数交给 CAN 通信对象
  bool Connect(std::string can_name) override {
    return ConnectPort(can_name,
                       [this](can_frame *rx_frame)
                                    { ParseCANFrame(rx_frame); }
                      );
                                 
  }

  // switch to commanded mode,构造 CAN 控制模式请求
  void EnableCommandedMode() {
    // construct message
    Message msg;
    msg.type = MsgControlModeConfig;
    msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

    // encode msg to can frame and send to bus
    if (can_ != nullptr && can_->IsOpened()) {    //采用短路判断：如果指针为空，就不执行后面的成员函数调用。
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  //构造运动指令, must be called at a frequency >= 50Hz,表示要求上层持续调用，50 Hz 对应约 20 ms 一个周期
  void SendMotionCommand(double linear_vel, double angular_vel,
                         double lateral_vel, double steering_angle) 
  {
    if (can_ != nullptr && can_->IsOpened()) {
      // motion control message
      Message msg;
      if (parser_.GetParserProtocolVersion() == ProtocolVersion::SONGLING)
      {
            msg.type = MsgMotionCommand;
            msg.body.motion_command_msg.linear_velocity = linear_vel;
            msg.body.motion_command_msg.angular_velocity = angular_vel;
            msg.body.motion_command_msg.lateral_velocity = lateral_vel;
            msg.body.motion_command_msg.steering_angle = steering_angle;
      } else
      {
            std::cerr << "未知的底盘协议版本" << std::endl;
            return;
      }

      //   std::cout << "Sending motion cmd: " << linear_vel << ", " <<
      //   angular_vel
      //             << ", " << lateral_vel << std::endl;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // one-shot light command
  void SendLightCommand(AgxLightMode front_mode, uint8_t front_custom_value,
                        AgxLightMode rear_mode, uint8_t rear_custom_value) {
    if (can_ != nullptr && can_->IsOpened()) {
      Message msg;
      msg.type = MsgLightCommand;
      msg.body.light_command_msg.enable_cmd_ctrl = true;
      msg.body.light_command_msg.front_light.mode = front_mode;
      msg.body.light_command_msg.front_light.custom_value = front_custom_value;
      msg.body.light_command_msg.rear_light.mode = rear_mode;
      msg.body.light_command_msg.rear_light.custom_value = rear_custom_value;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  void DisableLightControl() {
    if (can_ != nullptr && can_->IsOpened()) {
      Message msg;
      msg.type = MsgLightCommand;

      msg.body.light_command_msg.enable_cmd_ctrl = false;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // motion mode change
  void SetMotionMode(uint8_t mode) {
    if (can_ != nullptr && can_->IsOpened()) {
      Message msg;
      msg.type = MsgSetMotionModeCommand;
      msg.body.motion_mode_msg.motion_mode = mode;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  void ResetRobotState() override {}

  ProtocolVersion GetParserProtocolVersion() override {
    return parser_.GetParserProtocolVersion();
  }

  CoreStateMsgGroup GetRobotCoreStateMsgGroup() {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    return core_state_msgs_;
  }

  ActuatorStateMsgGroup GetActuatorStateMsgGroup() {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    return actuator_state_msgs_;
  }

  CommonSensorStateMsgGroup GetCommonSensorStateMsgGroup() {
    std::lock_guard<std::mutex> guard(common_sensor_state_mtx_);
    return common_sensor_state_msgs_;
  }

 protected:
  ParserType parser_;

  // divide feedback messages into smaller groups to avoid the
  // state mutex being locked for too often such that accessing
  // the data become difficult

  /* feedback group 1: core state */
  std::mutex core_state_mtx_;
  CoreStateMsgGroup core_state_msgs_;

  /* feedback group 2: actuator state */
  std::mutex actuator_state_mtx_;
  ActuatorStateMsgGroup actuator_state_msgs_;

  /* feedback group 3: common sensor */
  std::mutex common_sensor_state_mtx_;
  CommonSensorStateMsgGroup common_sensor_state_msgs_;

  std::mutex version_str_buf_mtx_;
  std::string version_string_buffer_;

  // communication interface
  std::shared_ptr<AsyncCAN> can_;




  // connect to roboot from CAN or serial,实际打开 CAN
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;

  bool ConnectPort(std::string dev_name, CANFrameRxCallback cb) {
    //创建 CAN 通信对象。
    can_ = std::make_shared<AsyncCAN>(dev_name);
    //保存回调。
    can_->SetReceiveCallback(cb);
    //打开接口并启动对应接收流程。
    return can_->Open();
  }

  void DisconnectPort() {
    if (can_ != nullptr && can_->IsOpened()) can_->Close();
  }

  void SetBrakeMode(AgxBrakeMode mode) {
    // construct message
    Message msg;
    msg.type = MsgBrakeModeConfig;
    msg.body.brake_mode_config_msg.mode = mode;

    // encode msg to can frame and send to bus
    if (can_ != nullptr && can_->IsOpened()) {
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  std::string RequestVersion(int timeout_sec) {
    // clear buffer first
    version_string_buffer_.clear();

    // send request msg
    can_frame frame;
    frame.can_id = ((uint32_t)0x4a1);
    frame.can_dlc = 1;
    frame.data[0] = 0x01;
    can_->SendFrame(frame);

    // wait for response msgs
    using Clock = std::chrono::steady_clock;

    std::string version_string;
    auto start_time = Clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(Clock::now() -
                                                            start_time)
               .count() < timeout_sec) {
      // copy version string received from CAN
      {
        std::lock_guard<std::mutex> lock(version_str_buf_mtx_);
        version_string = version_string_buffer_;
      }
      if (version_string.size() >= 80) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // set version string to empty if not enough bytes received
    if (version_string.size() < 80) version_string = "";

    return version_string;
  }

  void ParseCANFrame(can_frame *rx_frame) {
    Message status_msg;

    //把 CAN 原始字节转换成 SDK 消息。
    if (parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateRobotCoreState(status_msg);
      UpdateActuatorState(status_msg);
      UpdateCommonSensorState(status_msg);
      // UpdateResponseVersion(status_msg);
    }
  }

  void UpdateRobotCoreState(const Message &status_msg) {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case MsgSystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }
      case MsgMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case MsgLightState: {
        // std::cout << "light control feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.light_state = status_msg.body.light_state_msg;
        break;
      }
      case MsgMotionModeState: {
        // std::cout << "motion mode feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.motion_mode_state =
            status_msg.body.motion_mode_state_msg;
        break;
      }
      case MsgRcState: {
        // std::cout << "rc feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateActuatorState(const Message &status_msg) {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    actuator_state_msgs_.time_stamp = SdkClock::now();
    switch (status_msg.type) {
      case MsgMotorAngle: {
        actuator_state_msgs_.motor_angles.angle_5 =
            status_msg.body.motor_angle_msg.angle_5;
        actuator_state_msgs_.motor_angles.angle_6 =
            status_msg.body.motor_angle_msg.angle_6;
        actuator_state_msgs_.motor_angles.angle_7 =
            status_msg.body.motor_angle_msg.angle_7;
        actuator_state_msgs_.motor_angles.angle_8 =
            status_msg.body.motor_angle_msg.angle_8;
        break;
      }
      case MsgMotorSpeed: {
        actuator_state_msgs_.motor_speeds.speed_1 =
            status_msg.body.motor_speed_msg.speed_1;
        actuator_state_msgs_.motor_speeds.speed_2 =
            status_msg.body.motor_speed_msg.speed_2;
        actuator_state_msgs_.motor_speeds.speed_3 =
            status_msg.body.motor_speed_msg.speed_3;
        actuator_state_msgs_.motor_speeds.speed_4 =
            status_msg.body.motor_speed_msg.speed_4;
        break;
      }
      case MsgActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        const auto motor_id = status_msg.body.actuator_hs_state_msg.motor_id;
        if (motor_id < MAX_ACTUATOR_NUM) {
          actuator_state_msgs_.actuator_hs_state[motor_id] =
              status_msg.body.actuator_hs_state_msg;
        }
        break;
      }
      case MsgActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        const auto motor_id = status_msg.body.actuator_ls_state_msg.motor_id;
        if (motor_id < MAX_ACTUATOR_NUM) {
          actuator_state_msgs_.actuator_ls_state[motor_id] =
              status_msg.body.actuator_ls_state_msg;
        }
        break;
      }
      default:
        break;
    }
  }

  void UpdateCommonSensorState(const Message &status_msg) {
    std::lock_guard<std::mutex> guard(common_sensor_state_mtx_);
    //    std::cout << common_sensor_state_msgs_.bms_basic_state.battery_soc<<
    //    std::endl;
    switch (status_msg.type) {
      case MsgBmsBasic: {
        //      std::cout << "system status feedback received" << std::endl;
        common_sensor_state_msgs_.time_stamp = SdkClock::now();
        common_sensor_state_msgs_.bms_basic_state =
            status_msg.body.bms_basic_msg;
        break;
      }
      case MsgBmsExtended: {
        common_sensor_state_msgs_.bms_extend_state = 
          status_msg.body.bms_extended_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateResponseVersion(const Message &status_msg) {
    switch (status_msg.type) {
      case MsgVersionResponse: {
        std::lock_guard<std::mutex> lock(version_str_buf_mtx_);
        for (int i = 0; i < 8; i++) {
          uint8_t data = status_msg.body.version_response_msg.bytes[i];
          if (data < 32 || data > 126) data = 32;
          version_string_buffer_ += data;
        }
        break;
      }   
      default:
        break;
    }
  }
};
}  // namespace fox


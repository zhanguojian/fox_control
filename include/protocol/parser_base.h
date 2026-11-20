/*
 * parser_interface.hpp
 *
 * Created on: nov 19, 2026 14:43
 * Description:
 * 编码、解码、计算校验和获取协议版本的接口类
 * Copyright (c) 2026 
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

#include "message_struct.h"

namespace fox {

enum class ProtocolVersion { UNKNOWN, SONGLING };

template <ProtocolVersion VersionNumber = ProtocolVersion::SONGLING>
class ParserBase {
 public:
  virtual ~ParserBase() = default;

  // 解码 CAN 帧为统一消息。
  virtual bool DecodeMessage(const struct can_frame *rx_frame,
                             Message *msg) = 0;

  // 将统一消息编码成 CAN 帧。
  virtual bool EncodeMessage(const Message *msg,
                             struct can_frame *tx_frame) = 0;

  // 计算 CAN 帧校验值。
  virtual uint8_t CalculateChecksum(uint16_t id, uint8_t *data,
                                    uint8_t dlc) = 0;

  ProtocolVersion GetParserProtocolVersion() const { return VersionNumber; }
};

}  // namespace fox

/*
 * sl_protocol.hpp
 *
 * Created on: nov 11, 2026 14:51
 * Description:
 * akeman 底盘协议解析类
 * Copyright (c) 
 */

#pragma once

#include "parser_base.h"
#include "msg_praser.h"

namespace fox {
class ProtocolParser : public ParserBase<ProtocolVersion::SONGLING>
{
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     Message *msg) override;
  bool EncodeMessage(const Message *msg,
                     struct can_frame *tx_frame) override;
  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override;
};


bool ProtocolParser::DecodeMessage(const struct can_frame *rx_frame,
                                   Message *msg) {
  return DecodeCanFrame(rx_frame, msg);
}

bool ProtocolParser::EncodeMessage(const Message *msg,
                                   struct can_frame *tx_frame) {
  return EncodeCanFrame(msg, tx_frame);
}

uint8_t ProtocolParser::CalculateChecksum(uint16_t id, uint8_t *data,
                                            uint8_t dlc) {
  return CalcCanFrameChecksum(id, data, dlc);
}




















}  // namespace fox


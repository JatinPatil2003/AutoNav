#pragma once

#include <cstddef>
#include <cstdint>

#include "autonav_protocol/protocol.h"

namespace autonav_protocol {

class FrameEncoder {
 public:
  // Writes SOF1+SOF2+CMD+LEN+payload+CRC8 into out_buf.
  // Returns the number of bytes written, or 0 if len exceeds kMaxPayloadLen
  // or out_buf_capacity is too small to hold the resulting frame.
  static size_t Encode(CommandId cmd, const uint8_t* payload, uint8_t len,
                       uint8_t* out_buf, size_t out_buf_capacity);
};

}  // namespace autonav_protocol

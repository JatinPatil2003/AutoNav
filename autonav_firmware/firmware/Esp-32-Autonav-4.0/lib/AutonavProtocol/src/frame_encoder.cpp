#include "autonav_protocol/frame_encoder.h"

namespace autonav_protocol {

size_t FrameEncoder::Encode(CommandId cmd, const uint8_t* payload, uint8_t len,
                            uint8_t* out_buf, size_t out_buf_capacity) {
  if (len > kMaxPayloadLen) {
    return 0;
  }
  const size_t frame_size = static_cast<size_t>(len) + 5;
  if (out_buf_capacity < frame_size) {
    return 0;
  }

  out_buf[0] = kStartOfFrame1;
  out_buf[1] = kStartOfFrame2;
  out_buf[2] = static_cast<uint8_t>(cmd);
  out_buf[3] = len;
  for (uint8_t i = 0; i < len; ++i) {
    out_buf[4 + i] = payload[i];
  }
  out_buf[4 + len] = Crc8(&out_buf[2], static_cast<size_t>(len) + 2);

  return frame_size;
}

}  // namespace autonav_protocol

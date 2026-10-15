#ifndef AUTONAV_FIRMWARE__ESP32_FRAME_CODEC_HPP_
#define AUTONAV_FIRMWARE__ESP32_FRAME_CODEC_HPP_

#include <cstddef>
#include <cstdint>

#include "autonav_firmware/esp32_protocol.hpp"

namespace autonav_firmware
{

class Esp32FrameEncoder
{
public:
  // Writes SOF1+SOF2+CMD+LEN+payload+CRC8 into out_buf.
  // Returns the number of bytes written, or 0 if len exceeds
  // esp32_protocol::MAX_PAYLOAD_LEN or out_buf_capacity is too small.
  static size_t encode(
    esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len,
    uint8_t * out_buf, size_t out_buf_capacity);
};

// Byte-at-a-time streaming decoder. On CRC or length error the frame is
// dropped and the state machine resyncs from WAIT_SOF1 instead of
// discarding the rest of the input buffer, so it recovers from noise
// mid-stream.
class Esp32FrameDecoder
{
public:
  enum class Result
  {
    INCOMPLETE,
    FRAME_READY,
    CRC_ERROR,
    LENGTH_ERROR,
  };

  Result feed(uint8_t byte);
  const esp32_protocol::Frame & frame() const {return frame_;}
  void reset();

private:
  enum class State
  {
    WAIT_SOF1,
    WAIT_SOF2,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CRC,
  };

  State state_ = State::WAIT_SOF1;
  esp32_protocol::Frame frame_{};
  uint8_t payload_index_ = 0;
};

}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__ESP32_FRAME_CODEC_HPP_

#pragma once

#include <cstdint>

#include "autonav_protocol/protocol.h"

namespace autonav_protocol {

// Byte-at-a-time streaming decoder. On CRC or length error the frame is
// dropped and the state machine resyncs from WaitSof1 instead of discarding
// the rest of the input buffer, so it recovers from noise mid-stream.
class FrameDecoder {
 public:
  enum class Result {
    kIncomplete,
    kFrameReady,
    kCrcError,
    kLengthError,
  };

  Result Feed(uint8_t byte);
  const Frame& frame() const { return frame_; }
  void Reset();

 private:
  enum class State {
    kWaitSof1,
    kWaitSof2,
    kReadCmd,
    kReadLen,
    kReadPayload,
    kReadCrc,
  };

  State state_ = State::kWaitSof1;
  Frame frame_{};
  uint8_t payload_index_ = 0;
};

}  // namespace autonav_protocol

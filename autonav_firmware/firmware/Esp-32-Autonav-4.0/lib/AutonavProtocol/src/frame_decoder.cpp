#include "autonav_protocol/frame_decoder.h"

namespace autonav_protocol {

void FrameDecoder::Reset() {
  state_ = State::kWaitSof1;
  payload_index_ = 0;
}

FrameDecoder::Result FrameDecoder::Feed(uint8_t byte) {
  switch (state_) {
    case State::kWaitSof1:
      if (byte == kStartOfFrame1) {
        state_ = State::kWaitSof2;
      }
      return Result::kIncomplete;

    case State::kWaitSof2:
      state_ = (byte == kStartOfFrame2) ? State::kReadCmd : State::kWaitSof1;
      return Result::kIncomplete;

    case State::kReadCmd:
      frame_.cmd = static_cast<CommandId>(byte);
      state_ = State::kReadLen;
      return Result::kIncomplete;

    case State::kReadLen:
      frame_.len = byte;
      payload_index_ = 0;
      if (frame_.len > kMaxPayloadLen) {
        Reset();
        return Result::kLengthError;
      }
      state_ = (frame_.len == 0) ? State::kReadCrc : State::kReadPayload;
      return Result::kIncomplete;

    case State::kReadPayload:
      frame_.payload[payload_index_++] = byte;
      if (payload_index_ >= frame_.len) {
        state_ = State::kReadCrc;
      }
      return Result::kIncomplete;

    case State::kReadCrc: {
      uint8_t header[2 + kMaxPayloadLen];
      header[0] = static_cast<uint8_t>(frame_.cmd);
      header[1] = frame_.len;
      for (uint8_t i = 0; i < frame_.len; ++i) {
        header[2 + i] = frame_.payload[i];
      }
      const uint8_t expected_crc = Crc8(header, static_cast<size_t>(frame_.len) + 2);
      Reset();
      return (byte == expected_crc) ? Result::kFrameReady : Result::kCrcError;
    }
  }

  return Result::kIncomplete;
}

}  // namespace autonav_protocol

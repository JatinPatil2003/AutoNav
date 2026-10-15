#include "autonav_firmware/esp32_frame_codec.hpp"

namespace autonav_firmware
{

namespace esp32_protocol
{

uint8_t crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace esp32_protocol

size_t Esp32FrameEncoder::encode(
  esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len,
  uint8_t * out_buf, size_t out_buf_capacity)
{
  if (len > esp32_protocol::MAX_PAYLOAD_LEN) {
    return 0;
  }
  const size_t frame_size = static_cast<size_t>(len) + 5;
  if (out_buf_capacity < frame_size) {
    return 0;
  }

  out_buf[0] = esp32_protocol::START_OF_FRAME_1;
  out_buf[1] = esp32_protocol::START_OF_FRAME_2;
  out_buf[2] = static_cast<uint8_t>(cmd);
  out_buf[3] = len;
  for (uint8_t i = 0; i < len; ++i) {
    out_buf[4 + i] = payload[i];
  }
  out_buf[4 + len] = esp32_protocol::crc8(&out_buf[2], static_cast<size_t>(len) + 2);

  return frame_size;
}

void Esp32FrameDecoder::reset()
{
  state_ = State::WAIT_SOF1;
  payload_index_ = 0;
}

Esp32FrameDecoder::Result Esp32FrameDecoder::feed(uint8_t byte)
{
  switch (state_) {
    case State::WAIT_SOF1:
      if (byte == esp32_protocol::START_OF_FRAME_1) {
        state_ = State::WAIT_SOF2;
      }
      return Result::INCOMPLETE;

    case State::WAIT_SOF2:
      state_ = (byte == esp32_protocol::START_OF_FRAME_2) ? State::READ_CMD : State::WAIT_SOF1;
      return Result::INCOMPLETE;

    case State::READ_CMD:
      frame_.cmd = static_cast<esp32_protocol::CommandId>(byte);
      state_ = State::READ_LEN;
      return Result::INCOMPLETE;

    case State::READ_LEN:
      frame_.len = byte;
      payload_index_ = 0;
      if (frame_.len > esp32_protocol::MAX_PAYLOAD_LEN) {
        reset();
        return Result::LENGTH_ERROR;
      }
      state_ = (frame_.len == 0) ? State::READ_CRC : State::READ_PAYLOAD;
      return Result::INCOMPLETE;

    case State::READ_PAYLOAD:
      frame_.payload[payload_index_++] = byte;
      if (payload_index_ >= frame_.len) {
        state_ = State::READ_CRC;
      }
      return Result::INCOMPLETE;

    case State::READ_CRC: {
      uint8_t header[2 + esp32_protocol::MAX_PAYLOAD_LEN];
      header[0] = static_cast<uint8_t>(frame_.cmd);
      header[1] = frame_.len;
      for (uint8_t i = 0; i < frame_.len; ++i) {
        header[2 + i] = frame_.payload[i];
      }
      const uint8_t expected_crc = esp32_protocol::crc8(header, static_cast<size_t>(frame_.len) + 2);
      reset();
      return (byte == expected_crc) ? Result::FRAME_READY : Result::CRC_ERROR;
    }
  }

  return Result::INCOMPLETE;
}

}  // namespace autonav_firmware

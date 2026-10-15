#include <Arduino.h>
#include <FastLED.h>

#include <cstring>

#include "autonav_estop/estop_monitor.h"
#include "autonav_identity/device_identity.h"
#include "autonav_led/led_controller.h"
#include "autonav_protocol/frame_decoder.h"
#include "autonav_protocol/frame_encoder.h"
#include "board_config.h"

using autonav_estop::EstopMonitor;
using autonav_identity::DeviceIdentity;
using autonav_led::LedController;
using autonav_led::LedPatternId;
using autonav_led::LedPatternParams;
using autonav_protocol::AckStatus;
using autonav_protocol::CommandId;
using autonav_protocol::Frame;
using autonav_protocol::FrameDecoder;
using autonav_protocol::FrameEncoder;
using autonav_protocol::kMaxPayloadLen;

namespace {

DeviceIdentity identity;
EstopMonitor estop(board::ESTOP_PIN, board::ESTOP_ACTIVE_LOW, board::ESTOP_DEBOUNCE_MS);
LedController led;
FrameDecoder decoder;

uint32_t last_heartbeat_ms = 0;

void SendFrame(CommandId cmd, const uint8_t* payload, uint8_t len) {
  uint8_t buf[kMaxPayloadLen + 5];
  const size_t frame_size = FrameEncoder::Encode(cmd, payload, len, buf, sizeof(buf));
  if (frame_size > 0) {
    Serial.write(buf, frame_size);
  }
}

void SendAck(CommandId original_cmd, AckStatus status) {
  const uint8_t payload[2] = {static_cast<uint8_t>(original_cmd), static_cast<uint8_t>(status)};
  SendFrame(CommandId::kAck, payload, sizeof(payload));
}

void SendHeartbeat(uint32_t now_ms) {
  uint8_t payload[autonav_protocol::kDeviceIdLen + 1 + 4] = {};
  memcpy(payload, identity.id(), autonav_protocol::kDeviceIdLen);
  payload[autonav_protocol::kDeviceIdLen] = estop.IsTriggered() ? 1 : 0;
  memcpy(&payload[autonav_protocol::kDeviceIdLen + 1], &now_ms, sizeof(now_ms));
  SendFrame(CommandId::kHeartbeat, payload, sizeof(payload));
}

void HandleFrame(const Frame& frame) {
  switch (frame.cmd) {
    case CommandId::kGetDeviceId: {
      SendFrame(CommandId::kDeviceIdResponse,
                reinterpret_cast<const uint8_t*>(identity.id()),
                autonav_protocol::kDeviceIdLen);
      break;
    }

    case CommandId::kSetLedPattern: {
      // 1-byte payload (pattern_id only): use the pattern's hardcoded
      // default params. 6-byte payload: pattern_id + explicit hue/sat/
      // val/speed_ms override. Anything else is malformed.
      if (frame.len == 1) {
        led.SetPattern(static_cast<LedPatternId>(frame.payload[0]));
      } else if (frame.len >= 6) {
        LedPatternParams params;
        params.hue = frame.payload[1];
        params.sat = frame.payload[2];
        params.val = frame.payload[3];
        params.speed_ms = static_cast<uint16_t>(frame.payload[4] | (frame.payload[5] << 8));
        led.SetPattern(static_cast<LedPatternId>(frame.payload[0]), params);
      } else {
        SendAck(frame.cmd, AckStatus::kErrorLength);
        break;
      }
      SendAck(frame.cmd, AckStatus::kOk);
      break;
    }

    case CommandId::kSetBrightness: {
      if (frame.len < 1) {
        SendAck(frame.cmd, AckStatus::kErrorLength);
        break;
      }
      led.SetBrightness(frame.payload[0]);
      SendAck(frame.cmd, AckStatus::kOk);
      break;
    }

    case CommandId::kEmergencyCommand: {
      if (frame.len < 1) {
        SendAck(frame.cmd, AckStatus::kErrorLength);
        break;
      }
      led.SetEmergency(frame.payload[0] != 0);
      SendAck(frame.cmd, AckStatus::kOk);
      break;
    }

    default:
      SendAck(frame.cmd, AckStatus::kErrorUnknownCommand);
      break;
  }
}

void PollSerial() {
  while (Serial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    const FrameDecoder::Result result = decoder.Feed(byte);
    if (result == FrameDecoder::Result::kFrameReady) {
      HandleFrame(decoder.frame());
    }
    // CRC/length errors are silently dropped; the decoder has already
    // resynced, and a missed heartbeat/ack on the ROS side is how those
    // get noticed there.
  }
}

}  // namespace

void setup() {
  Serial.begin(board::UART_BAUD);
  identity.Begin();
  estop.Begin();
  led.Begin();
}

void loop() {
  const uint32_t now_ms = millis();

  PollSerial();

  estop.Update(now_ms);
  if (estop.ConsumeChanged()) {
    const uint8_t state = estop.IsTriggered() ? 1 : 0;
    SendFrame(CommandId::kEstopPhysicalPush, &state, 1);
  }

  led.Update(now_ms);

  if (now_ms - last_heartbeat_ms >= board::HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_ms = now_ms;
    SendHeartbeat(now_ms);
  }
}

#pragma once

#include <cstddef>
#include <cstdint>

// Wire protocol for the UART link between this ESP32 and the ROS2-side
// Esp32BridgeNode (autonav_firmware). This file must stay byte-for-byte
// identical in meaning to the mirror copy at
// autonav_firmware/include/autonav_firmware/esp32_protocol.hpp.
namespace autonav_protocol {

enum class CommandId : uint8_t {
  kHeartbeat = 0x01,            // ESP32 -> ROS, periodic
  kGetDeviceId = 0x02,          // ROS -> ESP32
  kDeviceIdResponse = 0x03,     // ESP32 -> ROS
  kSetLedPattern = 0x04,        // ROS -> ESP32
  kSetBrightness = 0x05,        // ROS -> ESP32
  kEstopPhysicalPush = 0x06,    // ESP32 -> ROS, on change
  kEmergencyCommand = 0x07,     // ROS -> ESP32
  kAck = 0x08,                  // either direction
  kGetBatteryStatus = 0x09,     // ROS -> ESP32
  kBatteryStatusResponse = 0x0A, // ESP32 -> ROS
};

// kBatteryStatusResponse payload — packed manually into Frame::payload
// byte-by-byte, same convention as kSetLedPattern's multi-field payload.
// Little-endian, 7 bytes: float32 voltage_volts, uint8 percentage (0-100),
// uint8 charging (0/1), uint8 is_default (1 until a real voltage-sense ADC
// is wired up on this board — see GetBatteryStatus() in main.cpp). Must
// stay identical to the ROS-side mirror's comment in esp32_protocol.hpp.
constexpr size_t kBatteryStatusPayloadLen = 7;

enum class AckStatus : uint8_t {
  kOk = 0,
  kErrorCrc = 1,
  kErrorLength = 2,
  kErrorUnknownCommand = 3,
  kErrorBusy = 4,
};

constexpr uint8_t kStartOfFrame1 = 0xAA;
constexpr uint8_t kStartOfFrame2 = 0x55;
constexpr size_t kMaxPayloadLen = 32;
constexpr size_t kDeviceIdLen = 16;  // fixed-width, NUL-padded ASCII

struct Frame {
  CommandId cmd = CommandId::kAck;
  uint8_t len = 0;
  uint8_t payload[kMaxPayloadLen] = {};
};

// CRC-8/SMBUS: poly 0x07, init 0x00, no reflect, no xorout.
// Computed over CMD || LEN || PAYLOAD (never the SOF bytes).
uint8_t Crc8(const uint8_t* data, size_t len);

}  // namespace autonav_protocol

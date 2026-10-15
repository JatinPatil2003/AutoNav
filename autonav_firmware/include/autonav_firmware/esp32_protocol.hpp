#ifndef AUTONAV_FIRMWARE__ESP32_PROTOCOL_HPP_
#define AUTONAV_FIRMWARE__ESP32_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

// Wire protocol for the UART link between the ESP32 (Esp-32-Autonav-4.0
// firmware) and this node. Must stay byte-for-byte identical in meaning to
// the firmware's copy at
// autonav_firmware/firmware/Esp-32-Autonav-4.0/lib/AutonavProtocol/include/autonav_protocol/protocol.h
namespace autonav_firmware
{
namespace esp32_protocol
{

enum class CommandId : uint8_t
{
  HEARTBEAT = 0x01,           // ESP32 -> ROS, periodic
  GET_DEVICE_ID = 0x02,       // ROS -> ESP32
  DEVICE_ID_RESPONSE = 0x03,  // ESP32 -> ROS
  SET_LED_PATTERN = 0x04,     // ROS -> ESP32
  SET_BRIGHTNESS = 0x05,      // ROS -> ESP32
  ESTOP_PHYSICAL_PUSH = 0x06, // ESP32 -> ROS, on change
  EMERGENCY_COMMAND = 0x07,   // ROS -> ESP32
  ACK = 0x08,                 // either direction
};

enum class AckStatus : uint8_t
{
  OK = 0,
  ERROR_CRC = 1,
  ERROR_LENGTH = 2,
  ERROR_UNKNOWN_COMMAND = 3,
  ERROR_BUSY = 4,
};

constexpr uint8_t START_OF_FRAME_1 = 0xAA;
constexpr uint8_t START_OF_FRAME_2 = 0x55;
constexpr size_t MAX_PAYLOAD_LEN = 32;
constexpr size_t DEVICE_ID_LEN = 16;  // fixed-width, NUL-padded ASCII

struct Frame
{
  CommandId cmd = CommandId::ACK;
  uint8_t len = 0;
  uint8_t payload[MAX_PAYLOAD_LEN] = {};
};

// CRC-8/SMBUS: poly 0x07, init 0x00, no reflect, no xorout.
// Computed over CMD || LEN || PAYLOAD (never the SOF bytes).
uint8_t crc8(const uint8_t * data, size_t len);

}  // namespace esp32_protocol
}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__ESP32_PROTOCOL_HPP_

#pragma once

#include <Arduino.h>

namespace board {

// LED strip
constexpr uint8_t LED_PIN = 17;
constexpr uint16_t NUM_LEDS = 45;
constexpr uint8_t LED_BRIGHTNESS_DEFAULT = 128;
constexpr uint32_t LED_TICK_INTERVAL_MS = 20;

// Emergency stop input
constexpr uint8_t ESTOP_PIN = 27;
constexpr bool ESTOP_ACTIVE_LOW = true;
constexpr uint32_t ESTOP_DEBOUNCE_MS = 30;

// UART link to the ROS-side bridge node
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 200;

// Device identity (persisted in NVS; this is only the first-boot default)
constexpr const char* DEVICE_ID_DEFAULT = "AUTONAV-001";

}  // namespace board

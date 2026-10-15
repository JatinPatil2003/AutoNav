#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// All LEDs off.
class OffPattern : public LedPattern {
 public:
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;
};

}  // namespace autonav_led

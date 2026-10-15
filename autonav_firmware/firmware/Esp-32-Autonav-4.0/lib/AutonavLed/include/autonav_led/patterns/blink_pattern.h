#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// Alternates the whole strip between the configured color and black.
// Generalizes the old Green_Blink().
class BlinkPattern : public LedPattern {
 public:
  void OnEnter(CRGB* leds, uint16_t num_leds) override;
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;

 private:
  bool lit_ = false;
  uint32_t last_toggle_ms_ = 0;
};

}  // namespace autonav_led

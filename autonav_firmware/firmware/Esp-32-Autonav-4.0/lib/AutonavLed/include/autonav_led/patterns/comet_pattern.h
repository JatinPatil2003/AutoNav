#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// A single lit pixel travels around the strip, leaving a fading trail
// ("comet"). Generalizes the old Blue_Rotate()/Cyan_Rotate() into one
// parameterized pattern (hue comes from params_).
class CometPattern : public LedPattern {
 public:
  void OnEnter(CRGB* leds, uint16_t num_leds) override;
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;

 private:
  uint16_t position_ = 0;
  uint32_t last_step_ms_ = 0;
};

}  // namespace autonav_led

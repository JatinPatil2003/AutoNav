#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// Brightness bounces between a low and high bound at the configured hue/sat
// ("breathing"). Generalizes the old Red/Green/Blue/Violet/White/Yellow_InOut()
// functions into a single parameterized pattern.
class BreathePattern : public LedPattern {
 public:
  void OnEnter(CRGB* leds, uint16_t num_leds) override;
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;

 private:
  uint8_t brightness_ = 255;
  int8_t fade_dir_ = -1;
  uint32_t last_step_ms_ = 0;
};

}  // namespace autonav_led

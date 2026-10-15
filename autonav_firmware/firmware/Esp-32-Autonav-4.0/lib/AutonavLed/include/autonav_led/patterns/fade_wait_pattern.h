#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// Fades the configured color out to black, then pauses for params_.speed_ms
// before fading back in. Mirrors the old Green_OutWait() behavior, but
// generalized to any hue/sat via LedPatternParams.
class FadeWaitPattern : public LedPattern {
 public:
  void OnEnter(CRGB* leds, uint16_t num_leds) override;
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;

 private:
  uint8_t brightness_ = 255;
  bool fading_ = true;
  uint32_t resume_at_ms_ = 0;
};

}  // namespace autonav_led

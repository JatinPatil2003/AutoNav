#pragma once

#include "autonav_led/led_pattern.h"

namespace autonav_led {

// Fast red strobe used exclusively while an emergency is active. Not
// reachable via SET_LED_PATTERN; only LedController::SetEmergency(true)
// activates it, overriding whatever pattern was previously running.
class EmergencyAlertPattern : public LedPattern {
 public:
  void OnEnter(CRGB* leds, uint16_t num_leds) override;
  void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) override;

 private:
  bool lit_ = false;
  uint32_t last_toggle_ms_ = 0;
};

}  // namespace autonav_led

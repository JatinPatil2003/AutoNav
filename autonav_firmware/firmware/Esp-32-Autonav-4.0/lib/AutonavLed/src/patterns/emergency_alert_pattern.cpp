#include "autonav_led/patterns/emergency_alert_pattern.h"

namespace autonav_led {

namespace {
constexpr uint32_t kStrobeIntervalMs = 150;
}  // namespace

void EmergencyAlertPattern::OnEnter(CRGB* /*leds*/, uint16_t /*num_leds*/) {
  lit_ = true;
  last_toggle_ms_ = 0;
}

void EmergencyAlertPattern::Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) {
  if (now_ms - last_toggle_ms_ >= kStrobeIntervalMs) {
    last_toggle_ms_ = now_ms;
    lit_ = !lit_;
  }

  fill_solid(leds, num_leds, lit_ ? CRGB::Red : CRGB::Black);
}

}  // namespace autonav_led

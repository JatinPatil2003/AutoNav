#include "autonav_led/patterns/blink_pattern.h"

namespace autonav_led {

void BlinkPattern::OnEnter(CRGB* /*leds*/, uint16_t /*num_leds*/) {
  lit_ = false;
  last_toggle_ms_ = 0;
}

void BlinkPattern::Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) {
  if (now_ms - last_toggle_ms_ >= params_.speed_ms) {
    last_toggle_ms_ = now_ms;
    lit_ = !lit_;
  }

  if (lit_) {
    fill_solid(leds, num_leds, CHSV(params_.hue, params_.sat, params_.val));
  } else {
    fill_solid(leds, num_leds, CRGB::Black);
  }
}

}  // namespace autonav_led

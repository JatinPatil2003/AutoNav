#include "autonav_led/patterns/comet_pattern.h"

namespace autonav_led {

namespace {
constexpr uint8_t kTrailFadeAmount = 20;
}  // namespace

void CometPattern::OnEnter(CRGB* leds, uint16_t num_leds) {
  position_ = 0;
  last_step_ms_ = 0;
  fill_solid(leds, num_leds, CRGB::Black);
}

void CometPattern::Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) {
  leds[position_] = CHSV(params_.hue, params_.sat, params_.val);
  fadeToBlackBy(leds, num_leds, kTrailFadeAmount);

  if (now_ms - last_step_ms_ >= params_.speed_ms) {
    last_step_ms_ = now_ms;
    position_ = static_cast<uint16_t>((position_ + 1) % num_leds);
  }
}

}  // namespace autonav_led

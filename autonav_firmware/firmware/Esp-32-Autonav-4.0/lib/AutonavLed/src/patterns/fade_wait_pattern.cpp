#include "autonav_led/patterns/fade_wait_pattern.h"

namespace autonav_led {

namespace {
constexpr uint8_t kFadeStep = 5;
constexpr uint8_t kMinBrightness = 5;
}  // namespace

void FadeWaitPattern::OnEnter(CRGB* /*leds*/, uint16_t /*num_leds*/) {
  brightness_ = 255;
  fading_ = true;
  resume_at_ms_ = 0;
}

void FadeWaitPattern::Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) {
  if (!fading_) {
    if (now_ms >= resume_at_ms_) {
      brightness_ = 255;
      fading_ = true;
    }
    return;
  }

  brightness_ = (brightness_ > kFadeStep) ? (brightness_ - kFadeStep) : 0;
  fill_solid(leds, num_leds, CHSV(params_.hue, params_.sat, brightness_));

  if (brightness_ <= kMinBrightness) {
    fading_ = false;
    resume_at_ms_ = now_ms + params_.speed_ms;
  }
}

}  // namespace autonav_led

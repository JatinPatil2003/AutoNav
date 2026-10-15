#include "autonav_led/patterns/breathe_pattern.h"

namespace autonav_led {

namespace {
constexpr uint8_t kBrightnessStep = 4;
constexpr uint8_t kMaxBrightness = 245;
constexpr uint8_t kMinBrightness = 10;
}  // namespace

void BreathePattern::OnEnter(CRGB* /*leds*/, uint16_t /*num_leds*/) {
  brightness_ = 255;
  fade_dir_ = -1;
  last_step_ms_ = 0;
}

void BreathePattern::Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) {
  if (now_ms - last_step_ms_ >= params_.speed_ms) {
    last_step_ms_ = now_ms;

    brightness_ = static_cast<uint8_t>(brightness_ + fade_dir_ * kBrightnessStep);
    if (brightness_ >= kMaxBrightness) {
      fade_dir_ = -1;
    } else if (brightness_ <= kMinBrightness) {
      fade_dir_ = 1;
    }
  }

  fill_solid(leds, num_leds, CHSV(params_.hue, params_.sat, brightness_));
}

}  // namespace autonav_led

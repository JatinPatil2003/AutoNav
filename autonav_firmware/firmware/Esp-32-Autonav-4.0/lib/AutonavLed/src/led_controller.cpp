#include "autonav_led/led_controller.h"

namespace autonav_led {

void LedController::Begin() {
  FastLED.addLeds<WS2812B, board::LED_PIN, GRB>(leds_, board::NUM_LEDS);
  FastLED.setBrightness(board::LED_BRIGHTNESS_DEFAULT);
  active_pattern_->OnEnter(leds_, board::NUM_LEDS);
}

LedPattern* LedController::PatternForId(LedPatternId id) {
  switch (id) {
    case LedPatternId::kOff:
      return &off_;
    case LedPatternId::kFadeWait:
      return &fade_wait_;
    case LedPatternId::kBreathe:
      return &breathe_;
    case LedPatternId::kComet:
      return &comet_;
    case LedPatternId::kBlink:
      return &blink_;
    default:
      return &off_;
  }
}

void LedController::SetPattern(LedPatternId id, const LedPatternParams& params) {
  LedPattern* next = PatternForId(id);
  next->SetParams(params);

  if (emergency_active_) {
    // Remember the newly-requested pattern so it takes effect once the
    // emergency clears, but don't let it preempt the alert on screen now.
    pre_emergency_pattern_ = next;
    return;
  }

  if (next != active_pattern_) {
    active_pattern_ = next;
    active_pattern_->OnEnter(leds_, board::NUM_LEDS);
  }
}

void LedController::SetPattern(LedPatternId id) {
  SetPattern(id, DefaultParamsForPattern(id));
}

void LedController::SetBrightness(uint8_t brightness) {
  FastLED.setBrightness(brightness);
}

void LedController::SetEmergency(bool active) {
  if (active == emergency_active_) {
    return;
  }
  emergency_active_ = active;

  if (active) {
    pre_emergency_pattern_ = active_pattern_;
    active_pattern_ = &emergency_;
  } else {
    active_pattern_ = pre_emergency_pattern_ ? pre_emergency_pattern_ : &off_;
    pre_emergency_pattern_ = nullptr;
  }
  active_pattern_->OnEnter(leds_, board::NUM_LEDS);
}

void LedController::Update(uint32_t now_ms) {
  if (now_ms - last_tick_ms_ < board::LED_TICK_INTERVAL_MS) {
    return;
  }
  last_tick_ms_ = now_ms;

  active_pattern_->Update(leds_, board::NUM_LEDS, now_ms);
  FastLED.show();
}

}  // namespace autonav_led

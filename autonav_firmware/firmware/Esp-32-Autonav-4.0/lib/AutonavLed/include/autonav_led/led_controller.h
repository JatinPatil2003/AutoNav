#pragma once

#include <FastLED.h>

#include "autonav_led/led_pattern.h"
#include "autonav_led/patterns/blink_pattern.h"
#include "autonav_led/patterns/breathe_pattern.h"
#include "autonav_led/patterns/comet_pattern.h"
#include "autonav_led/patterns/emergency_alert_pattern.h"
#include "autonav_led/patterns/fade_wait_pattern.h"
#include "autonav_led/patterns/off_pattern.h"
#include "board_config.h"

namespace autonav_led {

// Owns the physical LED strip, the fixed set of pattern instances, and
// which one is currently active. Ticks are throttled internally to
// board::LED_TICK_INTERVAL_MS so callers can simply invoke Update() from
// loop() on every iteration.
class LedController {
 public:
  void Begin();

  void SetPattern(LedPatternId id, const LedPatternParams& params);

  // Applies the pattern's hardcoded default params (see
  // DefaultParamsForPattern in led_pattern.h) instead of caller-supplied
  // ones -- used when a SetLedPattern request omits hue/sat/val/speed_ms.
  void SetPattern(LedPatternId id);

  void SetBrightness(uint8_t brightness);

  // true: switches to the emergency strobe, remembering whatever pattern
  // was active so it can be restored. false: restores that pattern.
  // Calling with the same value twice is a no-op.
  void SetEmergency(bool active);

  void Update(uint32_t now_ms);

 private:
  LedPattern* PatternForId(LedPatternId id);

  CRGB leds_[board::NUM_LEDS];

  OffPattern off_;
  FadeWaitPattern fade_wait_;
  BreathePattern breathe_;
  CometPattern comet_;
  BlinkPattern blink_;
  EmergencyAlertPattern emergency_;

  LedPattern* active_pattern_ = &off_;
  LedPattern* pre_emergency_pattern_ = nullptr;
  bool emergency_active_ = false;

  uint32_t last_tick_ms_ = 0;
};

}  // namespace autonav_led

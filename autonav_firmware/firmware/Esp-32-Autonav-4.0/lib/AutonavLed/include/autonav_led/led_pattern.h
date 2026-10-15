#pragma once

#include <FastLED.h>

#include <cstdint>

namespace autonav_led {

struct LedPatternParams {
  uint8_t hue = 0;
  uint8_t sat = 255;
  uint8_t val = 255;
  uint16_t speed_ms = 50;
};

enum class LedPatternId : uint8_t {
  kOff = 0,
  kFadeWait = 1,
  kBreathe = 2,
  kComet = 3,
  kBlink = 4,
  // 5 is reserved for future patterns.
  // kEmergencyAlert is intentionally not selectable here; it is only
  // ever activated via LedController::SetEmergency().
};

// Hardcoded default color/speed per pattern, used whenever a caller sends
// only a pattern id (no explicit hue/sat/val/speed_ms) -- see
// autonav_protocol::CommandId::kSetLedPattern. Each default is chosen to
// give the pattern a distinct, recognizable "meaning" at a glance so a
// caller never has to know FastLED hue values just to get a sane look:
//   kOff      -- n/a, strip is black regardless of params.
//   kFadeWait -- amber breathing fade with a long pause: "standby/waiting".
//   kBreathe  -- slow cyan breathe: "idle/ready".
//   kComet    -- blue comet chasing around the strip: "active/moving".
//   kBlink    -- fast red blink: "alert/attention".
inline LedPatternParams DefaultParamsForPattern(LedPatternId id) {
  switch (id) {
    case LedPatternId::kFadeWait:
      return LedPatternParams{.hue = 32, .sat = 255, .val = 255, .speed_ms = 1200};
    case LedPatternId::kBreathe:
      return LedPatternParams{.hue = 128, .sat = 255, .val = 245, .speed_ms = 30};
    case LedPatternId::kComet:
      return LedPatternParams{.hue = 160, .sat = 255, .val = 255, .speed_ms = 30};
    case LedPatternId::kBlink:
      return LedPatternParams{.hue = 0, .sat = 255, .val = 255, .speed_ms = 200};
    case LedPatternId::kOff:
    default:
      return LedPatternParams{};
  }
}

// Base class for a single non-blocking LED animation. A pattern owns all
// of its own animation state (no file-scope globals) and is driven by
// repeated, non-blocking calls to Update() from LedController's tick.
class LedPattern {
 public:
  virtual ~LedPattern() = default;

  // Called once when the pattern becomes active; resets internal state.
  virtual void OnEnter(CRGB* leds, uint16_t num_leds) { (void)leds; (void)num_leds; }

  // Called every LedController tick; must return promptly (non-blocking).
  virtual void Update(CRGB* leds, uint16_t num_leds, uint32_t now_ms) = 0;

  virtual void SetParams(const LedPatternParams& params) { params_ = params; }

 protected:
  LedPatternParams params_;
};

}  // namespace autonav_led

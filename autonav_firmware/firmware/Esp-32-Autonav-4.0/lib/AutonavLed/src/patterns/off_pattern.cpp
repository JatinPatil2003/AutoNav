#include "autonav_led/patterns/off_pattern.h"

namespace autonav_led {

void OffPattern::Update(CRGB* leds, uint16_t num_leds, uint32_t /*now_ms*/) {
  fill_solid(leds, num_leds, CRGB::Black);
}

}  // namespace autonav_led

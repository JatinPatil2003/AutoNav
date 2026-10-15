#include "autonav_estop/estop_monitor.h"

#include <Arduino.h>

namespace autonav_estop {

EstopMonitor::EstopMonitor(uint8_t pin, bool active_low, uint32_t debounce_ms)
    : pin_(pin), active_low_(active_low), debounce_ms_(debounce_ms) {}

void EstopMonitor::Begin() {
  pinMode(pin_, active_low_ ? INPUT_PULLUP : INPUT);
  const bool raw = digitalRead(pin_) == LOW;
  stable_state_ = active_low_ ? raw : !raw;
  last_raw_state_ = stable_state_;
}

void EstopMonitor::Update(uint32_t now_ms) {
  const bool raw_low = digitalRead(pin_) == LOW;
  const bool raw_state = active_low_ ? raw_low : !raw_low;

  if (raw_state != last_raw_state_) {
    last_raw_state_ = raw_state;
    last_change_ms_ = now_ms;
  }

  if (raw_state != stable_state_ && (now_ms - last_change_ms_) >= debounce_ms_) {
    stable_state_ = raw_state;
    changed_flag_ = true;
  }
}

bool EstopMonitor::ConsumeChanged() {
  const bool changed = changed_flag_;
  changed_flag_ = false;
  return changed;
}

}  // namespace autonav_estop

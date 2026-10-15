#pragma once

#include <cstdint>

namespace autonav_estop {

// Debounced digital input for a physical emergency-stop button/switch.
class EstopMonitor {
 public:
  EstopMonitor(uint8_t pin, bool active_low, uint32_t debounce_ms);

  void Begin();

  // Call every loop() iteration; debounces the raw GPIO reads against
  // debounce_ms_ before updating the stable state.
  void Update(uint32_t now_ms);

  // Debounced, stable state (true = e-stop engaged).
  bool IsTriggered() const { return stable_state_; }

  // Returns true exactly once per stable-state transition since the last
  // call, for edge-triggered reporting (e.g. ESTOP_PHYSICAL_PUSH frames).
  bool ConsumeChanged();

 private:
  uint8_t pin_;
  bool active_low_;
  uint32_t debounce_ms_;

  bool stable_state_ = false;
  bool last_raw_state_ = false;
  uint32_t last_change_ms_ = 0;
  bool changed_flag_ = false;
};

}  // namespace autonav_estop

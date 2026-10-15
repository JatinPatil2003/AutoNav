#pragma once

#include <Preferences.h>

#include <cstddef>

#include "autonav_protocol/protocol.h"

namespace autonav_identity {

// Persists a short device/robot identity string in NVS so the ROS-side
// bridge node can confirm which board it's talking to on connect.
class DeviceIdentity {
 public:
  static constexpr size_t kIdLen = autonav_protocol::kDeviceIdLen;

  // Opens the "autonav" NVS namespace and loads the stored id, writing
  // board::DEVICE_ID_DEFAULT the first time no id has been persisted yet.
  void Begin();

  // NUL-terminated, at most kIdLen characters.
  const char* id() const { return id_; }

  // Persists a new id to NVS. Truncated to kIdLen if longer.
  // Not reachable over the wire protocol; intended for a future
  // provisioning tool only.
  bool SetId(const char* new_id);

 private:
  Preferences prefs_;
  char id_[kIdLen + 1] = {};
};

}  // namespace autonav_identity

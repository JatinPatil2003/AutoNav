#include "autonav_identity/device_identity.h"

#include <cstring>

#include "board_config.h"

namespace autonav_identity {

namespace {
constexpr const char* kNvsNamespace = "autonav";
constexpr const char* kNvsKeyDeviceId = "device_id";
}  // namespace

void DeviceIdentity::Begin() {
  prefs_.begin(kNvsNamespace, /*readOnly=*/false);

  String stored = prefs_.getString(kNvsKeyDeviceId, "");
  if (stored.length() == 0) {
    stored = board::DEVICE_ID_DEFAULT;
    prefs_.putString(kNvsKeyDeviceId, stored);
  }

  memset(id_, 0, sizeof(id_));
  strncpy(id_, stored.c_str(), kIdLen);
}

bool DeviceIdentity::SetId(const char* new_id) {
  if (new_id == nullptr) {
    return false;
  }

  memset(id_, 0, sizeof(id_));
  strncpy(id_, new_id, kIdLen);
  return prefs_.putString(kNvsKeyDeviceId, id_) > 0;
}

}  // namespace autonav_identity

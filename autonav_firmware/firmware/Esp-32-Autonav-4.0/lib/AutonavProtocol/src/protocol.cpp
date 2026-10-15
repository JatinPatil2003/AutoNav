#include "autonav_protocol/protocol.h"

namespace autonav_protocol {

uint8_t Crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace autonav_protocol

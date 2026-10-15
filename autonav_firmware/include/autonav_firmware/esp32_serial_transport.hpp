#ifndef AUTONAV_FIRMWARE__ESP32_SERIAL_TRANSPORT_HPP_
#define AUTONAV_FIRMWARE__ESP32_SERIAL_TRANSPORT_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "autonav_firmware/esp32_frame_codec.hpp"

namespace autonav_firmware
{

// Owns the raw UART file descriptor to the ESP32 and a dedicated reader
// thread. Mirrors MotorController's threading split: a background thread
// does the blocking I/O, callers only ever touch atomics/mutex-guarded
// state from the ROS-facing thread. FrameCallback is invoked directly from
// the reader thread and must never block or call into rclcpp publishers;
// it should only stash state for a timer on the node's own thread to act on.
class Esp32SerialTransport
{
public:
  using FrameCallback =
    std::function<void (esp32_protocol::CommandId, const uint8_t * payload, uint8_t len)>;

  Esp32SerialTransport();
  ~Esp32SerialTransport();

  bool open(const std::string & port, int baudrate);
  void close();
  bool isOpen() const;

  void setFrameCallback(FrameCallback cb);

  // Mutex-guarded write; safe to call from any thread.
  bool sendFrame(esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len);

private:
  void readerLoop();
  static int baudrateToTermiosConstant(int baudrate);

  int fd_;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  std::mutex write_mutex_;
  Esp32FrameDecoder decoder_;
  FrameCallback frame_callback_;
};

}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__ESP32_SERIAL_TRANSPORT_HPP_

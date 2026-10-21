#include "autonav_firmware/esp32_serial_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace autonav_firmware
{

namespace
{

rclcpp::Logger transportLogger()
{
  return rclcpp::get_logger("esp32_serial_transport");
}

const char * commandName(esp32_protocol::CommandId cmd)
{
  switch (cmd) {
    case esp32_protocol::CommandId::HEARTBEAT: return "HEARTBEAT";
    case esp32_protocol::CommandId::GET_DEVICE_ID: return "GET_DEVICE_ID";
    case esp32_protocol::CommandId::DEVICE_ID_RESPONSE: return "DEVICE_ID_RESPONSE";
    case esp32_protocol::CommandId::SET_LED_PATTERN: return "SET_LED_PATTERN";
    case esp32_protocol::CommandId::SET_BRIGHTNESS: return "SET_BRIGHTNESS";
    case esp32_protocol::CommandId::ESTOP_PHYSICAL_PUSH: return "ESTOP_PHYSICAL_PUSH";
    case esp32_protocol::CommandId::EMERGENCY_COMMAND: return "EMERGENCY_COMMAND";
    case esp32_protocol::CommandId::ACK: return "ACK";
    case esp32_protocol::CommandId::GET_BATTERY_STATUS: return "GET_BATTERY_STATUS";
    case esp32_protocol::CommandId::BATTERY_STATUS_RESPONSE: return "BATTERY_STATUS_RESPONSE";
    default: return "UNKNOWN";
  }
}

// Hex-dumps a payload for log lines, e.g. "04 02 ff ff 32 00". Debug-only;
// never used on a hot path that matters for timing.
std::string hexDump(const uint8_t * data, uint8_t len)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (uint8_t i = 0; i < len; ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

}  // namespace

Esp32SerialTransport::Esp32SerialTransport()
: fd_(-1)
{
}

Esp32SerialTransport::~Esp32SerialTransport()
{
  close();
}

int Esp32SerialTransport::baudrateToTermiosConstant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

bool Esp32SerialTransport::open(const std::string & port, int baudrate)
{
  if (isOpen()) {
    return true;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1) {
    std::cerr << "Esp32SerialTransport: failed to open " << port << ": " <<
      strerror(errno) << std::endl;
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "Esp32SerialTransport: tcgetattr failed: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  const int speed = baudrateToTermiosConstant(baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, raw mode, no flow control.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  cfmakeraw(&tty);

  // Blocking reads: return as soon as at least 1 byte is available.
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "Esp32SerialTransport: tcsetattr failed: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  running_ = true;
  reader_thread_ = std::thread(&Esp32SerialTransport::readerLoop, this);
  return true;
}

void Esp32SerialTransport::close()
{
  if (running_) {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Esp32SerialTransport::isOpen() const
{
  return fd_ != -1;
}

void Esp32SerialTransport::setFrameCallback(FrameCallback cb)
{
  frame_callback_ = std::move(cb);
}

bool Esp32SerialTransport::sendFrame(
  esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len)
{
  uint8_t buf[esp32_protocol::MAX_PAYLOAD_LEN + 5];
  const size_t frame_size = Esp32FrameEncoder::encode(cmd, payload, len, buf, sizeof(buf));
  if (frame_size == 0) {
    RCLCPP_WARN(
      transportLogger(), "TX %s: encode failed (len=%u exceeds MAX_PAYLOAD_LEN?)",
      commandName(cmd), len);
    return false;
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ == -1) {
    RCLCPP_WARN(transportLogger(), "TX %s: port not open, dropping frame", commandName(cmd));
    return false;
  }

  const bool ok = ::write(fd_, buf, frame_size) == static_cast<ssize_t>(frame_size);
  if (ok) {
    RCLCPP_DEBUG(
      transportLogger(), "TX %s len=%u payload=[%s] (%zu bytes on wire)",
      commandName(cmd), len, hexDump(payload, len).c_str(), frame_size);
  } else {
    RCLCPP_WARN(
      transportLogger(), "TX %s: write() failed: %s", commandName(cmd), strerror(errno));
  }
  return ok;
}

void Esp32SerialTransport::readerLoop()
{
  uint8_t byte = 0;
  while (running_) {
    const ssize_t n = ::read(fd_, &byte, 1);
    if (n <= 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }

    const Esp32FrameDecoder::Result result = decoder_.feed(byte);
    switch (result) {
      case Esp32FrameDecoder::Result::FRAME_READY: {
        const esp32_protocol::Frame & frame = decoder_.frame();
        RCLCPP_DEBUG(
          transportLogger(), "RX %s len=%u payload=[%s]",
          commandName(frame.cmd), frame.len, hexDump(frame.payload, frame.len).c_str());
        if (frame_callback_) {
          frame_callback_(frame.cmd, frame.payload, frame.len);
        }
        break;
      }
      case Esp32FrameDecoder::Result::CRC_ERROR:
        RCLCPP_WARN(transportLogger(), "RX: dropped frame with bad CRC, resyncing");
        break;
      case Esp32FrameDecoder::Result::LENGTH_ERROR:
        RCLCPP_WARN(transportLogger(), "RX: dropped frame with bad length, resyncing");
        break;
      case Esp32FrameDecoder::Result::INCOMPLETE:
        break;
    }
    // CRC_ERROR / LENGTH_ERROR frames are dropped; the decoder has already
    // resynced. A stalled heartbeat on the node side is how these surface.
  }
}

}  // namespace autonav_firmware

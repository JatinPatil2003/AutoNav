#include "autonav_firmware/esp32_bridge_node.hpp"

namespace autonav_firmware
{

namespace
{
constexpr auto kWatchdogPeriod = std::chrono::milliseconds(50);
constexpr auto kGetDeviceIdWaitTimeout = std::chrono::milliseconds(500);
constexpr auto kAckWaitTimeout = std::chrono::milliseconds(500);

std::string deviceIdFromPayload(const uint8_t * payload)
{
  const char * raw = reinterpret_cast<const char *>(payload);
  size_t len = 0;
  while (len < esp32_protocol::DEVICE_ID_LEN && raw[len] != '\0') {
    ++len;
  }
  return std::string(raw, len);
}

std::string ackStatusToString(esp32_protocol::AckStatus status)
{
  switch (status) {
    case esp32_protocol::AckStatus::OK:
      return "OK";
    case esp32_protocol::AckStatus::ERROR_CRC:
      return "ESP32 rejected the command: CRC error";
    case esp32_protocol::AckStatus::ERROR_LENGTH:
      return "ESP32 rejected the command: bad payload length";
    case esp32_protocol::AckStatus::ERROR_UNKNOWN_COMMAND:
      return "ESP32 rejected the command: unknown command";
    case esp32_protocol::AckStatus::ERROR_BUSY:
      return "ESP32 rejected the command: busy";
    default:
      return "ESP32 rejected the command: unrecognized status";
  }
}
}  // namespace

Esp32BridgeNode::Esp32BridgeNode()
: Node("esp32_bridge_node")
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB2");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  expected_device_id_ = declare_parameter<std::string>("expected_device_id", "");
  heartbeat_timeout_sec_ = declare_parameter<double>("heartbeat_timeout_sec", 1.0);

  connected_pub_ = create_publisher<std_msgs::msg::Bool>("~/connected", 10);
  device_id_pub_ = create_publisher<std_msgs::msg::String>("~/device_id", 10);
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>("/autonav/emergency_status", 10);

  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/autonav/emergency_status", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {onEmergencyStatusMsg(msg);});

  get_device_id_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/get_device_id",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      handleGetDeviceId(request, response);
    });

  set_led_pattern_srv_ = create_service<autonav_msgs::srv::SetLedPattern>(
    "~/led/set_pattern",
    [this](
      const autonav_msgs::srv::SetLedPattern::Request::SharedPtr request,
      autonav_msgs::srv::SetLedPattern::Response::SharedPtr response) {
      handleSetLedPattern(request, response);
    });

  set_brightness_srv_ = create_service<autonav_msgs::srv::SetBrightness>(
    "~/led/set_brightness",
    [this](
      const autonav_msgs::srv::SetBrightness::Request::SharedPtr request,
      autonav_msgs::srv::SetBrightness::Response::SharedPtr response) {
      handleSetBrightness(request, response);
    });

  transport_.setFrameCallback(
    [this](esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len) {
      onFrame(cmd, payload, len);
    });

  if (!transport_.open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to open serial port '%s' -- will remain disconnected",
      serial_port_.c_str());
  }

  watchdog_timer_ = create_wall_timer(kWatchdogPeriod, [this]() {onWatchdogTimer();});
}

Esp32BridgeNode::~Esp32BridgeNode()
{
  transport_.close();
}

void Esp32BridgeNode::onFrame(esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len)
{
  switch (cmd) {
    case esp32_protocol::CommandId::HEARTBEAT: {
      if (len < esp32_protocol::DEVICE_ID_LEN + 1) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      cached_device_id_ = deviceIdFromPayload(payload);
      estop_physical_.store(payload[esp32_protocol::DEVICE_ID_LEN] != 0);
      last_heartbeat_time_ = std::chrono::steady_clock::now();
      device_id_cv_.notify_all();
      break;
    }

    case esp32_protocol::CommandId::DEVICE_ID_RESPONSE: {
      if (len < esp32_protocol::DEVICE_ID_LEN) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      cached_device_id_ = deviceIdFromPayload(payload);
      device_id_cv_.notify_all();
      break;
    }

    case esp32_protocol::CommandId::ESTOP_PHYSICAL_PUSH: {
      if (len < 1) {
        return;
      }
      estop_physical_.store(payload[0] != 0);
      break;
    }

    case esp32_protocol::CommandId::ACK: {
      if (len < 2) {
        return;
      }
      if (payload[1] != static_cast<uint8_t>(esp32_protocol::AckStatus::OK)) {
        RCLCPP_WARN(
          get_logger(), "ESP32 NACKed command 0x%02X with status %u", payload[0], payload[1]);
      }
      std::lock_guard<std::mutex> lock(ack_mutex_);
      last_ack_status_ = static_cast<esp32_protocol::AckStatus>(payload[1]);
      ack_received_ = true;
      ack_cv_.notify_all();
      break;
    }

    default:
      break;
  }
}

void Esp32BridgeNode::onWatchdogTimer()
{
  std::string device_id;
  std::chrono::steady_clock::time_point last_heartbeat;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    device_id = cached_device_id_;
    last_heartbeat = last_heartbeat_time_;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool heartbeat_fresh = last_heartbeat.time_since_epoch().count() > 0 &&
    std::chrono::duration<double>(now - last_heartbeat).count() < heartbeat_timeout_sec_;
  const bool device_id_ok = expected_device_id_.empty() || device_id == expected_device_id_;

  std_msgs::msg::Bool connected_msg;
  connected_msg.data = transport_.isOpen() && heartbeat_fresh && device_id_ok;
  connected_pub_->publish(connected_msg);

  if (!device_id.empty()) {
    std_msgs::msg::String device_id_msg;
    device_id_msg.data = device_id;
    device_id_pub_->publish(device_id_msg);
  }

  std_msgs::msg::Bool emergency_msg;
  emergency_msg.data = estop_physical_.load();
  emergency_pub_->publish(emergency_msg);
}

void Esp32BridgeNode::onEmergencyStatusMsg(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool state = msg->data;
  if (last_emergency_sent_.exchange(state) == state) {
    return;
  }

  const uint8_t payload = state ? 1 : 0;
  transport_.sendFrame(esp32_protocol::CommandId::EMERGENCY_COMMAND, &payload, 1);
}

bool Esp32BridgeNode::sendAndWaitForAck(
  esp32_protocol::CommandId cmd, const uint8_t * payload, uint8_t len,
  esp32_protocol::AckStatus & out_status)
{
  std::unique_lock<std::mutex> lock(ack_mutex_);
  ack_received_ = false;

  if (!transport_.sendFrame(cmd, payload, len)) {
    return false;
  }

  const bool got_ack = ack_cv_.wait_for(lock, kAckWaitTimeout, [this]() {return ack_received_;});
  if (!got_ack) {
    return false;
  }

  out_status = last_ack_status_;
  return true;
}

void Esp32BridgeNode::handleSetLedPattern(
  const autonav_msgs::srv::SetLedPattern::Request::SharedPtr request,
  autonav_msgs::srv::SetLedPattern::Response::SharedPtr response)
{
  esp32_protocol::AckStatus status = esp32_protocol::AckStatus::OK;
  bool sent;
  if (request->use_custom_params) {
    const uint8_t payload[6] = {
      request->pattern_id, request->hue, request->sat, request->val,
      static_cast<uint8_t>(request->speed_ms & 0xFF),
      static_cast<uint8_t>((request->speed_ms >> 8) & 0xFF),
    };
    sent = sendAndWaitForAck(esp32_protocol::CommandId::SET_LED_PATTERN, payload, sizeof(payload), status);
  } else {
    // Pattern id only -- the ESP32 applies that pattern's hardcoded
    // default color/speed (see DefaultParamsForPattern in the firmware's
    // led_pattern.h).
    sent = sendAndWaitForAck(esp32_protocol::CommandId::SET_LED_PATTERN, &request->pattern_id, 1, status);
  }

  if (!sent) {
    response->success = false;
    response->message = "No ACK from ESP32 (not connected?)";
    return;
  }

  response->success = (status == esp32_protocol::AckStatus::OK);
  response->message = ackStatusToString(status);
}

void Esp32BridgeNode::handleSetBrightness(
  const autonav_msgs::srv::SetBrightness::Request::SharedPtr request,
  autonav_msgs::srv::SetBrightness::Response::SharedPtr response)
{
  const uint8_t payload = request->brightness;

  esp32_protocol::AckStatus status = esp32_protocol::AckStatus::OK;
  if (!sendAndWaitForAck(esp32_protocol::CommandId::SET_BRIGHTNESS, &payload, 1, status)) {
    response->success = false;
    response->message = "No ACK from ESP32 (not connected?)";
    return;
  }

  response->success = (status == esp32_protocol::AckStatus::OK);
  response->message = ackStatusToString(status);
}

void Esp32BridgeNode::handleGetDeviceId(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (cached_device_id_.empty()) {
    lock.unlock();
    transport_.sendFrame(esp32_protocol::CommandId::GET_DEVICE_ID, nullptr, 0);
    lock.lock();
    device_id_cv_.wait_for(
      lock, kGetDeviceIdWaitTimeout, [this]() {return !cached_device_id_.empty();});
  }

  if (cached_device_id_.empty()) {
    response->success = false;
    response->message = "No response from ESP32";
    return;
  }

  response->success = true;
  response->message = cached_device_id_;
}

}  // namespace autonav_firmware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonav_firmware::Esp32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}

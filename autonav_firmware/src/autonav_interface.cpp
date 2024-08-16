#include "autonav_firmware/autonav_interface.hpp"
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonav_firmware
{
AutonavInterface::AutonavInterface()
  : node_(std::make_shared<rclcpp::Node>("autonav_interface_node"))
{
  feedback_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/motor/feedback", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      this->processFeedback(msg);
    });

  // left_cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/motor/left_cmd", 10);
  // right_cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/motor/right_cmd", 10);
  cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/motor/command", 10);
}

AutonavInterface::~AutonavInterface()
{
}

hardware_interface::CallbackReturn AutonavInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AutonavInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AutonavInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AutonavInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutonavInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void AutonavInterface::processFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 2) {
    hw_positions_[1] = msg->data[0];  // base2left
    hw_positions_[0] = msg->data[1];  // base2right
    // RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "position successfully readed!");
  }
}

hardware_interface::return_type AutonavInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type autonav_firmware::AutonavInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto cmd_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  cmd_msg->data.push_back(hw_commands_[1]);  // base2left command
  cmd_msg->data.push_back(hw_commands_[0]);  // base2right command
  cmd_publisher_->publish(*cmd_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace autonav_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autonav_firmware::AutonavInterface,
  hardware_interface::SystemInterface)
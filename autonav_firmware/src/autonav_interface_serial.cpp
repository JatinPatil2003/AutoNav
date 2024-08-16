#include "autonav_firmware/autonav_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>

namespace autonav_firmware
{
AutonavInterface::AutonavInterface()
{
}

AutonavInterface::~AutonavInterface()
{
  if (esp_.isOpen()) {
    try {
      esp_.close();
    }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonavInterface"), "Something went wrong while closing connection with port " << port_);
    }
  }
}

hardware_interface::CallbackReturn AutonavInterface::on_init(
    const hardware_interface::HardwareInfo &info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;


  try {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("AutonavInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
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
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  try {
    esp_.setPort(port_);
    esp_.setBaudrate(115200);
    esp_.open();
  }
  catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonavInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutonavInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Deactivating ...please wait...");

  if (esp_.isOpen()) {
    try {
      esp_.close();
    }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonavInterface"), "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AutonavInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string message;
  esp_.readline(message);
  size_t commaIndex = message.find(',');
  if (commaIndex != std::string::npos) {
    std::string leftPos = message.substr(0, commaIndex);
    std::string rightPos = message.substr(commaIndex + 1);

    // Convert the strings to float variables
    hw_positions_[1] = std::stof(leftPos) * 2 * M_PI / 1600.0;
    hw_positions_[0] = std::stof(rightPos) * 2 * M_PI / 1600.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type autonav_firmware ::AutonavInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::stringstream ss;
  auto leftCmd = hw_commands_[1] * 1600.0 / (2 * M_PI);
  auto rightCmd = hw_commands_[0] * 1600.0 / (2 * M_PI);
  ss << leftCmd << "," << rightCmd << "\n";
  try {
    esp_.write(ss.str());
  }
  catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"), "Something went wrong while sending the message " << ss.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace autonav_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autonav_firmware::AutonavInterface,
  hardware_interface::SystemInterface)
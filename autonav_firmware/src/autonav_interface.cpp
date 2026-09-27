// Copyright (c) 2024 Jatin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <memory>
#include <vector>

#include "autonav_firmware/autonav_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonav_firmware
{
AutonavInterface::AutonavInterface()
{
}

AutonavInterface::~AutonavInterface()
{
  if (emergency_executor_) {
    emergency_executor_->cancel();
  }
  if (emergency_spin_thread_.joinable()) {
    emergency_spin_thread_.join();
  }
}

hardware_interface::CallbackReturn AutonavInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  left_port_ = info_.hardware_parameters["left_port"];
  right_port_ = info_.hardware_parameters["right_port"];
  left_slave_id_ = std::stoi(info_.hardware_parameters["left_slave_id"]);
  right_slave_id_ = std::stoi(info_.hardware_parameters["right_slave_id"]);

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

  if (!left_motor_controller_.connect(left_port_, 9600)) {
    RCLCPP_ERROR(rclcpp::get_logger("AutonavInterface"), "Failed to connect to left Modbus motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!right_motor_controller_.connect(right_port_, 9600)) {
    RCLCPP_ERROR(rclcpp::get_logger("AutonavInterface"), "Failed to connect to right Modbus motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_motor_controller_.enableMotor(left_slave_id_);
  right_motor_controller_.enableMotor(right_slave_id_);

  left_motor_controller_.startWorker(left_slave_id_);
  right_motor_controller_.startWorker(right_slave_id_);

  emergency_stop_.store(false);

  // Subscribe to /autonav/emergency_status
  emergency_node_ = std::make_shared<rclcpp::Node>("autonav_emergency_listener");
  emergency_sub_ = emergency_node_->create_subscription<std_msgs::msg::Bool>(
    "/autonav/emergency_status", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      bool is_emergency = msg->data;
      if (is_emergency != emergency_stop_.load()) {
        if (is_emergency) {
          RCLCPP_WARN(
            rclcpp::get_logger("AutonavInterface"),
            "EMERGENCY STATUS TRUE: Engaging brake! Motor write commands halted.");
          left_motor_controller_.setTargetVelocity(0.0);
          right_motor_controller_.setTargetVelocity(0.0);
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("AutonavInterface"),
            "EMERGENCY STATUS FALSE: Emergency cleared. Resuming normal motor write commands.");
        }
        emergency_stop_.store(is_emergency);
      }
    });

  emergency_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  emergency_executor_->add_node(emergency_node_);
  emergency_spin_thread_ = std::thread([this]() {
    emergency_executor_->spin();
  });

  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutonavInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Deactivating ...please wait...");

  if (emergency_executor_) {
    emergency_executor_->cancel();
  }
  if (emergency_spin_thread_.joinable()) {
    emergency_spin_thread_.join();
  }
  emergency_sub_.reset();
  emergency_node_.reset();
  emergency_executor_.reset();

  left_motor_controller_.stopWorker();
  right_motor_controller_.stopWorker();

  left_motor_controller_.disableMotor(left_slave_id_);
  right_motor_controller_.disableMotor(right_slave_id_);

  left_motor_controller_.disconnect();
  right_motor_controller_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("AutonavInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AutonavInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Instantaneous non-blocking read from background workers
  hw_positions_[0] = right_motor_controller_.getFeedbackPosition();
  hw_velocities_[0] = right_motor_controller_.getFeedbackVelocity();

  hw_positions_[1] = left_motor_controller_.getFeedbackPosition();
  hw_velocities_[1] = left_motor_controller_.getFeedbackVelocity();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type autonav_firmware::AutonavInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (emergency_stop_.load()) {
    // When emergency status is true, enforce brake (0.0 rad/s) and refuse to forward hw_commands_
    left_motor_controller_.setTargetVelocity(0.0);
    right_motor_controller_.setTargetVelocity(0.0);
    return hardware_interface::return_type::OK;
  }

  // Instantaneous non-blocking write to background workers
  right_motor_controller_.setTargetVelocity(hw_commands_[0]);
  left_motor_controller_.setTargetVelocity(hw_commands_[1]);

  return hardware_interface::return_type::OK;
}

}  // namespace autonav_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autonav_firmware::AutonavInterface,
  hardware_interface::SystemInterface)

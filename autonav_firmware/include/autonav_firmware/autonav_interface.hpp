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

#ifndef AUTONAV_FIRMWARE__AUTONAV_INTERFACE_HPP_
#define AUTONAV_FIRMWARE__AUTONAV_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "autonav_firmware/motor_controller.hpp"

namespace autonav_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AutonavInterface : public hardware_interface::SystemInterface
{
public:
  AutonavInterface();
  virtual ~AutonavInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;
  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  MotorController motor_controller_;
  
  int left_slave_id_ = 1;
  int right_slave_id_ = 2;
  double hw_start_sec_;
  double hw_stop_sec_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};
}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__AUTONAV_INTERFACE_HPP_

#ifndef AUTONAV_INTERFACE_HPP
#define AUTONAV_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>

#include <vector>
#include <string>


namespace autonav_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AutonavInterface : public hardware_interface::SystemInterface
{
public:
  AutonavInterface();
  virtual ~AutonavInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;
  virtual hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_subscription_;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd_publisher_;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_publisher_;

  void processFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  double hw_start_sec_;
  double hw_stop_sec_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};
}  // namespace autonav_firmware

#endif  // AUTONAV_INTERFACE_HPP

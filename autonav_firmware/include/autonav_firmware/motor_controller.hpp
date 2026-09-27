#ifndef AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_
#define AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_

#include <string>
#include <mutex>
#include <modbus/modbus.h>

namespace autonav_firmware
{

class MotorController
{
public:
  MotorController();
  ~MotorController();

  /**
   * Connect to the Modbus serial port.
   * @param port Typically "/dev/ttyUSB0"
   * @param baudrate Typically 115200
   * @return true if successful
   */
  bool connect(const std::string& port, int baudrate);

  /**
   * Disconnect the serial port
   */
  void disconnect();

  /**
   * Send a velocity command to a specific motor
   * @param slave_id The Modbus slave ID of the motor
   * @param velocity The target velocity (RPM or internal units)
   * @return true if successful
   */
  bool setVelocity(int slave_id, double velocity);

  /**
   * Read current position and velocity feedback
   * @param slave_id The Modbus slave ID of the motor
   * @param position Output reference for position
   * @param velocity Output reference for velocity
   * @return true if successful
   */
  bool readFeedback(int slave_id, double& position, double& velocity);

private:
  modbus_t* ctx_;
  bool connected_;
  std::mutex modbus_mutex_; // To protect concurrent access to modbus context

  // RMCS-3001 Modbus Addresses
  // Please adjust these based on your specific drive manual if different
  static constexpr int REG_SPEED_COMMAND = 6;    // e.g., Frequency / Speed command
  static constexpr int REG_SPEED_FEEDBACK = 8;   // e.g., Speed feedback
  static constexpr int REG_POS_FEEDBACK_LOW = 12; // Placeholder for position (adjust as needed)
  static constexpr int REG_POS_FEEDBACK_HIGH = 13;
};

}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_

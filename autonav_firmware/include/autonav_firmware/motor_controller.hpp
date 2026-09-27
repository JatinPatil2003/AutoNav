#ifndef AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_
#define AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
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
   * @param baudrate Typically 9600
   * @return true if successful
   */
  bool connect(const std::string& port, int baudrate);

  /**
   * Disconnect the serial port
   */
  void disconnect();

  bool enableMotor(int slave_id);
  bool disableMotor(int slave_id);
  bool brakeMotor(int slave_id);

  // Background worker thread management
  bool startWorker(int slave_id);
  void stopWorker();

  // Thread-safe command and feedback interface for ros2_control
  void setTargetVelocity(double velocity);
  double getFeedbackVelocity() const;
  double getFeedbackPosition() const;

  /**
   * Send a velocity command to a specific motor
   * @param slave_id The Modbus slave ID of the motor
   * @param velocity The target velocity (rad/s at wheel)
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

  // RMCS-3001 Modbus Addresses (per Operating Manual v1.0)
  static constexpr int REG_MODBUS_ADDR      = 0;   // 40001: Device Address & EEPROM
  static constexpr int REG_CONTROL_MODE     = 2;   // 40003: Control and Mode Register
  static constexpr int REG_PWM_SPEED        = 4;   // 40005: PWM Register (Mode 2)
  static constexpr int REG_SPEED_COMMAND    = 6;   // 40007: Frequency (Hz) command (Mode 1)
  static constexpr int REG_SPEED_FEEDBACK   = 8;   // 40009: Speed feedback (Hz)
  static constexpr int REG_CURRENT_FEEDBACK = 10;  // 40011: Current feedback (A)
  static constexpr int REG_MOVEMENT_LIMIT   = 12;  // 40013: Movement limit

  // RMCS-3001 Mode 1 (Digital Closed Loop) Command Values for Address 2
  static constexpr uint16_t CMD_DISABLE     = 256; // 0x0100: Mode 01, Control 00 (Disable Motor)
  static constexpr uint16_t CMD_ENABLE_CW   = 257; // 0x0101: Mode 01, Control 01 (Enable Motor CW)
  static constexpr uint16_t CMD_BRAKE       = 259; // 0x0103: Mode 01, Control 03 (Brake)
  static constexpr uint16_t CMD_ENABLE_CCW  = 265; // 0x0109: Mode 01, Control 09 (Enable Motor CCW)

private:
  void workerLoop(int slave_id);

  modbus_t* ctx_;
  bool connected_;
  std::mutex modbus_mutex_; // To protect concurrent access to modbus context
  uint16_t last_control_val_{0};
  int last_freq_hz_{-1};

  // Background worker variables
  std::atomic<bool> worker_running_{false};
  std::thread worker_thread_;
  std::atomic<double> target_velocity_{0.0};
  std::atomic<double> feedback_velocity_{0.0};
  std::atomic<double> feedback_position_{0.0};
  double internal_position_{0.0};
  std::chrono::steady_clock::time_point last_feedback_time_;
};

}  // namespace autonav_firmware

#endif  // AUTONAV_FIRMWARE__MOTOR_CONTROLLER_HPP_

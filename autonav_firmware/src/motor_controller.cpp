#include "autonav_firmware/motor_controller.hpp"
#include <iostream>
#include <cmath>

namespace autonav_firmware
{

MotorController::MotorController()
: ctx_(nullptr), connected_(false)
{
}

MotorController::~MotorController()
{
  stopWorker();
  disconnect();
}

bool MotorController::connect(const std::string& port, int baudrate)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  
  if (connected_) {
    return true;
  }

  // Initialize Modbus RTU context
  // 'N' for No parity, 8 data bits, 1 stop bit
  ctx_ = modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1);
  
  if (ctx_ == nullptr) {
    std::cerr << "Unable to create Modbus RTU context" << std::endl;
    return false;
  }

  // Set timeout to 0.05 seconds (50,000 microseconds)
  modbus_set_response_timeout(ctx_, 0, 50000);

  // Connect
  if (modbus_connect(ctx_) == -1) {
    std::cerr << "Modbus connection failed: " << modbus_strerror(errno) << std::endl;
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  // Small delay to allow the serial port to settle
  usleep(100000);

  connected_ = true;
  return true;
}

void MotorController::disconnect()
{
  stopWorker();
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (ctx_ != nullptr) {
    if (connected_) {
      modbus_close(ctx_);
    }
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
  connected_ = false;
  last_control_val_ = 0;
  last_freq_hz_ = -1;
}

bool MotorController::enableMotor(int slave_id)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }
  modbus_set_slave(ctx_, slave_id);

  // Initialize motor in Brake mode (259: 0x0103) with speed 0 so it starts safely held
  if (modbus_write_register(ctx_, REG_CONTROL_MODE, CMD_BRAKE) == -1) {
    std::cerr << "Error: Could not set brake mode for motor " << slave_id << ": " << modbus_strerror(errno) << std::endl;
    return false;
  }
  last_control_val_ = CMD_BRAKE;

  modbus_write_register(ctx_, REG_SPEED_COMMAND, 0);
  last_freq_hz_ = 0;

  return true;
}

bool MotorController::brakeMotor(int slave_id)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }
  modbus_set_slave(ctx_, slave_id);

  // Brake in Digital Closed Loop Mode: Write 0103 hex (259) to Address 2
  if (modbus_write_register(ctx_, REG_CONTROL_MODE, CMD_BRAKE) == -1) {
    return false;
  }
  last_control_val_ = CMD_BRAKE;

  if (last_freq_hz_ != 0) {
    modbus_write_register(ctx_, REG_SPEED_COMMAND, 0);
    last_freq_hz_ = 0;
  }

  return true;
}

bool MotorController::disableMotor(int slave_id)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }
  modbus_set_slave(ctx_, slave_id);

  // Disable motors: Write 0100 hex (256) to Address 2
  if (modbus_write_register(ctx_, REG_CONTROL_MODE, CMD_DISABLE) == -1) {
    return false;
  }
  last_control_val_ = CMD_DISABLE;
  return true;
}

bool MotorController::setVelocity(int slave_id, double velocity)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  // Handle NaN gracefully
  if (std::isnan(velocity)) {
    velocity = 0.0;
  }

  modbus_set_slave(ctx_, slave_id);

  // Stop / Brake condition (within 0.001 rad/s deadband)
  if (std::abs(velocity) < 0.001) {
    // When stopping, put motor into Brake (259: 0x0103) per RMCS-3001 datasheet
    // Mode byte: 01 (Digital Closed Loop), Control byte: 03 (Brake)
    // Note: Brake has higher priority than enable, disabling driving loop and locking motor.
    // Avoids jerking forward from CCW to CW when stopping.
    if (last_control_val_ != CMD_BRAKE) {
      if (modbus_write_register(ctx_, REG_CONTROL_MODE, CMD_BRAKE) == -1) {
        std::cerr << "Failed to apply brake for slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
        return false;
      }
      last_control_val_ = CMD_BRAKE;
    }

    // Set speed to 0 if not already 0
    if (last_freq_hz_ != 0) {
      if (modbus_write_register(ctx_, REG_SPEED_COMMAND, 0) == -1) {
        std::cerr << "Failed to write 0 speed for slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
        return false;
      }
      last_freq_hz_ = 0;
    }

    return true;
  }

  // Active driving calculation
  // velocity is in rad/s from ros2_control
  double gearbox_ratio = 13.0;
  double pole_pairs = 4.0;
  
  double target_output_rpm = velocity * (60.0 / (2.0 * M_PI));
  double motor_rpm = target_output_rpm * gearbox_ratio;
  
  // Calculate absolute frequency (must be between 0 and 400 Hz)
  int freq_hz = static_cast<int>((std::abs(motor_rpm) * pole_pairs) / 60.0);
  if (freq_hz > 400) {
    freq_hz = 400; // Cap at maximum frequency per datasheet
  }

  // Direction:
  // Forward (> 0): CW  (0x0101) = 257
  // Reverse (< 0): CCW (0x0109) = 265
  uint16_t control_val = (velocity > 0.0) ? CMD_ENABLE_CW : CMD_ENABLE_CCW;

  if (control_val != last_control_val_) {
    if (modbus_write_register(ctx_, REG_CONTROL_MODE, control_val) == -1) {
      std::cerr << "Failed to write direction for slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
      return false;
    }
    last_control_val_ = control_val;
  }

  // Write to speed command register
  int rc = modbus_write_register(ctx_, REG_SPEED_COMMAND, freq_hz);
  if (rc == -1) {
    std::cerr << "Failed to write velocity for slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
    return false;
  }
  last_freq_hz_ = freq_hz;

  return true;
}

bool MotorController::startWorker(int slave_id)
{
  if (worker_running_) {
    return true;
  }
  worker_running_ = true;
  worker_thread_ = std::thread(&MotorController::workerLoop, this, slave_id);
  return true;
}

void MotorController::stopWorker()
{
  if (worker_running_) {
    worker_running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }
}

void MotorController::setTargetVelocity(double velocity)
{
  target_velocity_.store(velocity);
}

double MotorController::getFeedbackVelocity() const
{
  return feedback_velocity_.load();
}

double MotorController::getFeedbackPosition() const
{
  return feedback_position_.load();
}

void MotorController::workerLoop(int slave_id)
{
  while (worker_running_) {
    // 1. Send latest command
    double cmd = target_velocity_.load();
    setVelocity(slave_id, cmd);

    // 2. Read latest feedback
    double pos = 0.0, vel = 0.0;
    if (readFeedback(slave_id, pos, vel)) {
      feedback_position_.store(pos);
      feedback_velocity_.store(vel);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool MotorController::readFeedback(int slave_id, double& position, double& velocity)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  modbus_set_slave(ctx_, slave_id);
  uint16_t tab_reg[4];

  // Try to read speed feedback
  int rc = modbus_read_registers(ctx_, REG_SPEED_FEEDBACK, 1, tab_reg);
  if (rc == -1) {
    // If it occasionally fails, it's normal in serial. We just return false.
    return false;
  }

  // Convert read value to signed 16-bit
  int16_t raw_speed = static_cast<int16_t>(tab_reg[0]);
  double avg_freq = static_cast<double>(raw_speed);

  double pole_pairs = 4.0;
  double gearbox_ratio = 13.0;

  double motor_current_rpm = (60.0 * avg_freq) / pole_pairs;
  double output_current_rpm = motor_current_rpm / gearbox_ratio;
  
  // Convert RPM to rad/s
  velocity = output_current_rpm * (2.0 * M_PI / 60.0);

  // Register 8 returns absolute frequency magnitude.
  // Negate if moving CCW (reverse), or 0 if braked/disabled:
  if (last_control_val_ == CMD_ENABLE_CCW) {
    velocity = -velocity;
  } else if (last_control_val_ == CMD_BRAKE || last_control_val_ == CMD_DISABLE) {
    velocity = 0.0;
  }

  // Integrate position over dt
  auto now = std::chrono::steady_clock::now();
  if (last_feedback_time_.time_since_epoch().count() > 0) {
    double dt = std::chrono::duration<double>(now - last_feedback_time_).count();
    internal_position_ += velocity * dt;
  }
  last_feedback_time_ = now;
  position = internal_position_;

  return true;
}

}  // namespace autonav_firmware

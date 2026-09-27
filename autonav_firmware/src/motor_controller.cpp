#include "autonav_firmware/motor_controller.hpp"
#include <iostream>

namespace autonav_firmware
{

MotorController::MotorController()
: ctx_(nullptr), connected_(false)
{
}

MotorController::~MotorController()
{
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

  // Set recovery mode
  modbus_set_error_recovery(ctx_,
    static_cast<modbus_error_recovery_mode>(MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));

  // Connect
  if (modbus_connect(ctx_) == -1) {
    std::cerr << "Modbus connection failed: " << modbus_strerror(errno) << std::endl;
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  connected_ = true;
  return true;
}

void MotorController::disconnect()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (ctx_ != nullptr) {
    if (connected_) {
      modbus_close(ctx_);
    }
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
  connected_ = false;
}

bool MotorController::setVelocity(int slave_id, double velocity)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  // Set the target slave ID for this transaction
  modbus_set_slave(ctx_, slave_id);

  // Convert double velocity (likely rad/s or m/s) to motor's internal unit (e.g. RPM)
  // Assuming a direct conversion factor here. E.g., if velocity is in RPM:
  int16_t int_vel = static_cast<int16_t>(velocity);

  // Write to speed command register
  int rc = modbus_write_register(ctx_, REG_SPEED_COMMAND, int_vel);
  if (rc == -1) {
    std::cerr << "Failed to write velocity for slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
    return false;
  }

  return true;
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
  velocity = static_cast<double>(raw_speed);

  // For RMCS-3001, if position is needed, you might need to read multiple registers.
  // Here we use a placeholder logic. Update with correct register if the drive provides position feedback.
  position += (velocity * 0.1); // Fake integration for position if no real register

  return true;
}

}  // namespace autonav_firmware

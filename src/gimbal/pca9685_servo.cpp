/**
 * @file pca9685_servo.cpp
 * @brief Advanced PCA9685 Servo Controller Implementation
 * @version 2.0.0
 * @date 2025-01-04
 */

#include "pca9685_servo.h"

#include <cstring>

namespace dof
{

// Global servo controller instance
static PCA9685ServoController* g_servo_controller = nullptr;

bool PCA9685ServoController::initialize(I2C_HandleTypeDef* hi2c)
{
    hi2c_ = hi2c;
    emergency_stop_active_ = false;
    last_update_time_ = HAL_GetTick();

    if (!hi2c_)
    {
        return false;
    }

    // Debug: Check I2C bus status first
    // HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(hi2c_);

    // Simple device ready check with short timeout to avoid hanging
    // HAL_StatusTypeDef device_ready_result = HAL_I2C_IsDeviceReady(
    //     hi2c_,
    //     PCA9685_ADDRESS << 1,
    //     1,
    //     100);
    // if (device_ready_result != HAL_OK)
    // {
    //     // Try once more with longer timeout
    //     device_ready_result = HAL_I2C_IsDeviceReady(
    //         hi2c_,
    //         PCA9685_ADDRESS << 1,
    //         1,
    //         500);
    //     if (device_ready_result != HAL_OK)
    //     {
    //         // Store debug info for later retrieval
    //         return false;
    //     }
    // }

    // Initialize a telemetry array
    for (size_t i = 0; i < telemetry_.size(); ++i)
    {
        auto& telem = telemetry_[i];
        telem.id = indexToServoID(i);
        telem.current_position = getServoConfig(telem.id).home_position;
        telem.target_position = telem.current_position;
        telem.current_speed = 0;
        telem.status = ServoStatus::IDLE;
        telem.error = ServoError::NONE;
        telem.temperature = 25;
        telem.load_percentage = 0;
        telem.timestamp = last_update_time_;
        telem.voltage_mv = 5000; // Assume 5V
    }

    /*
    uint8_t data;

    // Reset PCA9685
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c_, PCA9685_ADDRESS << 1, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return false;
    }
    HAL_Delay(10);

    // Set PWM frequency to 50Hz for servos
    data = 0x10; // Sleep mode
    if (HAL_I2C_Mem_Write(hi2c_, PCA9685_ADDRESS << 1, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return false;
    }

    // Set prescaler for 50Hz
    data = 0x79; // 50Hz prescaler value
    if (HAL_I2C_Mem_Write(hi2c_, PCA9685_ADDRESS << 1, PCA9685_PRESCALE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return false;
    }

    // Wake up PCA9685
    data = 0x00; // Normal mode
    if (HAL_I2C_Mem_Write(hi2c_, PCA9685_ADDRESS << 1, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return false;
    }
    HAL_Delay(5);

    // Enable auto-increment mode
    data = 0xA0; // Auto-increment + restart
    if (HAL_I2C_Mem_Write(hi2c_, PCA9685_ADDRESS << 1, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return false;
    }
    */

    // Move all servos to home position
    return true;
    // return homeAllServos();
}

uint16_t PCA9685ServoController::angleToPulseWidth(
    ServoID id,
    uint16_t angle_degrees)
{
    const auto config = getServoConfig(id);

    // Clamp angle to servo constraints
    const auto& constraints = config.constraints;
    if (angle_degrees < constraints.min_angle)
        angle_degrees = constraints.min_angle;
    if (angle_degrees > constraints.max_angle)
        angle_degrees = constraints.max_angle;

    // Convert angle to pulse width using servo config
    uint16_t pulse_width = config.min_pulse_width_us +
                           ((angle_degrees * (
                                 config.max_pulse_width_us - config.
                                 min_pulse_width_us)) / 180);

    // Convert microseconds to PCA9685 ticks (50Hz = 20ms period, 4096 ticks)
    // 1 tick = 20000us / 4096 = 4.88us per tick
    pulse_width = (pulse_width * 4096) / 20000;

    return pulse_width;
}

bool PCA9685ServoController::validateCommand(const ServoCommand& command) const
{
    if (!isValidServoID(command.servo_id))
    {
        return false;
    }

    if (emergency_stop_active_ && command.type != CommandType::EMERGENCY_STOP)
    {
        return false;
    }

    const auto& constraints = getServoConstraints(command.servo_id);

    switch (command.type)
    {
        case CommandType::MOVE_TO_POSITION: return (
                command.target_position >= constraints.min_angle &&
                command.target_position <= constraints.max_angle &&
                command.speed >= constraints.min_speed &&
                command.speed <= constraints.max_speed);

        case CommandType::SET_SPEED: return (
                command.speed >= constraints.min_speed &&
                command.speed <= constraints.max_speed);

        default: return true;
    }
}

void PCA9685ServoController::updateTelemetry(
    ServoID id,
    uint16_t target_angle,
    ServoStatus status,
    ServoError error)
{
    if (!isValidServoID(id))
    {
        return;
    }

    auto& telem = telemetry_[servoIDToIndex(id)];
    telem.target_position = target_angle;
    telem.status = status;
    telem.error = error;
    telem.timestamp = HAL_GetTick();
}

void PCA9685ServoController::setPCA9685Channel(
    uint8_t channel,
    uint16_t pulse_width) const
{
    if (!hi2c_ || channel >= 16)
    {
        return;
    }

    uint8_t reg = PCA9685_LED0_ON_L + (channel * 4);
    uint8_t data[4];

    data[0] = 0x00;                      // ON_L (start at 0)
    data[1] = 0x00;                      // ON_H (start at 0)
    data[2] = pulse_width & 0xFF;        // OFF_L
    data[3] = (pulse_width >> 8) & 0xFF; // OFF_H

    HAL_I2C_Mem_Write(
        hi2c_,
        PCA9685_ADDRESS << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        4,
        100);
}

bool PCA9685ServoController::executeCommand(const ServoCommand& command)
{
    if (!validateCommand(command))
    {
        updateTelemetry(
            command.servo_id,
            command.target_position,
            ServoStatus::ERROR,
            ServoError::POSITION_LIMIT_EXCEEDED);
        return false;
    }

    switch (command.type)
    {
        case CommandType::MOVE_TO_POSITION: return setServoAngle(
                command.servo_id,
                command.target_position,
                command.speed);

        case CommandType::STOP: updateTelemetry(
                command.servo_id,
                telemetry_[servoIDToIndex(command.servo_id)].current_position,
                ServoStatus::IDLE);
            return true;

        case CommandType::HOME: return setServoAngle(
                command.servo_id,
                getServoConfig(command.servo_id).home_position);

        case CommandType::EMERGENCY_STOP: emergencyStop();
            return true;

        default: return false;
    }
}

bool PCA9685ServoController::setServoAngle(
    ServoID id,
    uint16_t angle_degrees,
    uint8_t speed)
{
    if (!isValidServoID(id) || emergency_stop_active_)
    {
        return false;
    }

    const auto config = getServoConfig(id);
    const auto& constraints = config.constraints;

    // Validate angle within constraints
    if (angle_degrees < constraints.min_angle || angle_degrees > constraints.
        max_angle)
    {
        updateTelemetry(
            id,
            angle_degrees,
            ServoStatus::ERROR,
            ServoError::POSITION_LIMIT_EXCEEDED);
        return false;
    }

    // Convert angle to pulse width
    uint16_t pulse_width = angleToPulseWidth(id, angle_degrees);

    // Set servo position
    setPCA9685Channel(config.pwm_channel, pulse_width);

    // Update telemetry
    auto& telem = telemetry_[servoIDToIndex(id)];
    telem.current_position = angle_degrees;
    telem.target_position = angle_degrees;
    telem.current_speed = speed;
    telem.status = ServoStatus::IDLE; // Assuming instant movement for now
    telem.error = ServoError::NONE;
    telem.timestamp = HAL_GetTick();

    return true;
}

bool PCA9685ServoController::homeAllServos()
{
    bool success = true;

    for (const auto servo_id: getAllServoIDs())
    {
        const auto config = getServoConfig(servo_id);
        if (!setServoAngle(servo_id, config.home_position))
        {
            success = false;
        }
    }

    return success;
}

bool PCA9685ServoController::moveToPreset(
    const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>& positions)
{
    bool success = true;

    for (size_t i = 0; i < positions.size(); ++i)
    {
        ServoID id = indexToServoID(i);
        if (!setServoAngle(id, positions[i]))
        {
            success = false;
        }
    }

    return success;
}

void PCA9685ServoController::emergencyStop()
{
    emergency_stop_active_ = true;

    // Move all servos to safe positions (home)
    homeAllServos();

    // Update telemetry for all servos
    for (size_t i = 0; i < telemetry_.size(); ++i)
    {
        auto& telem = telemetry_[i];
        telem.status = ServoStatus::EMERGENCY_STOP;
        telem.current_speed = 0;
        telem.timestamp = HAL_GetTick();
    }
}

void PCA9685ServoController::clearEmergencyStop()
{
    emergency_stop_active_ = false;

    // Update telemetry for all servos
    for (size_t i = 0; i < telemetry_.size(); ++i)
    {
        auto& telem = telemetry_[i];
        if (telem.status == ServoStatus::EMERGENCY_STOP)
        {
            telem.status = ServoStatus::IDLE;
        }
        telem.timestamp = HAL_GetTick();
    }
}

ServoTelemetry PCA9685ServoController::getTelemetry(ServoID id) const
{
    if (isValidServoID(id))
    {
        return telemetry_[servoIDToIndex(id)];
    }
    return ServoTelemetry{};
}

bool PCA9685ServoController::isServoMoving(ServoID id) const
{
    if (isValidServoID(id))
    {
        return telemetry_[servoIDToIndex(id)].status == ServoStatus::MOVING;
    }
    return false;
}

void PCA9685ServoController::updateTelemetry()
{
    uint32_t current_time = HAL_GetTick();

    for (auto& telem: telemetry_)
    {
        // Simple telemetry update - in a real system you'd read actual servo feedback
        if (telem.status == ServoStatus::MOVING)
        {
            // Check if movement should be completed (simplified)
            if (current_time - telem.timestamp > 1000)
            {
                // 1 second movement time
                telem.status = ServoStatus::IDLE;
                telem.current_speed = 0;
            }
        }

        telem.timestamp = current_time;
    }

    last_update_time_ = current_time;
}

bool PCA9685ServoController::isSystemHealthy() const
{
    for (const auto& telem: telemetry_)
    {
        if (telem.error != ServoError::NONE)
        {
            return false;
        }
    }
    return !emergency_stop_active_;
}

bool PCA9685ServoController::isConnected() const
{
    if (!hi2c_)
    {
        return false;
    }

    // Quick device ready check  
    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(
        hi2c_,
        PCA9685_ADDRESS << 1,
        1,
        100);
    return (result == HAL_OK);
}

} // namespace dof

// C-style interface implementation
extern "C"
{
bool PCA9685_Init(I2C_HandleTypeDef* hi2c)
{
    static dof::PCA9685ServoController controller;
    if (controller.initialize(hi2c))
    {
        dof::g_servo_controller = &controller;
        return true;
    }
    return false;
}

bool PCA9685_SetServoAngle(uint8_t servo_id, uint16_t angle_degrees)
{
    if (!dof::g_servo_controller || servo_id >= static_cast<uint8_t>(
            dof::ServoID::COUNT))
    {
        return false;
    }
    return dof::g_servo_controller->setServoAngle(
        dof::indexToServoID(servo_id),
        angle_degrees);
}

bool PCA9685_HomeAllServos()
{
    if (!dof::g_servo_controller)
    {
        return false;
    }
    return dof::g_servo_controller->homeAllServos();
}

bool PCA9685_EmergencyStop()
{
    if (!dof::g_servo_controller)
    {
        return false;
    }
    dof::g_servo_controller->emergencyStop();
    return true;
}

bool PCA9685_ClearEmergencyStop()
{
    if (!dof::g_servo_controller)
    {
        return false;
    }
    dof::g_servo_controller->clearEmergencyStop();
    return true;
}

const char* PCA9685_GetServoStatus(uint8_t servo_id)
{
    if (!dof::g_servo_controller || servo_id >= static_cast<uint8_t>(
            dof::ServoID::COUNT))
    {
        return "ERROR";
    }

    auto telem = dof::g_servo_controller->getTelemetry(
        dof::indexToServoID(servo_id));
    switch (telem.status)
    {
        case dof::ServoStatus::IDLE: return "IDLE";
        case dof::ServoStatus::MOVING: return "MOVING";
        case dof::ServoStatus::ERROR: return "ERROR";
        case dof::ServoStatus::CALIBRATING: return "CALIBRATING";
        case dof::ServoStatus::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

bool PCA9685_IsConnected()
{
    if (!dof::g_servo_controller)
    {
        return false;
    }

    return dof::g_servo_controller->isConnected();
}
}

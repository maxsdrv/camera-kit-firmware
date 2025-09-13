/**
 * @file pca9685_servo.h  
 * @brief Advanced PCA9685 Servo Controller for DOF Camera Gimbal
 * @version 2.0.0
 * @date 2025-01-04
 */

#pragma once

#include "main.h"
#include "types/servo_types.h"
#include "configs/servo_config.h"

namespace dof
{

/**
 * @brief Advanced PCA9685 Servo Controller Class
 * 
 * Provides angle-based servo control with safety constraints,
 * telemetry, and error handling for 6-DOF camera gimbal
 */
class PCA9685ServoController
{

public:
    PCA9685ServoController() = default;
    /**
     * @brief Initialize PCA9685 and servo system
     */
    bool initialize(I2C_HandleTypeDef* hi2c);

    /**
     * @brief Execute servo command with safety checks
     */
    bool executeCommand(const ServoCommand& command);

    /**
     * @brief Set servo to specific angle with constraints
     */
    bool setServoAngle(ServoID id, uint16_t angle_degrees, uint8_t speed = 100);

    /**
     * @brief Move all servos to home position
     */
    bool homeAllServos();

    /**
     * @brief Move servos to predefined preset
     */
    bool moveToPreset(
        const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>&
        positions);

    /**
     * @brief Emergency stop all servos
     */
    void emergencyStop();

    /**
     * @brief Clear emergency stop state
     */
    void clearEmergencyStop();

    /**
     * @brief Get current telemetry for specific servo
     */
    [[nodiscard]] ServoTelemetry getTelemetry(ServoID id) const;

    /**
     * @brief Get telemetry for all servos
     */
    [[nodiscard]] const TelemetryArray& getAllTelemetry() const
    {
        return telemetry_;
    }

    /**
     * @brief Check if servo is currently moving
     */
    [[nodiscard]] bool isServoMoving(ServoID id) const;

    /**
     * @brief Update telemetry (call periodically)
     */
    void updateTelemetry();

    /**
     * @brief Check system health
     */
    [[nodiscard]] bool isSystemHealthy() const;

    /**
     * @brief Check if PCA9685 is connected and responding
     */
    [[nodiscard]] bool isConnected() const;

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;
    TelemetryArray telemetry_{};
    uint32_t last_update_time_ = 0;
    bool emergency_stop_active_ = false;

    // PCA9685 Constants
    static constexpr uint8_t PCA9685_ADDRESS = 0x40;
    static constexpr uint8_t PCA9685_MODE1 = 0x00;
    static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
    static constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

    // Internal helper methods
    static uint16_t angleToPulseWidth(ServoID id, uint16_t angle_degrees);
    [[nodiscard]] bool validateCommand(const ServoCommand& command) const;
    void updateTelemetry(
        ServoID id,
        uint16_t target_angle,
        ServoStatus status,
        ServoError error = ServoError::NONE);
    void setPCA9685Channel(uint8_t channel, uint16_t pulse_width) const;
};

} // namespace dof

// C-style interface for compatibility with existing code
extern "C"
{
bool PCA9685_Init(I2C_HandleTypeDef* hi2c);
bool PCA9685_SetServoAngle(uint8_t servo_id, uint16_t angle_degrees);
bool PCA9685_HomeAllServos();
bool PCA9685_EmergencyStop();
bool PCA9685_ClearEmergencyStop();
const char* PCA9685_GetServoStatus(uint8_t servo_id);
bool PCA9685_IsConnected();
}

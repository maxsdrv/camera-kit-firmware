#pragma once

#include "../types/servo_types.h"
#include "../types/iservo_controller.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace DOF {

// Forward declarations for STM32 HAL
// These will be included via stm32f4xx_hal.h
struct GPIO_InitTypeDef;

/**
 * @brief PWM-based servo controller for MG996R servos
 * 
 * This controller uses PWM signals to control standard hobby servos.
 * It supports acceleration/deceleration, position limits, and real-time
 * status monitoring.
 */
class PWMServoController : public IServoController {
public:
    // Constructor with servo configurations
    explicit PWMServoController(const std::span<const ServoConfig>& configs);
    ~PWMServoController() override;

    // IServoController implementation
    bool initialize() override;
    void shutdown() override;
    bool isInitialized() const override;

    // Individual servo control
    bool moveServo(ServoID id, uint16_t position, uint8_t speed = 128) override;
    bool stopServo(ServoID id) override;
    bool homeServo(ServoID id) override;
    bool setServoSpeed(ServoID id, uint8_t speed) override;

    // Multi-servo control
    bool moveMultipleServos(std::span<const ServoCommand> commands) override;
    bool stopAllServos() override;
    bool homeAllServos() override;
    bool emergencyStop() override;

    // Status and telemetry
    ServoTelemetry getServoTelemetry(ServoID id) const override;
    TelemetryArray getAllServoTelemetry() const override;
    ServoStatus getServoStatus(ServoID id) const override;
    bool isServoMoving(ServoID id) const override;

    // Configuration
    bool setServoConstraints(ServoID id, const ServoConstraints& constraints) override;
    ServoConstraints getServoConstraints(ServoID id) const override;
    bool calibrateServo(ServoID id) override;

    // System control
    void update() override;
    bool isSystemHealthy() const override;
    void resetErrors() override;

    // Observer management
    void addObserver(std::shared_ptr<IServoObserver> observer);
    void removeObserver(std::shared_ptr<IServoObserver> observer);

private:
    // Internal servo state
    struct ServoState {
        ServoConfig config;
        ServoTelemetry telemetry;
        uint16_t current_position;
        uint16_t target_position;
        uint8_t current_speed;
        uint8_t target_speed;
        uint16_t acceleration;
        uint16_t deceleration;
        ServoStatus status;
        ServoError error;
        uint32_t movement_start_time;
        uint32_t last_update_time;
        bool is_moving;
        bool is_homed;
    };

    // Private methods
    bool initializePWM();
    bool initializeGPIO();
    void updateServoState(ServoState& servo, uint32_t current_time);
    bool validatePosition(ServoID id, uint16_t position) const;
    uint16_t angleToPulseWidth(const ServoConfig& config, uint16_t angle) const;
    void setPWMPulseWidth(uint8_t channel, uint16_t pulse_width_us);
    void notifyObservers();
    void notifyServoPositionChanged(ServoID id, uint16_t oldPos, uint16_t newPos);
    void notifyServoStatusChanged(ServoID id, ServoStatus oldStatus, ServoStatus newStatus);
    void notifyServoError(ServoID id, ServoError error);
    
    // Observer notification methods
    void notifyServoMovementStarted(ServoID id, uint16_t targetPosition);
    void notifyServoMovementCompleted(ServoID id, uint16_t finalPosition);

    // Member variables
    std::array<ServoState, static_cast<size_t>(ServoID::COUNT)> servo_states_;
    std::vector<std::shared_ptr<IServoObserver>> observers_;
    bool initialized_;
    bool emergency_stop_active_;
    uint32_t system_start_time_;
    
    // STM32 HAL handles (these would be initialized by STM32CubeMX)
    TIM_HandleTypeDef* pwm_timer_;
    
    // Constants
    static constexpr uint16_t DEFAULT_PWM_FREQUENCY_HZ = 50;
    static constexpr uint16_t DEFAULT_MIN_PULSE_WIDTH_US = 500;   // 0.5ms
    static constexpr uint16_t DEFAULT_MAX_PULSE_WIDTH_US = 2500;  // 2.5ms
    static constexpr uint16_t DEFAULT_ACCELERATION = 90;          // 90°/sec²
    static constexpr uint16_t DEFAULT_DECELERATION = 90;          // 90°/sec²
    static constexpr uint32_t UPDATE_RATE_HZ = 100;               // 100Hz update rate
    static constexpr uint32_t UPDATE_PERIOD_MS = 1000 / UPDATE_RATE_HZ;
};

/**
 * @brief Factory for creating PWM servo controllers
 */
class PWMServoControllerFactory : public IServoControllerFactory {
public:
    std::unique_ptr<IServoController> createController(
        const std::span<const ServoConfig>& configs) override;
};

} // namespace DOF

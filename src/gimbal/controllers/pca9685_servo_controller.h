#pragma once

#include "../types/servo_types.h"
#include "../types/iservo_controller.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace DOF {

// Forward declarations for STM32 HAL
// These will be included via stm32f4xx_hal.h

/**
 * @brief PCA9685-based servo controller for MG996R servos
 * 
 * This controller uses the PCA9685 I2C PWM controller to drive multiple servos.
 * It provides precise 12-bit PWM control with configurable frequency and
 * supports acceleration/deceleration, position limits, and real-time monitoring.
 */
class PCA9685ServoController : public IServoController {
public:
    // Constructor with servo configurations and I2C settings
    explicit PCA9685ServoController(const std::span<const ServoConfig>& configs,
                                   I2C_HandleTypeDef* i2c_handle,
                                   uint8_t pca9685_address = 0x40);
    ~PCA9685ServoController() override;

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
    [[nodiscard]] ServoTelemetry getServoTelemetry(ServoID id) const override;
    [[nodiscard]] TelemetryArray getAllServoTelemetry() const override;
    [[nodiscard]] ServoStatus getServoStatus(ServoID id) const override;
    [[nodiscard]] bool isServoMoving(ServoID id) const override;

    // Configuration
    bool setServoConstraints(ServoID id, const ServoConstraints& constraints) override;
    [[nodiscard]] ServoConstraints getServoConstraints(ServoID id) const override;
    bool calibrateServo(ServoID id) override;

    // System control
    void update() override;
    [[nodiscard]] bool isSystemHealthy() const override;
    void resetErrors() override;

    // Observer management
    void addObserver(std::shared_ptr<IServoObserver> observer);
    void removeObserver(std::shared_ptr<IServoObserver> observer);

    // PCA9685 specific methods
    bool setPWMFrequency(uint16_t frequency_hz);
    [[nodiscard]] uint16_t getPWMFrequency() const;
    bool sleep(bool enable);
    bool isSleeping() const;
    bool resetPCA9685();

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
        uint16_t current_pwm_value;  // Current PWM value (0-4095)
    };

    // PCA9685 register addresses
    static constexpr uint8_t PCA9685_MODE1 = 0x00;
    static constexpr uint8_t PCA9685_MODE2 = 0x01;
    static constexpr uint8_t PCA9685_SUBADR1 = 0x02;
    static constexpr uint8_t PCA9685_SUBADR2 = 0x03;
    static constexpr uint8_t PCA9685_SUBADR3 = 0x04;
    static constexpr uint8_t PCA9685_ALLCALLADR = 0x05;
    static constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
    static constexpr uint8_t PCA9685_LED0_ON_H = 0x07;
    static constexpr uint8_t PCA9685_LED0_OFF_L = 0x08;
    static constexpr uint8_t PCA9685_LED0_OFF_H = 0x09;
    static constexpr uint8_t PCA9685_ALLLED_ON_L = 0xFA;
    static constexpr uint8_t PCA9685_ALLLED_ON_H = 0xFB;
    static constexpr uint8_t PCA9685_ALLLED_OFF_L = 0xFC;
    static constexpr uint8_t PCA9685_ALLLED_OFF_H = 0xFD;
    static constexpr uint8_t PCA9685_PRESCALE = 0xFE;

    // PCA9685 mode bits
    static constexpr uint8_t MODE1_SLEEP = 0x10;
    static constexpr uint8_t MODE1_AUTOINC = 0x20;
    static constexpr uint8_t MODE1_RESTART = 0x80;

    // Private methods
    bool initializePCA9685();
    bool initializeGPIO();
    void updateServoState(ServoState& servo, uint32_t current_time);
    [[nodiscard]] bool validatePosition(ServoID id, uint16_t position) const;
    [[nodiscard]] uint16_t angleToPWMValue(const ServoConfig& config, uint16_t angle) const;
    bool setPCA9685PWM(uint8_t channel, uint16_t on_value, uint16_t off_value);
    
    // Observer notification methods
    void notifyServoMovementStarted(ServoID id, uint16_t targetPosition);
    void notifyServoMovementCompleted(ServoID id, uint16_t finalPosition);
    bool setPCA9685Channel(uint8_t channel, uint16_t pwm_value);
    bool writePCA9685Register(uint8_t reg, uint8_t value);
    uint8_t readPCA9685Register(uint8_t reg);
    bool writePCA9685Registers(uint8_t start_reg, const std::span<const uint8_t>& data);
    void notifyObservers();
    void notifyServoPositionChanged(ServoID id, uint16_t oldPos, uint16_t newPos);
    void notifyServoStatusChanged(ServoID id, ServoStatus oldStatus, ServoStatus newStatus);
    void notifyServoError(ServoID id, ServoError error);

    // Member variables
    std::array<ServoState, static_cast<size_t>(ServoID::COUNT)> servo_states_{};
    std::vector<std::shared_ptr<IServoObserver>> observers_;
    bool initialized_;
    bool emergency_stop_active_;
    uint32_t system_start_time_;
    
    // PCA9685 specific variables
    I2C_HandleTypeDef* i2c_handle_;
    uint8_t pca9685_address_;
    uint16_t pwm_frequency_hz_;
    bool sleeping_;
    
    // Constants
    static constexpr uint16_t DEFAULT_PWM_FREQUENCY_HZ = 50;
    static constexpr uint16_t DEFAULT_MIN_PULSE_WIDTH_US = 500;   // 0.5ms
    static constexpr uint16_t DEFAULT_MAX_PULSE_WIDTH_US = 2500;  // 2.5ms
    static constexpr uint16_t DEFAULT_ACCELERATION = 90;          // 90°/sec²
    static constexpr uint16_t DEFAULT_DECELERATION = 90;          // 90°/sec²
    static constexpr uint32_t UPDATE_RATE_HZ = 100;               // 100Hz update rate
    static constexpr uint32_t UPDATE_PERIOD_MS = 1000 / UPDATE_RATE_HZ;
    static constexpr uint16_t PWM_RESOLUTION = 4096;              // 12-bit resolution
    static constexpr uint32_t I2C_TIMEOUT_MS = 100;               // I2C timeout
};

/**
 * @brief Factory for creating PCA9685 servo controllers
 */
class PCA9685ServoControllerFactory : public IServoControllerFactory {
public:
    explicit PCA9685ServoControllerFactory(I2C_HandleTypeDef* i2c_handle,
                                         uint8_t pca9685_address = 0x40);
    
    std::unique_ptr<IServoController> createController(
        const std::span<const ServoConfig>& configs) override;

private:
    I2C_HandleTypeDef* i2c_handle_;
    uint8_t pca9685_address_;
};

} // namespace DOF

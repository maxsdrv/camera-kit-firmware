#pragma once

#include "../configs/servo_config.h"
#include "../types/servo_types.h"
#include "pca9685_servo_controller.h"
#include <memory>
#include <vector>

namespace DOF {

// Forward declarations
class IServoObserver;

/**
 * @brief Main DOF Robotic Arm controller
 * 
 * This class provides a high-level interface for controlling the 6-DOF
 * robotic arm. It manages servo movements, safety, and provides
 * convenient methods for common operations.
 */
class DOFRoboticArm {
public:
    // Constructor
    explicit DOFRoboticArm(I2C_HandleTypeDef* i2c_handle,
                           uint8_t pca9685_address = PCA9685_DEFAULT_ADDRESS);
    
    // Destructor
    ~DOFRoboticArm();

    // Initialization and control
    bool initialize();
    void shutdown();
    bool isInitialized() const;

    // High-level movement commands
    bool moveToPosition(const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>& positions,
                       uint8_t speed = 128);
    bool moveToHome();
    bool moveToRest();
    bool moveCameraForward();
    bool moveCameraDown();
    bool moveCameraToAngle(uint16_t pitch, uint16_t roll);

    // Individual servo control
    bool moveServo(ServoID id, uint16_t position, uint8_t speed = 128);
    bool stopServo(ServoID id);
    bool homeServo(ServoID id);

    // Safety and control
    bool emergencyStop();
    bool stopAllServos();
    bool isMoving() const;
    bool isHealthy() const;

    // Status and telemetry
    ServoTelemetry getServoTelemetry(ServoID id) const;
    TelemetryArray getAllServoTelemetry() const;
    ServoStatus getServoStatus(ServoID id) const;

    // Configuration
    bool setServoConstraints(ServoID id, const ServoConstraints& constraints);
    bool calibrateAllServos();

    // Observer management
    void addObserver(std::shared_ptr<IServoObserver> observer);
    void removeObserver(std::shared_ptr<IServoObserver> observer);

    // Main loop update (call this in your main loop)
    void update();

    // Utility methods
    std::string getStatusString() const;
    void printTelemetry() const;

private:
    // Private methods
    bool validatePositions(const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>& positions) const;
    void updateSystemHealth();
    void logMovement(const std::string& movement_type, bool success);

    // Member variables
    std::unique_ptr<IServoController> servo_controller_;
    std::vector<std::shared_ptr<IServoObserver>> observers_;
    bool initialized_;
    bool emergency_stop_active_;
    uint32_t last_health_check_;
    uint32_t system_start_time_;
    
    // System state
    bool system_healthy_;
    uint8_t active_movements_;
    uint32_t last_telemetry_update_;
    
    // Configuration
    I2C_HandleTypeDef* i2c_handle_;
    uint8_t pca9685_address_;
    
    // Constants
    static constexpr uint32_t HEALTH_CHECK_INTERVAL_MS = 1000;  // 1 second
    static constexpr uint32_t TELEMETRY_UPDATE_INTERVAL_MS = 10; // 100Hz
};

} // namespace DOF

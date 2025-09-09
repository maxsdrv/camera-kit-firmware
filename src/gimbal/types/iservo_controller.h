#pragma once

#include "servo_types.h"
#include <concepts>
#include <memory>
#include <span>

namespace DOF {

/**
 * @brief Abstract interface for servo controllers
 * 
 * This interface defines the contract that all servo controllers must implement.
 * It provides methods for controlling individual servos and managing the overall
 * servo system.
 */
class IServoController {
public:
    virtual ~IServoController() = default;

    // Core servo control methods
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool isInitialized() const = 0;

    // Individual servo control
    virtual bool moveServo(ServoID id, uint16_t position, uint8_t speed = 128) = 0;
    virtual bool stopServo(ServoID id) = 0;
    virtual bool homeServo(ServoID id) = 0;
    virtual bool setServoSpeed(ServoID id, uint8_t speed) = 0;

    // Multi-servo control
    virtual bool moveMultipleServos(std::span<const ServoCommand> commands) = 0;
    virtual bool stopAllServos() = 0;
    virtual bool homeAllServos() = 0;
    virtual bool emergencyStop() = 0;

    // Status and telemetry
    virtual ServoTelemetry getServoTelemetry(ServoID id) const = 0;
    virtual TelemetryArray getAllServoTelemetry() const = 0;
    virtual ServoStatus getServoStatus(ServoID id) const = 0;
    virtual bool isServoMoving(ServoID id) const = 0;

    // Configuration
    virtual bool setServoConstraints(ServoID id, const ServoConstraints& constraints) = 0;
    virtual ServoConstraints getServoConstraints(ServoID id) const = 0;
    virtual bool calibrateServo(ServoID id) = 0;

    // System control
    virtual void update() = 0;  // Called in main loop
    virtual bool isSystemHealthy() const = 0;
    virtual void resetErrors() = 0;
};

/**
 * @brief Abstract interface for servo observers
 * 
 * Observers are notified of servo state changes and can be used for
 * telemetry, logging, or external communication.
 */
class IServoObserver {
public:
    virtual ~IServoObserver() = default;

    // Servo state change notifications
    virtual void onServoPositionChanged(ServoID id, uint16_t oldPosition, uint16_t newPosition) = 0;
    virtual void onServoStatusChanged(ServoID id, ServoStatus oldStatus, ServoStatus newStatus) = 0;
    virtual void onServoError(ServoID id, ServoError error) = 0;
    virtual void onServoMovementStarted(ServoID id, uint16_t targetPosition) = 0;
    virtual void onServoMovementCompleted(ServoID id, uint16_t finalPosition) = 0;

    // System-wide notifications
            virtual void onEmergencyStop() = 0;
        virtual void onSystemError(const char* errorMessage) = 0;
        virtual void onTelemetryUpdate(const TelemetryArray& telemetry) = 0;
};

/**
 * @brief Factory interface for creating servo controllers
 * 
 * This factory pattern allows for different servo controller implementations
 * to be created based on configuration or runtime requirements.
 */
class IServoControllerFactory {
public:
    virtual ~IServoControllerFactory() = default;
    
    virtual std::unique_ptr<IServoController> createController(
        const std::span<const ServoConfig>& configs) = 0;
};

} // namespace DOF

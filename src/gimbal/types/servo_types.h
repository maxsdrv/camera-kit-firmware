#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <concepts>
#include <type_traits>

namespace DOF {

// Forward declarations
class IServoController;
class IServoObserver;
class ServoCommand;

// Servo identification
enum class ServoID : uint8_t {
    BASE_ROTATION = 0,      // Base rotation (Yaw)
    SHOULDER_PITCH = 1,     // Shoulder pitch
    SHOULDER_ROLL = 2,      // Shoulder roll
    ELBOW_PITCH = 3,        // Elbow pitch
    WRIST_PITCH = 4,        // Wrist pitch
    WRIST_ROLL = 5,         // Wrist roll (Camera mount)
    COUNT = 6
};

// Servo movement constraints
struct ServoConstraints {
    uint16_t min_angle;     // Minimum angle in degrees (0-180)
    uint16_t max_angle;     // Maximum angle in degrees (0-180)
    uint8_t min_speed;      // Minimum speed (1-255)
    uint8_t max_speed;      // Maximum speed (1-255)
    uint16_t acceleration;  // Acceleration limit (degrees/sec²)
    uint16_t deceleration;  // Deceleration limit (degrees/sec²)
};

// Servo status
enum class ServoStatus : uint8_t {
    IDLE = 0,
    MOVING = 1,
    ERROR = 2,
    CALIBRATING = 3,
    EMERGENCY_STOP = 4
};

// Servo error codes
enum class ServoError : uint8_t {
    NONE = 0,
    POSITION_LIMIT_EXCEEDED = 1,
    TEMPERATURE_HIGH = 2,
    OVERLOAD = 3,
    COMMUNICATION_ERROR = 4,
    TIMEOUT = 5
};

// Servo telemetry data
struct ServoTelemetry {
    ServoID id;
    uint16_t current_position;      // Current angle in degrees
    uint16_t target_position;       // Target angle in degrees
    uint8_t current_speed;          // Current speed (1-255)
    ServoStatus status;
    ServoError error;
    uint8_t temperature;            // Temperature in °C
    uint8_t load_percentage;        // Load percentage (0-100)
    uint32_t timestamp;             // System timestamp
    uint16_t voltage_mv;            // Voltage in millivolts
};

// Servo command types
enum class CommandType : uint8_t {
    MOVE_TO_POSITION = 0,
    SET_SPEED = 1,
    STOP = 2,
    HOME = 3,
    CALIBRATE = 4,
    EMERGENCY_STOP = 5,
    SET_CONSTRAINTS = 6
};

// Servo movement command
struct ServoCommand {
    CommandType type;
    ServoID servo_id;
    uint16_t target_position;       // Target angle in degrees
    uint8_t speed;                  // Movement speed (1-255)
    uint16_t acceleration;          // Acceleration (degrees/sec²)
    uint16_t deceleration;          // Deceleration (degrees/sec²)
    uint32_t timeout_ms;            // Movement timeout
};

// Servo configuration
struct ServoConfig {
    ServoID id;
    uint8_t pwm_channel;            // PWM channel number
    uint16_t pwm_frequency_hz;      // PWM frequency (typically 50Hz)
    uint16_t min_pulse_width_us;    // Minimum pulse width in microseconds
    uint16_t max_pulse_width_us;    // Maximum pulse width in microseconds
    ServoConstraints constraints;    // Movement constraints
    bool inverted;                   // Inverted movement direction
    uint16_t home_position;         // Home position in degrees
};

// Type aliases for convenience
using ServoArray = std::array<ServoID, static_cast<std::size_t>(ServoID::COUNT)>;
using TelemetryArray = std::array<ServoTelemetry, static_cast<std::size_t>(ServoID::COUNT)>;

// C++20 concepts
template<typename T>
concept ServoController = std::derived_from<T, IServoController>;

template<typename T>
concept ServoObserver = std::derived_from<T, IServoObserver>;

// Utility functions
constexpr ServoArray getAllServoIDs() {
    return {ServoID::BASE_ROTATION, ServoID::SHOULDER_PITCH, ServoID::SHOULDER_ROLL,
            ServoID::ELBOW_PITCH, ServoID::WRIST_PITCH, ServoID::WRIST_ROLL};
}

constexpr bool isValidServoID(ServoID id) {
    return static_cast<uint8_t>(id) < static_cast<uint8_t>(ServoID::COUNT);
}

constexpr uint8_t servoIDToIndex(ServoID id) {
    return static_cast<uint8_t>(id);
}

constexpr ServoID indexToServoID(uint8_t index) {
    return static_cast<ServoID>(index);
}

} // namespace DOF

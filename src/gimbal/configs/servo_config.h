#pragma once

#include "../types/servo_types.h"
#include <array>

namespace DOF {

/**
 * @brief Default servo configurations for 6-DOF robotic arm
 * 
 * These configurations are optimized for MG996R servos and typical
 * robotic arm kinematics. Each servo has specific constraints based
 * on mechanical limitations and safety considerations.
 */

// Default servo constraints for MG996R servos
inline constexpr ServoConstraints DEFAULT_SERVO_CONSTRAINTS = {
    .min_angle = 0,        // Minimum angle in degrees
    .max_angle = 180,      // Maximum angle in degrees
    .min_speed = 1,        // Minimum speed (1-255)
    .max_speed = 255,      // Maximum speed (1-255)
    .acceleration = 90,    // Acceleration limit (degrees/sec²)
    .deceleration = 90     // Deceleration limit (degrees/sec²)
};

// Base rotation servo (Yaw) - typically has full 180° range
inline constexpr ServoConstraints BASE_ROTATION_CONSTRAINTS = {
    .min_angle = 0,
    .max_angle = 180,
    .min_speed = 1,
    .max_speed = 200,      // Slightly slower for stability
    .acceleration = 60,    // Lower acceleration for base stability
    .deceleration = 60
};

// Shoulder pitch servo - limited range for mechanical safety
inline constexpr ServoConstraints SHOULDER_PITCH_CONSTRAINTS = {
    .min_angle = 30,       // Prevent arm from hitting base
    .max_angle = 150,      // Prevent over-extension
    .min_speed = 1,
    .max_speed = 180,      // Moderate speed for safety
    .acceleration = 75,
    .deceleration = 75
};

// Shoulder roll servo - limited range for mechanical safety
inline constexpr ServoConstraints SHOULDER_ROLL_CONSTRAINTS = {
    .min_angle = 45,       // Prevent arm from hitting base
    .max_angle = 135,      // Prevent over-extension
    .min_speed = 1,
    .max_speed = 180,
    .acceleration = 75,
    .deceleration = 75
};

// Elbow pitch servo - moderate range
inline constexpr ServoConstraints ELBOW_PITCH_CONSTRAINTS = {
    .min_angle = 20,       // Prevent elbow from hitting shoulder
    .max_angle = 160,      // Prevent over-extension
    .min_speed = 1,
    .max_speed = 200,
    .acceleration = 90,
    .deceleration = 90
};

// Wrist pitch servo - moderate range
inline constexpr ServoConstraints WRIST_PITCH_CONSTRAINTS = {
    .min_angle = 30,       // Prevent wrist from hitting arm
    .max_angle = 150,      // Prevent over-extension
    .min_speed = 1,
    .max_speed = 200,
    .acceleration = 90,
    .deceleration = 90
};

// Wrist roll servo (Camera mount) - full range for camera orientation
inline constexpr ServoConstraints WRIST_ROLL_CONSTRAINTS = {
    .min_angle = 0,        // Full rotation for camera
    .max_angle = 180,
    .min_speed = 1,
    .max_speed = 255,      // Full speed for responsive camera control
    .acceleration = 120,   // Higher acceleration for camera responsiveness
    .deceleration = 120
};

// Default servo configurations
inline constexpr std::array<ServoConfig, static_cast<size_t>(ServoID::COUNT)> DEFAULT_SERVO_CONFIGS = {{
    // Base rotation (Yaw)
    {
        .id = ServoID::BASE_ROTATION,
        .pwm_channel = 0,              // PCA9685 channel 0
        .pwm_frequency_hz = 50,        // 50Hz for standard servos
        .min_pulse_width_us = 500,     // 0.5ms pulse width
        .max_pulse_width_us = 2500,    // 2.5ms pulse width
        .constraints = BASE_ROTATION_CONSTRAINTS,
        .inverted = false,             // Normal direction
        .home_position = 90            // Center position
    },
    
    // Shoulder pitch
    {
        .id = ServoID::SHOULDER_PITCH,
        .pwm_channel = 1,              // PCA9685 channel 1
        .pwm_frequency_hz = 50,
        .min_pulse_width_us = 500,
        .max_pulse_width_us = 2500,
        .constraints = SHOULDER_PITCH_CONSTRAINTS,
        .inverted = false,
        .home_position = 90            // Center position
    },
    
    // Shoulder roll
    {
        .id = ServoID::SHOULDER_ROLL,
        .pwm_channel = 2,              // PCA9685 channel 2
        .pwm_frequency_hz = 50,
        .min_pulse_width_us = 500,
        .max_pulse_width_us = 2500,
        .constraints = SHOULDER_ROLL_CONSTRAINTS,
        .inverted = false,
        .home_position = 90            // Center position
    },
    
    // Elbow pitch
    {
        .id = ServoID::ELBOW_PITCH,
        .pwm_channel = 3,              // PCA9685 channel 3
        .pwm_frequency_hz = 50,
        .min_pulse_width_us = 500,
        .max_pulse_width_us = 2500,
        .constraints = ELBOW_PITCH_CONSTRAINTS,
        .inverted = false,
        .home_position = 90            // Center position
    },
    
    // Wrist pitch
    {
        .id = ServoID::WRIST_PITCH,
        .pwm_channel = 4,              // PCA9685 channel 4
        .pwm_frequency_hz = 50,
        .min_pulse_width_us = 500,
        .max_pulse_width_us = 2500,
        .constraints = WRIST_PITCH_CONSTRAINTS,
        .inverted = false,
        .home_position = 90            // Center position
    },
    
    // Wrist roll (Camera mount)
    {
        .id = ServoID::WRIST_ROLL,
        .pwm_channel = 5,              // PCA9685 channel 5
        .pwm_frequency_hz = 50,
        .min_pulse_width_us = 500,
        .max_pulse_width_us = 2500,
        .constraints = WRIST_ROLL_CONSTRAINTS,
        .inverted = false,
        .home_position = 90            // Center position
    }
}};

// Safety configurations
inline constexpr uint16_t EMERGENCY_STOP_TIMEOUT_MS = 1000;    // 1 second timeout
inline constexpr uint16_t MAX_SIMULTANEOUS_MOVEMENTS = 3;      // Limit concurrent movements
inline constexpr uint16_t TELEMETRY_UPDATE_RATE_HZ = 100;      // 100Hz telemetry
inline constexpr uint16_t COMMAND_TIMEOUT_MS = 5000;           // 5 second command timeout

// PCA9685 configuration
inline constexpr uint8_t PCA9685_DEFAULT_ADDRESS = 0x40;       // Default I2C address
inline constexpr uint16_t PCA9685_DEFAULT_FREQUENCY = 50;      // 50Hz PWM frequency
inline constexpr uint8_t PCA9685_I2C_TIMEOUT_MS = 100;         // I2C timeout

// Servo movement presets
namespace ServoPresets {
    
    // Home position for all servos
    inline constexpr std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> HOME_POSITIONS = {
        90,  // Base rotation
        90,  // Shoulder pitch
        90,  // Shoulder roll
        90,  // Elbow pitch
        90,  // Wrist pitch
        90   // Wrist roll
    };
    
    // Rest position (arm folded)
    inline constexpr std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> REST_POSITIONS = {
        90,  // Base rotation
        30,  // Shoulder pitch (folded)
        90,  // Shoulder roll
        160, // Elbow pitch (folded)
        30,  // Wrist pitch (folded)
        90   // Wrist roll
    };
    
    // Camera pointing forward
    inline constexpr std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> CAMERA_FORWARD_POSITIONS = {
        90,  // Base rotation
        90,  // Shoulder pitch
        90,  // Shoulder roll
        90,  // Elbow pitch
        90,  // Wrist pitch
        90   // Wrist roll
    };
    
    // Camera pointing down
    inline constexpr std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> CAMERA_DOWN_POSITIONS = {
        90,  // Base rotation
        90,  // Shoulder pitch
        90,  // Shoulder roll
        90,  // Elbow pitch
        150, // Wrist pitch (pointing down)
        90   // Wrist roll
    };
}

// Utility functions
inline constexpr ServoConfig getServoConfig(ServoID id) {
    if (isValidServoID(id)) {
        return DEFAULT_SERVO_CONFIGS[servoIDToIndex(id)];
    }
    return ServoConfig{};
}

inline constexpr ServoConstraints getServoConstraints(ServoID id) {
    if (isValidServoID(id)) {
        return DEFAULT_SERVO_CONFIGS[servoIDToIndex(id)].constraints;
    }
    return DEFAULT_SERVO_CONSTRAINTS;
}

inline constexpr uint8_t getServoPWMChannel(ServoID id) {
    if (isValidServoID(id)) {
        return DEFAULT_SERVO_CONFIGS[servoIDToIndex(id)].pwm_channel;
    }
    return 0;
}

} // namespace DOF

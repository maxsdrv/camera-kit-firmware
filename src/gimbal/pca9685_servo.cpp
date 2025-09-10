/**
 * @file pca9685_servo.cpp
 * @brief Simple PCA9685 Servo Controller Implementation
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "pca9685_servo.h"

// Global I2C handle
static I2C_HandleTypeDef* g_hi2c = nullptr;

// Current servo positions
uint16_t servo_positions[SERVO_COUNT] = {
    SERVO_CENTER_PULSE, SERVO_CENTER_PULSE, SERVO_CENTER_PULSE,
    SERVO_CENTER_PULSE, SERVO_CENTER_PULSE, SERVO_CENTER_PULSE
};

/**
 * @brief Initialize PCA9685 servo controller
 * @param hi2c I2C handle
 */
void PCA9685_Init(I2C_HandleTypeDef* hi2c)
{
    g_hi2c = hi2c;
    
    uint8_t data[2];
    
    // Reset PCA9685
    data[0] = PCA9685_MODE1;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 2, 100);
    HAL_Delay(10);
    
    // Set PWM frequency to 50Hz for servos
    // Put PCA9685 to sleep to set prescaler
    data[0] = PCA9685_MODE1;
    data[1] = 0x10; // Sleep mode
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 2, 100);
    
    // Set prescaler for 50Hz (formula: prescaler = 25MHz / (4096 * frequency) - 1)
    data[0] = PCA9685_PRESCALE;
    data[1] = 0x79; // 50Hz prescaler value (~121 decimal)
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 2, 100);
    
    // Wake up PCA9685
    data[0] = PCA9685_MODE1;
    data[1] = 0x00; // Normal mode
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 2, 100);
    HAL_Delay(5);
    
    // Enable auto-increment mode for multi-channel writes
    data[0] = PCA9685_MODE1;
    data[1] = 0xA0; // Auto-increment + restart
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 2, 100);
    
    // Set all servos to center position
    PCA9685_HomeAllServos();
}

/**
 * @brief Set servo position
 * @param servo_id Servo ID (0-5)
 * @param pulse_width Pulse width in timer ticks (150-600)
 */
void PCA9685_SetServo(uint8_t servo_id, uint16_t pulse_width)
{
    if (servo_id >= SERVO_COUNT || g_hi2c == nullptr) {
        return;
    }
    
    // Clamp pulse width to safe range
    if (pulse_width < SERVO_MIN_PULSE) pulse_width = SERVO_MIN_PULSE;
    if (pulse_width > SERVO_MAX_PULSE) pulse_width = SERVO_MAX_PULSE;
    
    // Calculate register address for this servo channel
    uint8_t reg = PCA9685_LED0_ON_L + (servo_id * 4);
    
    uint8_t data[5];
    data[0] = reg;           // Register address
    data[1] = 0x00;          // ON_L (start at 0)
    data[2] = 0x00;          // ON_H (start at 0)
    data[3] = pulse_width & 0xFF;        // OFF_L (low byte)
    data[4] = (pulse_width >> 8) & 0xFF; // OFF_H (high byte)
    
    HAL_I2C_Master_Transmit(g_hi2c, PCA9685_ADDRESS << 1, data, 5, 100);
    
    // Update stored position
    servo_positions[servo_id] = pulse_width;
}

/**
 * @brief Set all servos to the same position
 * @param pulse_width Pulse width in timer ticks (150-600)
 */
void PCA9685_SetAllServos(uint16_t pulse_width)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        PCA9685_SetServo(i, pulse_width);
    }
}

/**
 * @brief Move all servos to center (home) position
 */
void PCA9685_HomeAllServos(void)
{
    PCA9685_SetAllServos(SERVO_CENTER_PULSE);
}
/**
 * @file pca9685_servo.h  
 * @brief Simple PCA9685 Servo Controller for Firmware
 * @version 1.0.0
 * @date 2025-01-04
 */

#pragma once

#include "board/Inc/stm32_hal_config.h"

// PCA9685 I2C Address
#define PCA9685_ADDRESS     0x40

// PCA9685 Registers
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

// Servo constants
#define SERVO_COUNT         6
#define SERVO_MIN_PULSE     150   // ~1ms pulse width for 50Hz PWM
#define SERVO_MAX_PULSE     600   // ~2ms pulse width for 50Hz PWM
#define SERVO_CENTER_PULSE  375   // ~1.5ms center position

// Simple servo controller functions
void PCA9685_Init(I2C_HandleTypeDef* hi2c);
void PCA9685_SetServo(uint8_t servo_id, uint16_t pulse_width);
void PCA9685_SetAllServos(uint16_t pulse_width);
void PCA9685_HomeAllServos(void);

// Current servo positions
extern uint16_t servo_positions[SERVO_COUNT];
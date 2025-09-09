#include "pca9685_servo_controller.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace DOF {

// STM32 HAL I2C handle (extern from main)
// I2C handle must be provided during construction

PCA9685ServoController::PCA9685ServoController(const std::span<const ServoConfig>& configs,
                                               I2C_HandleTypeDef* i2c_handle,
                                               uint8_t pca9685_address)
    : initialized_(false)
    , emergency_stop_active_(false)
    , system_start_time_(0)
    , i2c_handle_(i2c_handle)
    , pca9685_address_(pca9685_address)
    , pwm_frequency_hz_(DEFAULT_PWM_FREQUENCY_HZ)
    , sleeping_(false)
{
    // Validate I2C handle
    if (!i2c_handle) {
        // This should not happen in normal operation
        return;
    }
    
    // Initialize servo states with default configurations
    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        auto& state = servo_states_[i];
        state.config = configs[i];
        state.current_position = state.config.home_position;
        state.target_position = state.config.home_position;
        state.current_speed = 128;
        state.target_speed = 128;
        state.acceleration = DEFAULT_ACCELERATION;
        state.deceleration = DEFAULT_DECELERATION;
        state.status = ServoStatus::IDLE;
        state.error = ServoError::NONE;
        state.movement_start_time = 0;
        state.last_update_time = 0;
        state.is_moving = false;
        state.is_homed = false;
        state.current_pwm_value = 0;
        
        // Initialize telemetry
        state.telemetry.id = static_cast<ServoID>(i);
        state.telemetry.current_position = state.current_position;
        state.telemetry.target_position = state.target_position;
        state.telemetry.current_speed = state.current_speed;
        state.telemetry.status = state.status;
        state.telemetry.error = state.error;
        state.telemetry.temperature = 25;  // Default room temperature
        state.telemetry.load_percentage = 0;
        state.telemetry.timestamp = 0;
        state.telemetry.voltage_mv = 6000;  // Default 6V
    }
}

PCA9685ServoController::~PCA9685ServoController() {
    shutdown();
}

bool PCA9685ServoController::initialize() {
    if (initialized_) {
        return true;
    }

    // Initialize PCA9685
    if (!initializePCA9685()) {
        return false;
    }

    // Initialize GPIO pins (if needed for additional control)
    if (!initializeGPIO()) {
        return false;
    }

    // Set all servos to home position
    for (auto& state : servo_states_) {
        uint16_t pwm_value = angleToPWMValue(state.config, state.current_position);
        if (!setPCA9685Channel(state.config.pwm_channel, pwm_value)) {
            return false;
        }
        state.current_pwm_value = pwm_value;
    }

    initialized_ = true;
    system_start_time_ = HAL_GetTick();
    
    return true;
}

void PCA9685ServoController::shutdown() {
    if (!initialized_) {
        return;
    }

    // Stop all servos
    emergencyStop();
    
    // Put PCA9685 to sleep
    sleep(true);
    
    initialized_ = false;
}

bool PCA9685ServoController::isInitialized() const {
    return initialized_;
}

bool PCA9685ServoController::moveServo(ServoID id, uint16_t position, uint8_t speed) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    
    // Validate position
    if (!validatePosition(id, position)) {
        notifyServoError(id, ServoError::POSITION_LIMIT_EXCEEDED);
        return false;
    }

    // Update state
    uint16_t old_position = state.current_position;
    state.target_position = position;
    state.target_speed = std::clamp(speed, state.config.constraints.min_speed, 
                                   state.config.constraints.max_speed);
    state.status = ServoStatus::MOVING;
    state.is_moving = true;
    state.movement_start_time = HAL_GetTick();
    state.last_update_time = HAL_GetTick();

    // Notify observers
    notifyServoPositionChanged(id, old_position, position);
    notifyServoMovementStarted(id, position);

    return true;
}

bool PCA9685ServoController::stopServo(ServoID id) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    
    if (state.is_moving) {
        state.status = ServoStatus::IDLE;
        state.is_moving = false;
        state.target_position = state.current_position;
        
        notifyServoStatusChanged(id, ServoStatus::MOVING, ServoStatus::IDLE);
    }

    return true;
}

bool PCA9685ServoController::homeServo(ServoID id) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    return moveServo(id, servo_states_[servoIDToIndex(id)].config.home_position, 64);
}

bool PCA9685ServoController::setServoSpeed(ServoID id, uint8_t speed) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    uint8_t old_speed = state.target_speed;
    state.target_speed = std::clamp(speed, state.config.constraints.min_speed, 
                                   state.config.constraints.max_speed);

    return old_speed != state.target_speed;
}

bool PCA9685ServoController::moveMultipleServos(std::span<const ServoCommand> commands) {
    if (!initialized_) {
        return false;
    }

    bool success = true;
    for (const auto& command : commands) {
        if (!moveServo(command.servo_id, command.target_position, command.speed)) {
            success = false;
        }
    }

    return success;
}

bool PCA9685ServoController::stopAllServos() {
    if (!initialized_) {
        return false;
    }

    for (auto& state : servo_states_) {
        if (state.is_moving) {
            state.status = ServoStatus::IDLE;
            state.is_moving = false;
            state.target_position = state.current_position;
        }
    }

    return true;
}

bool PCA9685ServoController::homeAllServos() {
    if (!initialized_) {
        return false;
    }

    bool success = true;
    for (auto& state : servo_states_) {
        if (!moveServo(state.config.id, state.config.home_position, 64)) {
            success = false;
        }
    }

    return success;
}

bool PCA9685ServoController::emergencyStop() {
    if (!initialized_) {
        return false;
    }

    emergency_stop_active_ = true;
    
    // Stop all servos immediately
    for (auto& state : servo_states_) {
        state.status = ServoStatus::EMERGENCY_STOP;
        state.is_moving = false;
        state.error = ServoError::NONE;
    }

    // Notify observers
    for (auto& observer : observers_) {
        observer->onEmergencyStop();
    }

    return true;
}

ServoTelemetry PCA9685ServoController::getServoTelemetry(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoTelemetry{};
    }
    return servo_states_[servoIDToIndex(id)].telemetry;
}

TelemetryArray PCA9685ServoController::getAllServoTelemetry() const {
    TelemetryArray telemetry;
    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        telemetry[i] = servo_states_[i].telemetry;
    }
    return telemetry;
}

ServoStatus PCA9685ServoController::getServoStatus(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoStatus::ERROR;
    }
    return servo_states_[servoIDToIndex(id)].status;
}

bool PCA9685ServoController::isServoMoving(ServoID id) const {
    if (!isValidServoID(id)) {
        return false;
    }
    return servo_states_[servoIDToIndex(id)].is_moving;
}

bool PCA9685ServoController::setServoConstraints(ServoID id, const ServoConstraints& constraints) {
    if (!isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    state.config.constraints = constraints;
    
    return true;
}

ServoConstraints PCA9685ServoController::getServoConstraints(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoConstraints{};
    }
    return servo_states_[servoIDToIndex(id)].config.constraints;
}

bool PCA9685ServoController::calibrateServo(ServoID id) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    state.status = ServoStatus::CALIBRATING;
    
    // Simple calibration: move to center position
    uint16_t center_position = (state.config.constraints.min_angle + 
                               state.config.constraints.max_angle) / 2;
    
    if (moveServo(id, center_position, 32)) {  // Slow speed for calibration
        state.is_homed = true;
        state.status = ServoStatus::IDLE;
        return true;
    }

    return false;
}

void PCA9685ServoController::update() {
    if (!initialized_ || emergency_stop_active_) {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    
    // Update each servo state
    for (auto& state : servo_states_) {
        updateServoState(state, current_time);
    }

    // Update telemetry timestamps
    for (auto& state : servo_states_) {
        state.telemetry.timestamp = current_time - system_start_time_;
    }

    // Notify observers of telemetry update
    notifyObservers();
}

bool PCA9685ServoController::isSystemHealthy() const {
    if (!initialized_) {
        return false;
    }

    for (const auto& state : servo_states_) {
        if (state.error != ServoError::NONE || state.status == ServoStatus::ERROR) {
            return false;
        }
    }

    return true;
}

void PCA9685ServoController::resetErrors() {
    for (auto& state : servo_states_) {
        state.error = ServoError::NONE;
        if (state.status == ServoStatus::ERROR) {
            state.status = ServoStatus::IDLE;
        }
    }
}

void PCA9685ServoController::addObserver(std::shared_ptr<IServoObserver> observer) {
    if (observer) {
        observers_.push_back(observer);
    }
}

void PCA9685ServoController::removeObserver(std::shared_ptr<IServoObserver> observer) {
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
}

// PCA9685 specific methods
bool PCA9685ServoController::setPWMFrequency(uint16_t frequency_hz) {
    if (!initialized_) {
        return false;
    }

    // Calculate prescaler value
    // Formula: prescaler = (clock_freq / (4096 * frequency)) - 1
    // PCA9685 clock is typically 25MHz
    uint32_t clock_freq = 25000000;  // 25MHz
    uint8_t prescaler = static_cast<uint8_t>((clock_freq / (4096 * frequency_hz)) - 1);
    
    // Put device to sleep to change prescaler
    if (!sleep(true)) {
        return false;
    }
    
    // Write prescaler value
    if (!writePCA9685Register(PCA9685_PRESCALE, prescaler)) {
        return false;
    }
    
    // Wake up device
    if (!sleep(false)) {
        return false;
    }
    
    pwm_frequency_hz_ = frequency_hz;
    return true;
}

uint16_t PCA9685ServoController::getPWMFrequency() const {
    return pwm_frequency_hz_;
}

bool PCA9685ServoController::sleep(bool enable) {
    uint8_t mode1 = readPCA9685Register(PCA9685_MODE1);
    
    if (enable) {
        mode1 |= MODE1_SLEEP;
    } else {
        mode1 &= ~MODE1_SLEEP;
    }
    
    if (!writePCA9685Register(PCA9685_MODE1, mode1)) {
        return false;
    }
    
    sleeping_ = enable;
    
    // If waking up, wait for oscillator to stabilize
    if (!enable) {
        HAL_Delay(1);  // Wait 1ms for oscillator to stabilize
    }
    
    return true;
}

bool PCA9685ServoController::isSleeping() const {
    return sleeping_;
}

bool PCA9685ServoController::resetPCA9685() {
    // Software reset by setting RESTART bit
    uint8_t mode1 = readPCA9685Register(PCA9685_MODE1);
    mode1 |= MODE1_RESTART;
    
    if (!writePCA9685Register(PCA9685_MODE1, mode1)) {
        return false;
    }
    
    // Wait for reset to complete
    HAL_Delay(10);
    
    return true;
}

// Private methods implementation
bool PCA9685ServoController::initializePCA9685() {
    // Reset device
    if (!resetPCA9685()) {
        return false;
    }
    
    // Configure MODE1 register
    uint8_t mode1 = MODE1_AUTOINC;  // Enable auto-increment
    if (!writePCA9685Register(PCA9685_MODE1, mode1)) {
        return false;
    }
    
    // Set PWM frequency
    if (!setPWMFrequency(DEFAULT_PWM_FREQUENCY_HZ)) {
        return false;
    }
    
    // Clear all PWM channels
    std::array<uint8_t, 64> clear_data{};  // 16 channels * 4 bytes per channel
    if (!writePCA9685Registers(PCA9685_LED0_ON_L, clear_data)) {
        return false;
    }
    
    return true;
}

bool PCA9685ServoController::initializeGPIO() {
    // GPIO should be initialized by STM32CubeMX
    // This is just a placeholder for any additional GPIO setup
    return true;
}

void PCA9685ServoController::updateServoState(ServoState& servo, uint32_t current_time) {
    if (!servo.is_moving || servo.status != ServoStatus::MOVING) {
        return;
    }

    uint32_t elapsed_time = current_time - servo.last_update_time;
    if (elapsed_time < UPDATE_PERIOD_MS) {
        return;
    }

    servo.last_update_time = current_time;

    // Calculate movement step based on speed and acceleration
    float speed_deg_per_sec = (servo.target_speed / 255.0f) * 180.0f;  // Max 180°/sec
    float step_size = (speed_deg_per_sec * elapsed_time) / 1000.0f;

    // Apply acceleration/deceleration
    uint32_t movement_time = current_time - servo.movement_start_time;
    float acceleration_factor = 1.0f;
    
    if (movement_time < 1000) {  // First second: acceleration
        acceleration_factor = std::min(1.0f, movement_time / 1000.0f);
    } else {
        // Check if we need to start decelerating
        float distance_to_target = std::abs(servo.target_position - servo.current_position);
        float decel_distance = (speed_deg_per_sec * speed_deg_per_sec) / (2.0f * servo.deceleration);
        
        if (distance_to_target <= decel_distance) {
            acceleration_factor = std::max(0.1f, distance_to_target / decel_distance);
        }
    }

    step_size *= acceleration_factor;

    // Move towards target
    if (servo.current_position < servo.target_position) {
        servo.current_position = std::min(servo.target_position, 
                                        static_cast<uint16_t>(servo.current_position + static_cast<uint16_t>(step_size)));
    } else if (servo.current_position > servo.target_position) {
        servo.current_position = std::max(servo.target_position, 
                                        static_cast<uint16_t>(servo.current_position - static_cast<uint16_t>(step_size)));
    }

    // Check if movement is complete
    if (std::abs(servo.current_position - servo.target_position) <= 1) {
        servo.current_position = servo.target_position;
        servo.status = ServoStatus::IDLE;
        servo.is_moving = false;
        
        notifyServoMovementCompleted(servo.config.id, servo.current_position);
        notifyServoStatusChanged(servo.config.id, ServoStatus::MOVING, ServoStatus::IDLE);
    }

    // Update PWM output if position changed
    uint16_t new_pwm_value = angleToPWMValue(servo.config, servo.current_position);
    if (new_pwm_value != servo.current_pwm_value) {
        if (setPCA9685Channel(servo.config.pwm_channel, new_pwm_value)) {
            servo.current_pwm_value = new_pwm_value;
        }
    }

    // Update telemetry
    servo.telemetry.current_position = servo.current_position;
    servo.telemetry.target_position = servo.target_position;
    servo.telemetry.current_speed = servo.target_speed;
    servo.telemetry.status = servo.status;
}

bool PCA9685ServoController::validatePosition(ServoID id, uint16_t position) const {
    if (!isValidServoID(id)) {
        return false;
    }

    const auto& state = servo_states_[servoIDToIndex(id)];
    return position >= state.config.constraints.min_angle && 
           position <= state.config.constraints.max_angle;
}

uint16_t PCA9685ServoController::angleToPWMValue(const ServoConfig& config, uint16_t angle) const {
    // Convert angle (0-180) to PWM value (0-4095)
    float normalized_angle = static_cast<float>(angle) / 180.0f;
    
    // Convert pulse width to PWM value
    float pulse_width_us = config.min_pulse_width_us + 
                           normalized_angle * (config.max_pulse_width_us - config.min_pulse_width_us);
    
    // Convert to PWM value (0-4095)
    // PWM period = 20ms = 20000us
    // PWM value = (pulse_width_us / 20000) * 4096
    uint16_t pwm_value = static_cast<uint16_t>((pulse_width_us / 20000.0f) * PWM_RESOLUTION);
    
    return std::clamp(pwm_value, static_cast<uint16_t>(0), static_cast<uint16_t>(PWM_RESOLUTION - 1));
}

bool PCA9685ServoController::setPCA9685PWM(uint8_t channel, uint16_t on_value, uint16_t off_value) {
    if (channel >= 16) {
        return false;
    }

    // Calculate register addresses for the channel
    uint8_t base_reg = PCA9685_LED0_ON_L + (channel * 4);
    
    // Prepare data: [ON_L, ON_H, OFF_L, OFF_H]
    std::array<uint8_t, 4> data = {
        static_cast<uint8_t>(on_value & 0xFF),           // ON_L
        static_cast<uint8_t>((on_value >> 8) & 0x0F),   // ON_H (only 4 bits)
        static_cast<uint8_t>(off_value & 0xFF),          // OFF_L
        static_cast<uint8_t>((off_value >> 8) & 0x0F)   // OFF_H (only 4 bits)
    };
    
    return writePCA9685Registers(base_reg, data);
}

bool PCA9685ServoController::setPCA9685Channel(uint8_t channel, uint16_t pwm_value) {
    // For servo control, we want the signal to start at 0 and end at pwm_value
    return setPCA9685PWM(channel, 0, pwm_value);
}

bool PCA9685ServoController::writePCA9685Register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    
    if (HAL_I2C_Master_Transmit(reinterpret_cast<I2C_HandleTypeDef*>(i2c_handle_), pca9685_address_ << 1, data, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    
    return true;
}

uint8_t PCA9685ServoController::readPCA9685Register(uint8_t reg) {
    uint8_t value = 0;
    
    // Send register address
    if (HAL_I2C_Master_Transmit(reinterpret_cast<I2C_HandleTypeDef*>(i2c_handle_), pca9685_address_ << 1, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    
    // Read register value
    if (HAL_I2C_Master_Receive(reinterpret_cast<I2C_HandleTypeDef*>(i2c_handle_), (pca9685_address_ << 1) | 1, &value, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    
    return value;
}

bool PCA9685ServoController::writePCA9685Registers(uint8_t start_reg, const std::span<const uint8_t>& data) {
    // Prepare buffer with register address and data
    std::vector<uint8_t> buffer;
    buffer.reserve(data.size() + 1);
    buffer.push_back(start_reg);
    buffer.insert(buffer.end(), data.begin(), data.end());
    
    if (HAL_I2C_Master_Transmit(reinterpret_cast<I2C_HandleTypeDef*>(i2c_handle_), pca9685_address_ << 1, buffer.data(), buffer.size(), I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    
    return true;
}

void PCA9685ServoController::notifyObservers() {
    TelemetryArray telemetry = getAllServoTelemetry();
    for (auto& observer : observers_) {
        observer->onTelemetryUpdate(telemetry);
    }
}

void PCA9685ServoController::notifyServoPositionChanged(ServoID id, uint16_t oldPos, uint16_t newPos) {
    for (auto& observer : observers_) {
        observer->onServoPositionChanged(id, oldPos, newPos);
    }
}

void PCA9685ServoController::notifyServoStatusChanged(ServoID id, ServoStatus oldStatus, ServoStatus newStatus) {
    for (auto& observer : observers_) {
        observer->onServoStatusChanged(id, oldStatus, newStatus);
    }
}

void PCA9685ServoController::notifyServoError(ServoID id, ServoError error) {
    for (auto& observer : observers_) {
        observer->onServoError(id, error);
    }
}

void PCA9685ServoController::notifyServoMovementStarted(ServoID id, uint16_t targetPosition) {
    for (auto& observer : observers_) {
        if (observer) {
            observer->onServoMovementStarted(id, targetPosition);
        }
    }
}

void PCA9685ServoController::notifyServoMovementCompleted(ServoID id, uint16_t finalPosition) {
    for (auto& observer : observers_) {
        if (observer) {
            observer->onServoMovementCompleted(id, finalPosition);
        }
    }
}

// Factory implementation
PCA9685ServoControllerFactory::PCA9685ServoControllerFactory(I2C_HandleTypeDef* i2c_handle,
                                                           uint8_t pca9685_address)
    : i2c_handle_(i2c_handle)
    , pca9685_address_(pca9685_address)
{
}

std::unique_ptr<IServoController> PCA9685ServoControllerFactory::createController(
    const std::span<const ServoConfig>& configs) {
    return std::make_unique<PCA9685ServoController>(configs, i2c_handle_, pca9685_address_);
}

} // namespace DOF

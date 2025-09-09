#include "pwm_servo_controller.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_tim.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace DOF {

// STM32 HAL timer handle (extern from main)
extern TIM_HandleTypeDef htim1;  // Assuming TIM1 is used for PWM

PWMServoController::PWMServoController(const std::span<const ServoConfig>& configs)
    : initialized_(false)
    , emergency_stop_active_(false)
    , system_start_time_(0)
    , pwm_timer_(&htim1)
{
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

PWMServoController::~PWMServoController() {
    shutdown();
}

bool PWMServoController::initialize() {
    if (initialized_) {
        return true;
    }

    // Initialize PWM timer
    if (!initializePWM()) {
        return false;
    }

    // Initialize GPIO pins
    if (!initializeGPIO()) {
        return false;
    }

    // Set all servos to home position
    for (auto& state : servo_states_) {
        uint16_t pulse_width = angleToPulseWidth(state.config, state.current_position);
        setPWMPulseWidth(state.config.pwm_channel, pulse_width);
    }

    // Start PWM timer
    if (HAL_TIM_PWM_Start(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }

    initialized_ = true;
    system_start_time_ = HAL_GetTick();
    
    return true;
}

void PWMServoController::shutdown() {
    if (!initialized_) {
        return;
    }

    // Stop all servos
    emergencyStop();
    
    // Stop PWM timer
    HAL_TIM_PWM_Stop(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_1);
    
    initialized_ = false;
}

bool PWMServoController::isInitialized() const {
    return initialized_;
}

bool PWMServoController::moveServo(ServoID id, uint16_t position, uint8_t speed) {
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

bool PWMServoController::stopServo(ServoID id) {
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

bool PWMServoController::homeServo(ServoID id) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    return moveServo(id, servo_states_[servoIDToIndex(id)].config.home_position, 64);
}

bool PWMServoController::setServoSpeed(ServoID id, uint8_t speed) {
    if (!initialized_ || !isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    uint8_t old_speed = state.target_speed;
    state.target_speed = std::clamp(speed, state.config.constraints.min_speed, 
                                   state.config.constraints.max_speed);

    return old_speed != state.target_speed;
}

bool PWMServoController::moveMultipleServos(std::span<const ServoCommand> commands) {
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

bool PWMServoController::stopAllServos() {
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

bool PWMServoController::homeAllServos() {
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

bool PWMServoController::emergencyStop() {
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

ServoTelemetry PWMServoController::getServoTelemetry(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoTelemetry{};
    }
    return servo_states_[servoIDToIndex(id)].telemetry;
}

TelemetryArray PWMServoController::getAllServoTelemetry() const {
    TelemetryArray telemetry;
    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        telemetry[i] = servo_states_[i].telemetry;
    }
    return telemetry;
}

ServoStatus PWMServoController::getServoStatus(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoStatus::ERROR;
    }
    return servo_states_[servoIDToIndex(id)].status;
}

bool PWMServoController::isServoMoving(ServoID id) const {
    if (!isValidServoID(id)) {
        return false;
    }
    return servo_states_[servoIDToIndex(id)].is_moving;
}

bool PWMServoController::setServoConstraints(ServoID id, const ServoConstraints& constraints) {
    if (!isValidServoID(id)) {
        return false;
    }

    auto& state = servo_states_[servoIDToIndex(id)];
    state.config.constraints = constraints;
    
    return true;
}

ServoConstraints PWMServoController::getServoConstraints(ServoID id) const {
    if (!isValidServoID(id)) {
        return ServoConstraints{};
    }
    return servo_states_[servoIDToIndex(id)].config.constraints;
}

bool PWMServoController::calibrateServo(ServoID id) {
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

void PWMServoController::update() {
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

bool PWMServoController::isSystemHealthy() const {
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

void PWMServoController::resetErrors() {
    for (auto& state : servo_states_) {
        state.error = ServoError::NONE;
        if (state.status == ServoStatus::ERROR) {
            state.status = ServoStatus::IDLE;
        }
    }
}

void PWMServoController::addObserver(std::shared_ptr<IServoObserver> observer) {
    if (observer) {
        observers_.push_back(observer);
    }
}

void PWMServoController::removeObserver(std::shared_ptr<IServoObserver> observer) {
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
}

// Private methods implementation
bool PWMServoController::initializePWM() {
    // PWM timer should be initialized by STM32CubeMX
    if (!pwm_timer_) {
        return false;
    }

    // Configure PWM frequency for 50Hz (20ms period)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    uint32_t period = timer_clock / DEFAULT_PWM_FREQUENCY_HZ;
    
    __HAL_TIM_SET_AUTORELOAD(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), period - 1);
    
    return true;
}

bool PWMServoController::initializeGPIO() {
    // GPIO should be initialized by STM32CubeMX
    // This is just a placeholder for any additional GPIO setup
    return true;
}

void PWMServoController::updateServoState(ServoState& servo, uint32_t current_time) {
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

    // Update PWM output
    uint16_t pulse_width = angleToPulseWidth(servo.config, servo.current_position);
    setPWMPulseWidth(servo.config.pwm_channel, pulse_width);

    // Update telemetry
    servo.telemetry.current_position = servo.current_position;
    servo.telemetry.target_position = servo.target_position;
    servo.telemetry.current_speed = servo.target_speed;
    servo.telemetry.status = servo.status;
}

bool PWMServoController::validatePosition(ServoID id, uint16_t position) const {
    if (!isValidServoID(id)) {
        return false;
    }

    const auto& state = servo_states_[servoIDToIndex(id)];
    return position >= state.config.constraints.min_angle && 
           position <= state.config.constraints.max_angle;
}

uint16_t PWMServoController::angleToPulseWidth(const ServoConfig& config, uint16_t angle) const {
    // Convert angle (0-180) to pulse width (min_pulse to max_pulse)
    float normalized_angle = static_cast<float>(angle) / 180.0f;
    uint16_t pulse_width = config.min_pulse_width_us + 
                           static_cast<uint16_t>(normalized_angle * 
                           (config.max_pulse_width_us - config.min_pulse_width_us));
    
    return pulse_width;
}

void PWMServoController::setPWMPulseWidth(uint8_t channel, uint16_t pulse_width_us) {
    // Convert pulse width to timer compare value
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    uint32_t period = timer_clock / DEFAULT_PWM_FREQUENCY_HZ;
    uint32_t compare_value = (pulse_width_us * period) / 20000;  // 20000us = 20ms period
    
    // Set PWM compare value based on channel
    switch (channel) {
        case 1:
            __HAL_TIM_SET_COMPARE(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_1, compare_value);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_2, compare_value);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_3, compare_value);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(reinterpret_cast<TIM_HandleTypeDef*>(pwm_timer_), TIM_CHANNEL_4, compare_value);
            break;
        default:
            break;
    }
}

void PWMServoController::notifyObservers() {
    TelemetryArray telemetry = getAllServoTelemetry();
    for (auto& observer : observers_) {
        observer->onTelemetryUpdate(telemetry);
    }
}

void PWMServoController::notifyServoPositionChanged(ServoID id, uint16_t oldPos, uint16_t newPos) {
    for (auto& observer : observers_) {
        observer->onServoPositionChanged(id, oldPos, newPos);
    }
}

void PWMServoController::notifyServoStatusChanged(ServoID id, ServoStatus oldStatus, ServoStatus newStatus) {
    for (auto& observer : observers_) {
        observer->onServoStatusChanged(id, oldStatus, newStatus);
    }
}

void PWMServoController::notifyServoError(ServoID id, ServoError error) {
    for (auto& observer : observers_) {
        observer->onServoError(id, error);
    }
}

void PWMServoController::notifyServoMovementStarted(ServoID id, uint16_t targetPosition) {
    for (auto& observer : observers_) {
        if (observer) {
            observer->onServoMovementStarted(id, targetPosition);
        }
    }
}

void PWMServoController::notifyServoMovementCompleted(ServoID id, uint16_t finalPosition) {
    for (auto& observer : observers_) {
        if (observer) {
            observer->onServoMovementCompleted(id, finalPosition);
        }
    }
}

// Factory implementation
std::unique_ptr<IServoController> PWMServoControllerFactory::createController(
    const std::span<const ServoConfig>& configs) {
    return std::make_unique<PWMServoController>(configs);
}

} // namespace DOF

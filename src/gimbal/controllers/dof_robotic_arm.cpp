#include "dof_robotic_arm.h"
#include <algorithm>
#include <cstring>

namespace DOF {

DOFRoboticArm::DOFRoboticArm(I2C_HandleTypeDef* i2c_handle, uint8_t pca9685_address)
    : initialized_(false)
    , emergency_stop_active_(false)
    , last_health_check_(0)
    , system_start_time_(0)
    , system_healthy_(false)
    , active_movements_(0)
    , last_telemetry_update_(0)
    , i2c_handle_(i2c_handle)
    , pca9685_address_(pca9685_address)
{
    system_start_time_ = HAL_GetTick();
}

DOFRoboticArm::~DOFRoboticArm() {
    shutdown();
}

bool DOFRoboticArm::initialize() {
    if (initialized_) {
        return true;
    }

    // Create servo controller using factory
    PCA9685ServoControllerFactory factory(i2c_handle_, pca9685_address_);
    servo_controller_ = factory.createController(DEFAULT_SERVO_CONFIGS);
    
    if (!servo_controller_) {
        return false;
    }

    // Initialize the servo controller
    if (!servo_controller_->initialize()) {
        return false;
    }

    initialized_ = true;
    system_healthy_ = true;
    last_health_check_ = HAL_GetTick();
    
    return true;
}

void DOFRoboticArm::shutdown() {
    if (servo_controller_) {
        servo_controller_->shutdown();
        servo_controller_.reset();
    }
    
    initialized_ = false;
    system_healthy_ = false;
    emergency_stop_active_ = false;
}

bool DOFRoboticArm::isInitialized() const {
    return initialized_ && servo_controller_ && servo_controller_->isInitialized();
}

bool DOFRoboticArm::moveToPosition(const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>& positions, uint8_t speed) {
    if (!isInitialized()) {
        return false;
    }

    if (!validatePositions(positions)) {
        return false;
    }

    // Create commands for all servos
    std::vector<ServoCommand> commands;
    commands.reserve(static_cast<size_t>(ServoID::COUNT));
    
    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        ServoCommand cmd;
        cmd.type = CommandType::MOVE_TO_POSITION;
        cmd.servo_id = static_cast<ServoID>(i);
        cmd.target_position = positions[i];
        cmd.speed = speed;
        cmd.acceleration = 90;
        cmd.deceleration = 90;
        cmd.timeout_ms = 5000;
        commands.push_back(cmd);
    }

    return servo_controller_->moveMultipleServos(commands);
}

bool DOFRoboticArm::moveToHome() {
    if (!isInitialized()) {
        return false;
    }

    // Define home positions for all servos
    std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> home_positions = {
        90,  // Base rotation
        90,  // Shoulder
        90,  // Elbow
        90,  // Wrist rotation
        90,  // Wrist pitch
        90   // Gripper
    };

    return moveToPosition(home_positions);
}

bool DOFRoboticArm::moveToRest() {
    if (!isInitialized()) {
        return false;
    }

    // Define rest positions for all servos
    std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)> rest_positions = {
        0,   // Base rotation
        45,  // Shoulder
        135, // Elbow
        90,  // Wrist rotation
        90,  // Wrist pitch
        90   // Gripper
    };

    return moveToPosition(rest_positions);
}

bool DOFRoboticArm::moveCameraForward() {
    if (!isInitialized()) {
        return false;
    }

    // Move camera to forward position
    return moveServo(ServoID::WRIST_PITCH, 0, 64); // Slow movement
}

bool DOFRoboticArm::moveCameraDown() {
    if (!isInitialized()) {
        return false;
    }

    // Move camera to downward position
    return moveServo(ServoID::WRIST_PITCH, 180, 64); // Slow movement
}

bool DOFRoboticArm::moveCameraToAngle(uint16_t pitch, uint16_t roll) {
    if (!isInitialized()) {
        return false;
    }

    bool success = true;
    
    // Set pitch (wrist pitch servo)
    if (pitch <= 180) {
        success &= moveServo(ServoID::WRIST_PITCH, pitch, 64);
    }
    
    // Set roll (wrist roll servo)
    if (roll <= 180) {
        success &= moveServo(ServoID::WRIST_ROLL, roll, 64);
    }
    
    return success;
}

bool DOFRoboticArm::moveServo(ServoID id, uint16_t position, uint8_t speed) {
    if (!isInitialized()) {
        return false;
    }

    return servo_controller_->moveServo(id, position, speed);
}

bool DOFRoboticArm::stopServo(ServoID id) {
    if (!isInitialized()) {
        return false;
    }

    return servo_controller_->stopServo(id);
}

bool DOFRoboticArm::homeServo(ServoID id) {
    if (!isInitialized()) {
        return false;
    }

    return servo_controller_->homeServo(id);
}

bool DOFRoboticArm::emergencyStop() {
    if (!isInitialized()) {
        return false;
    }

    emergency_stop_active_ = true;
    bool success = servo_controller_->emergencyStop();
    
    if (success) {
        for (auto& observer : observers_) {
            if (observer) {
                observer->onEmergencyStop();
            }
        }
    }
    
    return success;
}

bool DOFRoboticArm::stopAllServos() {
    if (!isInitialized()) {
        return false;
    }

    return servo_controller_->stopAllServos();
}

bool DOFRoboticArm::isMoving() const {
    if (!isInitialized()) {
        return false;
    }

    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        if (servo_controller_->isServoMoving(static_cast<ServoID>(i))) {
            return true;
        }
    }
    
    return false;
}

bool DOFRoboticArm::isHealthy() const {
    return system_healthy_ && isInitialized();
}

ServoTelemetry DOFRoboticArm::getServoTelemetry(ServoID id) const {
    if (!isInitialized()) {
        return ServoTelemetry{};
    }

    return servo_controller_->getServoTelemetry(id);
}

TelemetryArray DOFRoboticArm::getAllServoTelemetry() const {
    if (!isInitialized()) {
        return TelemetryArray{};
    }

    return servo_controller_->getAllServoTelemetry();
}

ServoStatus DOFRoboticArm::getServoStatus(ServoID id) const {
    if (!isInitialized()) {
        return ServoStatus::ERROR;
    }

    return servo_controller_->getServoStatus(id);
}

bool DOFRoboticArm::setServoConstraints(ServoID id, const ServoConstraints& constraints) {
    if (!isInitialized()) {
        return false;
    }

    return servo_controller_->setServoConstraints(id, constraints);
}

bool DOFRoboticArm::calibrateAllServos() {
    if (!isInitialized()) {
        return false;
    }

    bool success = true;
    for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
        success &= servo_controller_->calibrateServo(static_cast<ServoID>(i));
    }
    
    return success;
}

void DOFRoboticArm::addObserver(std::shared_ptr<IServoObserver> observer) {
    if (observer) {
        observers_.push_back(observer);
    }
}

void DOFRoboticArm::removeObserver(std::shared_ptr<IServoObserver> observer) {
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
}

void DOFRoboticArm::update() {
    if (!isInitialized()) {
        return;
    }

    // Update servo controller
    servo_controller_->update();
    
    // Update system health
    updateSystemHealth();
    
    // Update telemetry if needed
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_telemetry_update_ >= TELEMETRY_UPDATE_INTERVAL_MS) {
        last_telemetry_update_ = current_time;
        
        TelemetryArray telemetry = getAllServoTelemetry();
        for (auto& observer : observers_) {
            if (observer) {
                observer->onTelemetryUpdate(telemetry);
            }
        }
    }
}

std::string DOFRoboticArm::getStatusString() const {
    if (!isInitialized()) {
        return "Not initialized";
    }

    std::string status = "System: " + std::string(isHealthy() ? "Healthy" : "Unhealthy");
    status += ", Emergency Stop: " + std::string(emergency_stop_active_ ? "Active" : "Inactive");
    status += ", Moving: " + std::string(isMoving() ? "Yes" : "No");
    
    return status;
}

void DOFRoboticArm::printTelemetry() const {
    if (!isInitialized()) {
        return;
    }

    TelemetryArray telemetry = getAllServoTelemetry();
    for (size_t i = 0; i < telemetry.size(); ++i) {
        (void)telemetry[i]; // Suppress unused variable warning
        // In a real implementation, you might use UART or other output
        // For now, we'll just store the data
    }
}

bool DOFRoboticArm::validatePositions(const std::array<uint16_t, static_cast<size_t>(ServoID::COUNT)>& positions) const {
    for (size_t i = 0; i < positions.size(); ++i) {
        if (positions[i] > 180) {
            return false;
        }
    }
    return true;
}

void DOFRoboticArm::updateSystemHealth() {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_health_check_ >= HEALTH_CHECK_INTERVAL_MS) {
        last_health_check_ = current_time;
        
        // Check if servo controller is healthy
        system_healthy_ = servo_controller_->isSystemHealthy();
        
        // Check for emergency stop
        if (emergency_stop_active_) {
            system_healthy_ = false;
        }
        
        // Check for any servo errors
        for (size_t i = 0; i < static_cast<size_t>(ServoID::COUNT); ++i) {
            ServoStatus status = getServoStatus(static_cast<ServoID>(i));
            if (status == ServoStatus::ERROR) {
                system_healthy_ = false;
                break;
            }
        }
    }
}

void DOFRoboticArm::logMovement(const std::string& movement_type, bool success) {
    // In a real implementation, you might log to flash memory or send via UART
    // For now, we'll just store the information
    (void)movement_type;
    (void)success;
}

} // namespace DOF

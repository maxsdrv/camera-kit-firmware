/**
 * @file CommunicationManager.cpp
 * @brief Simple UART-based communication manager implementation
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "CommunicationManager.h"
#include <sstream>
#include <algorithm>

namespace DOF {

CommunicationManager::CommunicationManager(DOF::DOFRoboticArm* robotic_arm, 
                                          UART_HandleTypeDef* uart1, 
                                          UART_HandleTypeDef* uart2)
    : robotic_arm_(robotic_arm)
    , uart1_(uart1)
    , uart2_(uart2)
    , initialized_(false)
    , running_(false)
    , uart_rx_size_(0)
    , uart_tx_size_(0)
{
}

CommunicationManager::~CommunicationManager()
{
    stop();
}

bool CommunicationManager::initialize()
{
    if (initialized_) {
        return true;
    }
    
    if (!robotic_arm_ || !uart1_) {
        return false;
    }
    
    // Initialize UART
    HAL_UART_Receive_IT(uart1_, uart_rx_buffer_, 1);
    
    initialized_ = true;
    return true;
}

void CommunicationManager::start()
{
    if (!initialized_ || running_) {
        return;
    }
    
    running_ = true;
    
    if (status_callback_) {
        status_callback_("Communication manager started");
    }
}

void CommunicationManager::stop()
{
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (status_callback_) {
        status_callback_("Communication manager stopped");
    }
}

void CommunicationManager::restart()
{
    stop();
    start();
}

void CommunicationManager::setStatusCallback(StatusCallback callback)
{
    status_callback_ = callback;
}

void CommunicationManager::setErrorCallback(ErrorCallback callback)
{
    error_callback_ = callback;
}

void CommunicationManager::submitCommand(const CommunicationCommand& command)
{
    if (!running_) {
        return;
    }
    
    executeCommand(command);
}

void CommunicationManager::emergencyStop()
{
    if (robotic_arm_) {
        robotic_arm_->emergencyStop();
    }
    
    if (status_callback_) {
        status_callback_("Emergency stop executed");
    }
    
    sendResponse("EMERGENCY_STOP:OK");
}

void CommunicationManager::processUARTData()
{
    if (!running_ || uart_rx_size_ == 0) {
        return;
    }
    
    // Convert received data to string
    std::string command_str(reinterpret_cast<char*>(uart_rx_buffer_), uart_rx_size_);
    
    // Parse and execute command
    parseUARTCommand(command_str);
    
    // Clear buffer
    uart_rx_size_ = 0;
}

void CommunicationManager::sendTelemetry()
{
    if (!robotic_arm_ || !uart1_ || !running_) {
        return;
    }
    
    // Get telemetry from robotic arm
    auto telemetry = robotic_arm_->getAllServoTelemetry();
    
    // Send via UART
    std::ostringstream oss;
    oss << "TELEMETRY:";
    
    for (size_t i = 0; i < telemetry.size(); ++i) {
        if (i > 0) oss << ",";
        oss << telemetry[i].current_position << "|"
            << telemetry[i].target_position << "|"
            << static_cast<int>(telemetry[i].current_speed) << "|"
            << (telemetry[i].status == ServoStatus::MOVING ? 1 : 0) << "|"
            << static_cast<int>(telemetry[i].error);
    }
    
    oss << ":" << HAL_GetTick() << "\n";
    
    std::string telemetry_str = oss.str();
    HAL_UART_Transmit(uart1_, 
                     reinterpret_cast<const uint8_t*>(telemetry_str.c_str()),
                     telemetry_str.length(), 
                     1000);
}

void CommunicationManager::sendResponse(const std::string& response)
{
    if (uart1_) {
        HAL_UART_Transmit(uart1_, 
                         reinterpret_cast<const uint8_t*>(response.c_str()),
                         response.length(), 
                         1000);
    }
}

void CommunicationManager::executeCommand(const CommunicationCommand& command)
{
    if (!robotic_arm_) {
        return;
    }
    
    switch (command.type) {
        case CommunicationCommandType::MOVE_SERVO:
            robotic_arm_->moveServo(
                static_cast<DOF::ServoID>(command.servo_id),
                command.angle,
                command.speed
            );
            break;
            
        case CommunicationCommandType::MOVE_TO_POSITION:
            {
                std::array<uint16_t, 6> positions_array;
                for (int i = 0; i < 6; ++i) {
                    positions_array[i] = command.positions[i];
                }
                robotic_arm_->moveToPosition(positions_array, command.speed);
            }
            break;
            
        case CommunicationCommandType::HOME_POSITION:
            robotic_arm_->moveToHome();
            break;
            
        case CommunicationCommandType::REST_POSITION:
            robotic_arm_->moveToRest();
            break;
            
        case CommunicationCommandType::EMERGENCY_STOP:
            robotic_arm_->emergencyStop();
            break;
            
        case CommunicationCommandType::CAMERA_FORWARD:
            robotic_arm_->moveCameraForward();
            break;
            
        case CommunicationCommandType::CAMERA_DOWN:
            robotic_arm_->moveCameraDown();
            break;
            
        case CommunicationCommandType::CAMERA_ANGLE:
            robotic_arm_->moveCameraToAngle(command.positions[4], command.positions[5]);
            break;
            
        default:
            break;
    }
    
    // Send response via UART
    std::string response = "OK:" + std::to_string(static_cast<int>(command.type)) + "\n";
    sendResponse(response);
}

void CommunicationManager::parseUARTCommand(const std::string& command_str)
{
    // Remove whitespace and newlines
    std::string trimmed = command_str;
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), '\r'), trimmed.end());
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), '\n'), trimmed.end());
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), ' '), trimmed.end());
    
    if (trimmed.empty()) {
        return;
    }
    
    // Parse different command types
    if (trimmed.find("MOVE_SERVO:") == 0) {
        handleMoveServoCommand(trimmed.substr(11));
    }
    else if (trimmed.find("MOVE_POSITION:") == 0) {
        handleMovePositionCommand(trimmed.substr(14));
    }
    else if (trimmed == "HOME") {
        handleHomeCommand();
    }
    else if (trimmed == "REST") {
        handleRestCommand();
    }
    else if (trimmed == "EMERGENCY_STOP") {
        handleEmergencyStopCommand();
    }
    else if (trimmed == "STATUS") {
        handleStatusCommand();
    }
    else {
        sendResponse("ERROR:Unknown command");
    }
}

void CommunicationManager::handleMoveServoCommand(const std::string& params)
{
    // Parse MOVE_SERVO:servo_id,angle,speed
    std::istringstream iss(params);
    std::string token;
    
    CommunicationCommand cmd;
    cmd.type = CommunicationCommandType::MOVE_SERVO;
    cmd.timestamp = HAL_GetTick();
    
    if (std::getline(iss, token, ',')) {
        cmd.servo_id = std::stoi(token);
    }
    if (std::getline(iss, token, ',')) {
        cmd.angle = std::stoi(token);
    }
    if (std::getline(iss, token, ',')) {
        cmd.speed = std::stoi(token);
    } else {
        cmd.speed = 128; // Default speed
    }
    
    // Validate parameters
    if (cmd.servo_id >= static_cast<uint8_t>(ServoID::COUNT)) {
        sendResponse("ERROR:Invalid servo ID");
        return;
    }
    
    if (cmd.angle > 180) {
        sendResponse("ERROR:Invalid angle");
        return;
    }
    
    if (cmd.speed > 255) {
        sendResponse("ERROR:Invalid speed");
        return;
    }
    
    submitCommand(cmd);
}

void CommunicationManager::handleMovePositionCommand(const std::string& params)
{
    // Parse MOVE_POSITION:pos1,pos2,pos3,pos4,pos5,pos6,speed
    std::istringstream iss(params);
    std::string token;
    
    CommunicationCommand cmd;
    cmd.type = CommunicationCommandType::MOVE_TO_POSITION;
    cmd.timestamp = HAL_GetTick();
    cmd.speed = 128; // Default speed
    
    // Parse positions
    for (int i = 0; i < 6; i++) {
        if (std::getline(iss, token, ',')) {
            cmd.positions[i] = std::stoi(token);
        } else {
            cmd.positions[i] = 0;
        }
    }
    
    // Parse speed if available
    if (std::getline(iss, token, ',')) {
        cmd.speed = std::stoi(token);
    }
    
    submitCommand(cmd);
}

void CommunicationManager::handleHomeCommand()
{
    CommunicationCommand cmd;
    cmd.type = CommunicationCommandType::HOME_POSITION;
    cmd.timestamp = HAL_GetTick();
    submitCommand(cmd);
}

void CommunicationManager::handleRestCommand()
{
    CommunicationCommand cmd;
    cmd.type = CommunicationCommandType::REST_POSITION;
    cmd.timestamp = HAL_GetTick();
    submitCommand(cmd);
}

void CommunicationManager::handleEmergencyStopCommand()
{
    emergencyStop();
}

void CommunicationManager::handleStatusCommand()
{
    std::ostringstream oss;
    oss << "STATUS:running=" << (running_ ? "true" : "false")
        << ",uptime=" << HAL_GetTick() << "\n";
    sendResponse(oss.str());
}

std::string CommunicationManager::createJsonResponse(bool success, const std::string& message, const std::string& data)
{
    std::ostringstream oss;
    oss << "{\"success\":" << (success ? "true" : "false") 
        << ",\"message\":\"" << message << "\"";
    
    if (!data.empty()) {
        oss << ",\"data\":" << data;
    }
    
    oss << "}";
    return oss.str();
}

void CommunicationManager::sendTelemetry(const TelemetryArray& telemetry) {
    // Send telemetry data via UART
    std::ostringstream oss;
    oss << "TELEMETRY:";
    
    for (size_t i = 0; i < telemetry.size(); ++i) {
        if (i > 0) oss << ",";
        oss << telemetry[i].current_position << "|"
            << telemetry[i].target_position << "|"
            << static_cast<int>(telemetry[i].current_speed) << "|"
            << (telemetry[i].status == ServoStatus::MOVING ? 1 : 0) << "|"
            << static_cast<int>(telemetry[i].error);
    }
    
    oss << ":" << HAL_GetTick() << "\n";
    sendResponse(oss.str());
}

void CommunicationManager::sendStatus(const std::string& status) {
    // Send status message via UART
    std::string response = "STATUS:" + status + "\n";
    sendResponse(response);
}

std::vector<CommunicationCommand> CommunicationManager::getCommands() {
    // Process any pending UART data first
    processUARTData();
    
    // Return any commands that were parsed
    std::vector<CommunicationCommand> commands;
    // For now, return empty vector - commands are processed immediately
    return commands;
}

bool CommunicationManager::isTimeout() const {
    // Simple timeout check - in real implementation, track last communication time
    return false; // For now, never timeout
}

} // namespace DOF
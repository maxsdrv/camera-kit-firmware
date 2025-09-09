/**
 * @file CommunicationManager.h
 * @brief Simple UART-based communication manager for STM32
 * @version 1.0.0
 * @date 2025-01-04
 */

#pragma once

#include "../board/Inc/stm32_hal_config.h"
#include "../gimbal/controllers/dof_robotic_arm.h"
#include "../gimbal/types/servo_types.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace DOF {

/**
 * @brief Communication command types
 */
enum class CommunicationCommandType : uint8_t {
    MOVE_SERVO = 1,
    MOVE_TO_POSITION = 2,
    HOME_POSITION = 3,
    REST_POSITION = 4,
    EMERGENCY_STOP = 5,
    CAMERA_FORWARD = 6,
    CAMERA_DOWN = 7,
    CAMERA_ANGLE = 8,
    GET_STATUS = 9,
    GET_TELEMETRY = 10
};

/**
 * @brief Communication command structure
 */
struct CommunicationCommand {
    CommunicationCommandType type;
    uint8_t servo_id;
    uint16_t angle;
    uint8_t speed;
    uint16_t positions[6];
    uint32_t timestamp;
    
    CommunicationCommand() 
        : type(CommunicationCommandType::MOVE_SERVO)
        , servo_id(0)
        , angle(0)
        , speed(128)
        , positions{0, 0, 0, 0, 0, 0}
        , timestamp(0)
    {}
};

/**
 * @brief Simple communication manager for STM32
 * 
 * Uses only UART communication - no threading, no HTTP, no WebSocket.
 * Designed specifically for STM32 embedded systems.
 */
class CommunicationManager {
public:
    using StatusCallback = std::function<void(const std::string&)>;
    using ErrorCallback = std::function<void(const std::string&)>;
    
    CommunicationManager(DOF::DOFRoboticArm* robotic_arm, 
                        UART_HandleTypeDef* uart1, 
                        UART_HandleTypeDef* uart2);
    ~CommunicationManager();
    
    // Core functionality
    bool initialize();
    void start();
    void stop();
    void restart();
    
    // Status
    bool isRunning() const { return running_; }
    bool isInitialized() const { return initialized_; }
    
    // Callbacks
    void setStatusCallback(StatusCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // Command processing
    void submitCommand(const CommunicationCommand& command);
    void emergencyStop();
    
    // UART communication
    void processUARTData();
    void sendTelemetry();
    void sendTelemetry(const TelemetryArray& telemetry);
    void sendResponse(const std::string& response);
    
    // Status methods
    void sendStatus(const std::string& status);
    
    // Command processing
    std::vector<CommunicationCommand> getCommands();
    
    // Timeout checking
    bool isTimeout() const;

private:
    // Core components
    DOF::DOFRoboticArm* robotic_arm_;
    UART_HandleTypeDef* uart1_;
    UART_HandleTypeDef* uart2_;
    
    // State
    bool initialized_;
    bool running_;
    
    // Callbacks
    StatusCallback status_callback_;
    ErrorCallback error_callback_;
    
    // UART buffers
    uint8_t uart_rx_buffer_[256];
    uint8_t uart_tx_buffer_[256];
    size_t uart_rx_size_;
    size_t uart_tx_size_;
    
    // Internal methods
    void executeCommand(const CommunicationCommand& command);
    void parseUARTCommand(const std::string& command_str);
    void handleMoveServoCommand(const std::string& params);
    void handleMovePositionCommand(const std::string& params);
    void handleHomeCommand();
    void handleRestCommand();
    void handleEmergencyStopCommand();
    void handleStatusCommand();
    std::string createJsonResponse(bool success, const std::string& message, const std::string& data = "");
};

} // namespace DOF
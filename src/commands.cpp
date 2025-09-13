/**
 * @file commands.cpp
 * @brief UART command processing class implementation
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "commands.h"
#include "pca9685_servo.h"
#include "main.h"
#include <cstring>
#include <cstdio>

// External I2C handle
extern I2C_HandleTypeDef hi2c1;

// Static member definitions
UART_HandleTypeDef* Commands::uart_handle_ = nullptr;
uint8_t Commands::cmd_buffer_[UART_CMD_BUFFER_SIZE];
volatile uint8_t Commands::cmd_index_ = 0;
volatile bool Commands::command_ready_ = false;

void Commands::Init(UART_HandleTypeDef* huart)
{
    uart_handle_ = huart;
    cmd_index_ = 0;
    command_ready_ = false;
}

void Commands::StartReceiving()
{
    if (uart_handle_)
    {
        HAL_UART_Receive_IT(uart_handle_, &cmd_buffer_[0], 1);
    }
}

void Commands::ProcessCommands()
{
    if (command_ready_)
    {
        ProcessCommand();
        ResetCommand();
        StartReceiving(); // Restart UART reception
    }
}

void Commands::SendResponse(const char* response)
{
    if (uart_handle_)
    {
        uint16_t len = strlen(response);
        // Use polling mode with longer timeout
        HAL_UART_Transmit(
            uart_handle_,
            reinterpret_cast<const uint8_t*>(response),
            len,
            5000); // Much longer timeout

        // Wait for transmission to complete
        while (HAL_UART_GetState(uart_handle_) != HAL_UART_STATE_READY)
        {
            // Wait for UART to be ready
        }
    }
}

void Commands::OnUartReceiveComplete(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        // Check for command terminator
        if (cmd_buffer_[cmd_index_] == '\n' || cmd_buffer_[cmd_index_] == '\r')
        {
            cmd_buffer_[cmd_index_] = '\0';
            command_ready_ = true;
            return;
        }

        // Continue receiving if not at the end of buffer
        if (cmd_index_ < UART_CMD_BUFFER_SIZE - 1)
        {
            cmd_index_ = cmd_index_ + 1;
            HAL_UART_Receive_IT(uart_handle_, &cmd_buffer_[cmd_index_], 1);
        }
        else
        {
            // Buffer full, reset
            cmd_index_ = 0;
            HAL_UART_Receive_IT(uart_handle_, &cmd_buffer_[0], 1);
        }
    }
}

bool Commands::IsCommandReady()
{
    return command_ready_;
}

void Commands::ResetCommand()
{
    command_ready_ = false;
    cmd_index_ = 0;
}


void Commands::ProcessCommand()
{
    auto cmd = reinterpret_cast<char*>(cmd_buffer_);

    // New command format examples:
    // "S0090" - Set servo 0 to 90 degrees
    // "H" - Home all servos
    // "?" - Get status
    // "E" - Emergency stop
    // "R" - Clear emergency stop (resume)
    // "P1" - Camera forward preset
    // "P2" - Camera down preset
    // "P0" - Rest position preset

    if (cmd[0] == 'S' && cmd_index_ >= 5)
    {
        // Servo command: S<id><angle_degrees>
        uint8_t servo_id = cmd[1] - '0';
        uint16_t angle_degrees =
            (cmd[2] - '0') * 100 + (cmd[3] - '0') * 10 + (cmd[4] - '0');

        if (servo_id < 6 && angle_degrees <= 180)
        {
            if (PCA9685_SetServoAngle(servo_id, angle_degrees))
            {
                SendResponse("OK\n");
            }
            else
            {
                SendResponse("ERR_CONSTRAINT\n");
            }
        }
        else
        {
            SendResponse("ERR_PARAM\n");
        }
    }
    else if (cmd[0] == 'H')
    {
        // Home command
        if (PCA9685_HomeAllServos())
        {
            SendResponse("HOME_OK\n");
        }
        else
        {
            SendResponse("HOME_ERR\n");
        }
    }
    else if (cmd[0] == 'E')
    {
        // Emergency stop
        PCA9685_EmergencyStop();
        SendResponse("ESTOP_ACTIVE\n");
    }
    else if (cmd[0] == 'R')
    {
        // Resume from emergency stop
        if (PCA9685_ClearEmergencyStop())
        {
            SendResponse("RESUME_OK\n");
        }
        else
        {
            SendResponse("RESUME_ERR\n");
        }
    }
    else if (cmd[0] == 'P' && cmd_index_ >= 2)
    {
        // Preset command: P<preset_id>
        uint8_t preset_id = cmd[1] - '0';
        bool success = false;

        switch (preset_id)
        {
            case 0:                                       // Rest position
                success = PCA9685_SetServoAngle(0, 90) && // Base rotation
                          PCA9685_SetServoAngle(1, 30) &&
                          // Shoulder pitch (folded)
                          PCA9685_SetServoAngle(2, 90) && // Shoulder roll
                          PCA9685_SetServoAngle(3, 160) &&
                          // Elbow pitch (folded)
                          PCA9685_SetServoAngle(4, 30) &&
                          // Wrist pitch (folded)
                          PCA9685_SetServoAngle(5, 90); // Wrist roll
                break;

            case 1:                                       // Camera forward
                success = PCA9685_SetServoAngle(0, 90) && // Base rotation
                          PCA9685_SetServoAngle(1, 90) && // Shoulder pitch
                          PCA9685_SetServoAngle(2, 90) && // Shoulder roll
                          PCA9685_SetServoAngle(3, 90) && // Elbow pitch
                          PCA9685_SetServoAngle(4, 90) && // Wrist pitch
                          PCA9685_SetServoAngle(5, 90);   // Wrist roll
                break;

            case 2:                                       // Camera down
                success = PCA9685_SetServoAngle(0, 90) && // Base rotation
                          PCA9685_SetServoAngle(1, 90) && // Shoulder pitch
                          PCA9685_SetServoAngle(2, 90) && // Shoulder roll
                          PCA9685_SetServoAngle(3, 90) && // Elbow pitch
                          PCA9685_SetServoAngle(4, 150) &&
                          // Wrist pitch (pointing down)
                          PCA9685_SetServoAngle(5, 90); // Wrist roll
                break;

            default: SendResponse("ERR_PRESET\n");
                return;
        }

        if (success)
        {
            SendResponse("PRESET_OK\n");
        }
        else
        {
            SendResponse("PRESET_ERR\n");
        }
    }
    else if (cmd[0] == 'D' && cmd[1] == 'S')
    {
        // Debug: I2C Device Scan - "DS"
        SendResponse("I2C_SCAN_START\n");
        uint8_t devices_found = 0;
        
        for (uint8_t addr = 0x08; addr <= 0x77; addr++)
        {
            HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 50);
            if (result == HAL_OK)
            {
                devices_found++;
                char found_msg[32];
                snprintf(found_msg, sizeof(found_msg), "FOUND:0x%02X\n", addr);
                SendResponse(found_msg);
            }
        }
        
        char summary[64];
        snprintf(summary, sizeof(summary), "I2C_SCAN:TOTAL=%d\n", devices_found);
        SendResponse(summary);
    }
    else if (cmd[0] == 'D' && cmd[1] == 'P')
    {
        // Debug: PCA9685 specific test - "DP"
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, 0x40 << 1, 1, 100);
        char response[64];
        snprintf(
            response,
            sizeof(response),
            "PCA9685_TEST:RESULT=%s\n",
            result == HAL_OK ? "OK" : 
            result == HAL_ERROR ? "ERROR" : 
            result == HAL_BUSY ? "BUSY" : 
            result == HAL_TIMEOUT ? "TIMEOUT" : "UNKNOWN");
        SendResponse(response);
    }
    else if (cmd[0] == '?')
    {
        // Status command - shows gimbal status and I2C health
        if (PCA9685_IsConnected())
        {
            char status[256];
            snprintf(
                status,
                sizeof(status),
                "GIMBAL:OK,S0=%s,S1=%s,S2=%s,S3=%s,S4=%s,S5=%s\n",
                PCA9685_GetServoStatus(0),
                PCA9685_GetServoStatus(1),
                PCA9685_GetServoStatus(2),
                PCA9685_GetServoStatus(3),
                PCA9685_GetServoStatus(4),
                PCA9685_GetServoStatus(5));
            SendResponse(status);
        }
        else
        {
            SendResponse("GIMBAL:ERROR,I2C_DISCONNECTED\n");
            SendResponse("TRY:DS(scan),DP(pca_test)\n");
        }
    }
    else
    {
        SendResponse("ERR_CMD\n");
    }
}
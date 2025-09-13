/**
 * @file commands.h
 * @brief UART command processing class
 * @version 1.0.0
 * @date 2025-01-04
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <cstdint>

#define UART_CMD_BUFFER_SIZE 64

class Commands
{
public:
    Commands() = delete;
    /**
     * @brief Initialize the Commands system
     * @param huart Pointer to UART handle
     */
    static void Init(UART_HandleTypeDef* huart);

    /**
     * @brief Start receiving UART commands
     */
    static void StartReceiving();

    /**
     * @brief Process incoming UART command (call from main loop)
     */
    static void ProcessCommands();

    /**
     * @brief Send response via UART
     * @param response Response string to send
     */
    static void SendResponse(const char* response);

    /**
     * @brief UART receive a complete callback (call from interrupt)
     * @param huart UART handle that received data
     */
    static void OnUartReceiveComplete(UART_HandleTypeDef* huart);

    /**
     * @brief Check if the command is ready for processing
     * @return true if command is ready
     */
    static bool IsCommandReady();

    /**
     * @brief Reset command processing state
     */
    static void ResetCommand();

    /**
     * @brief Test command processing - simple responses for testing
     */
    static void ProcessCommandTest();

private:
    static UART_HandleTypeDef* uart_handle_;
    static uint8_t cmd_buffer_[UART_CMD_BUFFER_SIZE];
    static volatile uint8_t cmd_index_;
    static volatile bool command_ready_;

    /**
     * @brief Process the received command
     */
    static void ProcessCommand();
};

#ifdef __cplusplus
}
#endif

#endif // COMMANDS_H
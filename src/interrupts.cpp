/**
 * @file interrupts.cpp
 * @brief Interrupt callback function implementations
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "commands.h"
#include "pca9685_servo.h"
#include "main.h"

/**
 * @brief Timer interrupt callback for heartbeat
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2)
    {
        // Heartbeat - toggle LED every 100 interrupts (1Hz at 100Hz timer)
        static uint16_t counter = 0;
        counter++;
        if (counter >= 100)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            counter = 0;
        }
    }
}

/**
 * @brief GPIO interrupt callback for the emergency stop button
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)
    {
        // Emergency stop - activate emergency stop mode
        PCA9685_EmergencyStop();
        Commands::SendResponse("ESTOP_BUTTON\n");
    }
}

/**
 * @brief UART receive complete callback
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    Commands::OnUartReceiveComplete(huart);
}
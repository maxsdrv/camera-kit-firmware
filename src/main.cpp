/**
 * @file main.cpp
 * @brief DOF Camera Gimbal Firmware - Simple Direct Control
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "board/Inc/stm32_hal_config.h"
#include "pca9685_servo.h"
#include <cstring>
#include <cstdio>

// Hardware handles
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

// UART command buffer
#define UART_CMD_BUFFER_SIZE 64
static uint8_t uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
static volatile uint8_t uart_cmd_index = 0;
static volatile bool command_ready = false;

// Function prototypes
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_UART1_Init(void);
void MX_TIM2_Init(void);
static void ProcessUARTCommand(void);
static void SendUARTResponse(const char* response);

/**
 * @brief Main function
 */
int main(void)
{
    // Initialize HAL
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_UART1_Init();
    MX_TIM2_Init();

    // Initialize PCA9685 servo controller
    PCA9685_Init(&hi2c1);

    // Start UART interrupt for command reception
    HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[0], 1);

    // Start timer for heartbeat
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    // Turn on status LED
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    // Main firmware loop
    while (1)
    {
        // Process UART commands
        if (command_ready) {
            ProcessUARTCommand();
            command_ready = false;
            uart_cmd_index = 0;
            // Restart UART reception
            HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[0], 1);
        }

        // Small delay to prevent CPU overload
        HAL_Delay(1);
    }
}

/**
 * @brief Process incoming UART command
 */
static void ProcessUARTCommand(void)
{
    char* cmd = (char*)uart_cmd_buffer;
    
    // Command format examples:
    // "S0375" - Set servo 0 to position 375
    // "H" - Home all servos
    // "?" - Get status
    
    if (cmd[0] == 'S' && uart_cmd_index >= 5) {
        // Servo command: S<id><position>
        uint8_t servo_id = cmd[1] - '0';
        uint16_t position = (cmd[2] - '0') * 100 + (cmd[3] - '0') * 10 + (cmd[4] - '0');
        
        if (servo_id < SERVO_COUNT && position >= SERVO_MIN_PULSE && position <= SERVO_MAX_PULSE) {
            PCA9685_SetServo(servo_id, position);
            SendUARTResponse("OK\n");
        } else {
            SendUARTResponse("ERR\n");
        }
    }
    else if (cmd[0] == 'H') {
        // Home command
        PCA9685_HomeAllServos();
        SendUARTResponse("HOME\n");
    }
    else if (cmd[0] == '?') {
        // Status command
        char status[128];
        snprintf(status, sizeof(status), 
                "STATUS:S0=%d,S1=%d,S2=%d,S3=%d,S4=%d,S5=%d\n",
                servo_positions[0], servo_positions[1], servo_positions[2],
                servo_positions[3], servo_positions[4], servo_positions[5]);
        SendUARTResponse(status);
    }
    else {
        SendUARTResponse("ERR\n");
    }
}

/**
 * @brief Send response via UART
 */
static void SendUARTResponse(const char* response)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
}

/**
 * @brief Timer interrupt callback for heartbeat
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // Heartbeat - toggle LED every 100 interrupts (1Hz at 100Hz timer)
        static uint16_t counter = 0;
        counter++;
        if (counter >= 100) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            counter = 0;
        }
    }
}

/**
 * @brief GPIO interrupt callback for emergency stop button
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        // Emergency stop - home all servos
        PCA9685_HomeAllServos();
        SendUARTResponse("ESTOP\n");
    }
}

/**
 * @brief UART receive complete callback
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Check for command terminator
        if (uart_cmd_buffer[uart_cmd_index] == '\n' || uart_cmd_buffer[uart_cmd_index] == '\r') {
            uart_cmd_buffer[uart_cmd_index] = '\0';
            command_ready = true;
            return;
        }
        
        // Continue receiving if not at end of buffer
        if (uart_cmd_index < UART_CMD_BUFFER_SIZE - 1) {
            uart_cmd_index = uart_cmd_index + 1;
            HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[uart_cmd_index], 1);
        } else {
            // Buffer full, reset
            uart_cmd_index = 0;
            HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[0], 1);
        }
    }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief UART1 Initialization Function
 */
void MX_UART1_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 */
void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 */
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 499;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        
        for(volatile int i = 0; i < 100000; i += 1);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        for(volatile int i = 0; i < 100000; i += 1);
    }
}
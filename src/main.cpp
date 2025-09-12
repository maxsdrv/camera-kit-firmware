/**
 * @file main.cpp
 * @brief DOF Camera Gimbal Firmware - Simple Direct Control
 * @version 1.0.0
 * @date 2025-01-04
 */

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
static void MX_GPIO_Init();
static void MX_USART1_UART_Init();
static void MX_I2C1_Init();
static void MX_TIM2_Init();
static void SystemClock_Config();

static void ProcessUARTCommand();
static void SendUARTResponse(const char* response);

[[noreturn]] int main()
{
    // Initialize HAL
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    
    // Skip I2C recovery for now - test direct initialization
    
    MX_I2C1_Init();
    MX_TIM2_Init();

    // Initialize PCA9685 servo controller
    if (!PCA9685_Init(&hi2c1)) {
        Error_Handler();
    }

    // Start UART interrupt for command reception
    HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[0], 1);

    // Start timer for heartbeat
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    // Turn on status LED
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    // Main firmware loop
    while (true)
    {
        // Process UART commands
        // if (command_ready) {
        // ProcessUARTCommand();
        // command_ready = false;
        // uart_cmd_index = 0;
        // // Restart UART reception
        // HAL_UART_Receive_IT(&huart1, &uart_cmd_buffer[0], 1);
        // }

        // Small delay to prevent CPU overload
        HAL_Delay(1);
    }
}

/**
 * @brief Process incoming UART command
 */
static void ProcessUARTCommand()
{
    auto cmd = reinterpret_cast<char *>(uart_cmd_buffer);
    
    // New command format examples:
    // "S0090" - Set servo 0 to 90 degrees
    // "H" - Home all servos
    // "?" - Get status
    // "E" - Emergency stop
    // "R" - Clear emergency stop (resume)
    // "P1" - Camera forward preset
    // "P2" - Camera down preset
    // "P0" - Rest position preset
    
    if (cmd[0] == 'S' && uart_cmd_index >= 5) {
        // Servo command: S<id><angle_degrees>
        uint8_t servo_id = cmd[1] - '0';
        uint16_t angle_degrees = (cmd[2] - '0') * 100 + (cmd[3] - '0') * 10 + (cmd[4] - '0');
        
        if (servo_id < 6 && angle_degrees <= 180) {
            if (PCA9685_SetServoAngle(servo_id, angle_degrees)) {
                SendUARTResponse("OK\n");
            } else {
                SendUARTResponse("ERR_CONSTRAINT\n");
            }
        } else {
            SendUARTResponse("ERR_PARAM\n");
        }
    }
    else if (cmd[0] == 'H') {
        // Home command
        if (PCA9685_HomeAllServos()) {
            SendUARTResponse("HOME_OK\n");
        } else {
            SendUARTResponse("HOME_ERR\n");
        }
    }
    else if (cmd[0] == 'E') {
        // Emergency stop
        PCA9685_EmergencyStop();
        SendUARTResponse("ESTOP_ACTIVE\n");
    }
    else if (cmd[0] == 'R') {
        // Resume from emergency stop
        if (PCA9685_ClearEmergencyStop()) {
            SendUARTResponse("RESUME_OK\n");
        } else {
            SendUARTResponse("RESUME_ERR\n");
        }
    }
    else if (cmd[0] == 'P' && uart_cmd_index >= 2) {
        // Preset command: P<preset_id>
        uint8_t preset_id = cmd[1] - '0';
        bool success = false;
        
        switch (preset_id) {
            case 0: // Rest position
                success = PCA9685_SetServoAngle(0, 90) &&  // Base rotation
                         PCA9685_SetServoAngle(1, 30) &&  // Shoulder pitch (folded)
                         PCA9685_SetServoAngle(2, 90) &&  // Shoulder roll
                         PCA9685_SetServoAngle(3, 160) && // Elbow pitch (folded)
                         PCA9685_SetServoAngle(4, 30) &&  // Wrist pitch (folded)
                         PCA9685_SetServoAngle(5, 90);    // Wrist roll
                break;
                
            case 1: // Camera forward
                success = PCA9685_SetServoAngle(0, 90) &&  // Base rotation
                         PCA9685_SetServoAngle(1, 90) &&  // Shoulder pitch
                         PCA9685_SetServoAngle(2, 90) &&  // Shoulder roll
                         PCA9685_SetServoAngle(3, 90) &&  // Elbow pitch
                         PCA9685_SetServoAngle(4, 90) &&  // Wrist pitch
                         PCA9685_SetServoAngle(5, 90);    // Wrist roll
                break;
                
            case 2: // Camera down
                success = PCA9685_SetServoAngle(0, 90) &&  // Base rotation
                         PCA9685_SetServoAngle(1, 90) &&  // Shoulder pitch
                         PCA9685_SetServoAngle(2, 90) &&  // Shoulder roll
                         PCA9685_SetServoAngle(3, 90) &&  // Elbow pitch
                         PCA9685_SetServoAngle(4, 150) && // Wrist pitch (pointing down)
                         PCA9685_SetServoAngle(5, 90);    // Wrist roll
                break;
                
            default:
                SendUARTResponse("ERR_PRESET\n");
                return;
        }
        
        if (success) {
            SendUARTResponse("PRESET_OK\n");
        } else {
            SendUARTResponse("PRESET_ERR\n");
        }
    }
    else if (cmd[0] == '?') {
        // Status command - now shows angles and status
        char status[256];
        snprintf(status, sizeof(status), 
                "STATUS:S0=%s,S1=%s,S2=%s,S3=%s,S4=%s,S5=%s\n",
                PCA9685_GetServoStatus(0), PCA9685_GetServoStatus(1), PCA9685_GetServoStatus(2),
                PCA9685_GetServoStatus(3), PCA9685_GetServoStatus(4), PCA9685_GetServoStatus(5));
        SendUARTResponse(status);
    }
    else {
        SendUARTResponse("ERR_CMD\n");
    }
}

/**
 * @brief Send response via UART
 */
static void SendUARTResponse(const char* response)
{
    HAL_UART_Transmit(&huart1,
        reinterpret_cast<const uint8_t *>(*response),
        strlen(response),
        100);
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
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
void MX_USART1_UART_Init()
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
void MX_I2C1_Init()
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
void MX_TIM2_Init()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;
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
void MX_GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
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
 * @brief GPIO interrupt callback for the emergency stop button
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        // Emergency stop - activate emergency stop mode
        PCA9685_EmergencyStop();
        SendUARTResponse("ESTOP_BUTTON\n");
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

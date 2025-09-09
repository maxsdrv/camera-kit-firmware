/**
 * @file main.cpp
 * @brief DOF Camera Kit Firmware - Main Application
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "board/Inc/stm32_hal_config.h"
#include "communication/CommunicationManager.h"
#include "gimbal/controllers/dof_robotic_arm.h"
#include "telemetry/telemetry_system.h"
#include <array>
#include <memory>

// Hardware handles
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

// System components
std::unique_ptr<DOF::DOFRoboticArm> robotic_arm;
std::unique_ptr<DOF::CommunicationManager> comm_manager;
std::unique_ptr<DOF::TelemetrySystem> telemetry_system;

// System state
volatile uint32_t system_tick = 0;
volatile bool emergency_stop_flag = false;

// Function prototypes
void DOF_SystemInit(void);
void DOF_MainLoop(void);
void DOF_ProcessCommands(void);
void DOF_UpdateTelemetry(void);
void DOF_SafetyCheck(void);

/**
 * @brief Main application entry point
 */
int main(void)
{
    // Initialize HAL
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    // Initialize DOF system
    DOF_SystemInit();

    // Main loop
    while (1) {
        DOF_MainLoop();
    }
}

/**
 * @brief Initialize DOF robotic arm system
 */
void DOF_SystemInit(void)
{
    // Create robotic arm instance
    robotic_arm = std::make_unique<DOF::DOFRoboticArm>(&hi2c1);

    // Create communication manager
    comm_manager = std::make_unique<DOF::CommunicationManager>(robotic_arm.get(), &huart1, &huart2);

    // Create telemetry system
    telemetry_system = std::make_unique<DOF::TelemetrySystem>();

    // Initialize robotic arm
    if (!robotic_arm->initialize()) {
        Error_Handler();
    }

    // Initialize communication manager
    if (!comm_manager->initialize()) {
        Error_Handler();
    }
    
    // Start communication manager
    comm_manager->start();

    // Initialize telemetry system
    telemetry_system->initialize();

    // Move to home position
    robotic_arm->moveToHome();

    // Start telemetry timer (100Hz)
    HAL_TIM_Base_Start_IT(&htim2);

    // System ready
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

/**
 * @brief Main system loop (called continuously)
 */
void DOF_MainLoop(void)
{
    // Process incoming commands
    DOF_ProcessCommands();

    // Update robotic arm state
    robotic_arm->update();

    // Perform safety checks
    DOF_SafetyCheck();

    // Handle emergency stop
    if (emergency_stop_flag) {
        robotic_arm->emergencyStop();
        emergency_stop_flag = false;
    }

    // Small delay to prevent CPU overload
    // Use a debug-friendly delay that doesn't interfere with debugging
    #ifdef DEBUG
        // In debug mode, use a shorter delay or no delay
        for(volatile int i = 0; i < 1000; i = i + 1);
    #else
        HAL_Delay(1);
    #endif
}

/**
 * @brief Process incoming commands from different interfaces
 */
void DOF_ProcessCommands(void)
{
    // Process UART commands
    auto commands = comm_manager->getCommands();

    for (const auto& cmd : commands) {
        switch (cmd.type) {
            case DOF::CommunicationCommandType::MOVE_SERVO:
                robotic_arm->moveServo(
                    static_cast<DOF::ServoID>(cmd.servo_id),
                    cmd.angle,
                    cmd.speed
                );
                break;

            case DOF::CommunicationCommandType::MOVE_TO_POSITION:
                {
                    std::array<uint16_t, 6> positions_array;
                    for (int i = 0; i < 6; ++i) {
                        positions_array[i] = cmd.positions[i];
                    }
                    robotic_arm->moveToPosition(positions_array, cmd.speed);
                }
                break;

            case DOF::CommunicationCommandType::HOME_POSITION:
                robotic_arm->moveToHome();
                break;

            case DOF::CommunicationCommandType::REST_POSITION:
                robotic_arm->moveToRest();
                break;

            case DOF::CommunicationCommandType::EMERGENCY_STOP:
                emergency_stop_flag = true;
                break;

            case DOF::CommunicationCommandType::CAMERA_FORWARD:
                robotic_arm->moveCameraForward();
                break;

            case DOF::CommunicationCommandType::CAMERA_DOWN:
                robotic_arm->moveCameraDown();
                break;

            case DOF::CommunicationCommandType::CAMERA_ANGLE:
                robotic_arm->moveCameraToAngle(cmd.positions[4], cmd.positions[5]);
                break;

            case DOF::CommunicationCommandType::GET_STATUS:
                comm_manager->sendStatus(robotic_arm->getStatusString());
                break;

            case DOF::CommunicationCommandType::GET_TELEMETRY:
                comm_manager->sendTelemetry(robotic_arm->getAllServoTelemetry());
                break;

            default:
                // Unknown command
                break;
        }

        // Update telemetry with movement count
        telemetry_system->incrementMovementCount();
    }
}

/**
 * @brief Update and send telemetry data
 */
void DOF_UpdateTelemetry(void)
{
    // Get current telemetry from robotic arm
    auto telemetry = robotic_arm->getAllServoTelemetry();

    // Update telemetry system
    telemetry_system->update(telemetry);

    // Send telemetry via communication interfaces
    comm_manager->sendTelemetry(telemetry);
}

/**
 * @brief Perform system safety checks
 */
void DOF_SafetyCheck(void)
{
    // Check system health
    if (!robotic_arm->isHealthy()) {
        // Handle system error
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

        // Log error
        comm_manager->sendStatus("SYSTEM_ERROR: Robotic arm unhealthy");
    }

    // Check communication timeout
    if (comm_manager->isTimeout()) {
        // Move to safe position
        robotic_arm->moveToRest();
        comm_manager->sendStatus("WARNING: Communication timeout, moving to safe position");
    }
}

/**
 * @brief Timer interrupt callback for telemetry (100Hz)
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        system_tick = system_tick + 1;

        // Update telemetry every 10ms (100Hz)
        DOF_UpdateTelemetry();
    }
}

/**
 * @brief GPIO interrupt callback for emergency stop button
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        emergency_stop_flag = true;
        telemetry_system->incrementEmergencyStopCount();
    }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART1_UART_Init(void)
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
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
 * @param None
 * @retval None
 */
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399; // 84MHz / 8400 = 10kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99; // 10kHz / 100 = 100Hz (10ms)
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
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void)
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

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    while (1)
    {
        // Error state - blink LED rapidly
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
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
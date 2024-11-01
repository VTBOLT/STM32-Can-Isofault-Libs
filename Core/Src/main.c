/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_mimic.h"
#include "iso_fault_lib.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile int five = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BSP_PB_Callback(Button_TypeDef Button) {
    if (Button == BUTTON_USER) {
        BSP_LED_Toggle(LED_GREEN);
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */
    float *measuredFreq = malloc(sizeof(float *));
    float *measuredDutyCycle = malloc(sizeof(float *));
    char *messageToSend = malloc(100 * sizeof(char));
    int noChars;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_TIM3_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    /* USER CODE BEGIN 2 */
    IsoFault_Init(&htim3);
    /* USER CODE END 2 */

    /* Initialize led */
    BSP_LED_Init(LED_GREEN);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
    BspCOMInit.BaudRate = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits = COM_STOPBITS_1;
    BspCOMInit.Parity = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        get_signal_measurements(measuredFreq, measuredDutyCycle);
        noChars = sprintf(messageToSend, "Frequency: %.2f Hz\tDuty cycle: %.2f%%\t%.2f kohm\r\n", *measuredFreq, *measuredDutyCycle * 100, IsoFault_GetResistance_kOhms());
        HAL_UART_Transmit(hcom_uart, (uint8_t *)messageToSend, noChars, 1000);

        // CAN Time!
        FDCAN_SendMessage(&hfdcan1, 0x123, (uint8_t *)messageToSend, noChars);
        HAL_Delay(250);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

    /* USER CODE BEGIN FDCAN1_Init 0 */

    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

    /* USER CODE END FDCAN1_Init 1 */
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 68;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 5;
    hfdcan1.Init.NominalTimeSeg2 = 4;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN1_Init 2 */

    /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

    /* USER CODE BEGIN FDCAN2_Init 0 */

    /* USER CODE END FDCAN2_Init 0 */

    /* USER CODE BEGIN FDCAN2_Init 1 */

    /* USER CODE END FDCAN2_Init 1 */
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = DISABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;
    hfdcan2.Init.NominalPrescaler = 68;
    hfdcan2.Init.NominalSyncJumpWidth = 1;
    hfdcan2.Init.NominalTimeSeg1 = 5;
    hfdcan2.Init.NominalTimeSeg2 = 4;
    hfdcan2.Init.DataPrescaler = 1;
    hfdcan2.Init.DataSyncJumpWidth = 1;
    hfdcan2.Init.DataTimeSeg1 = 1;
    hfdcan2.Init.DataTimeSeg2 = 1;
    hfdcan2.Init.StdFiltersNbr = 0;
    hfdcan2.Init.ExtFiltersNbr = 0;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN2_Init 2 */

    /* USER CODE END FDCAN2_Init 2 */
}

/**
 * @brief FDCAN3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN3_Init(void) {

    /* USER CODE BEGIN FDCAN3_Init 0 */

    /* USER CODE END FDCAN3_Init 0 */

    /* USER CODE BEGIN FDCAN3_Init 1 */

    /* USER CODE END FDCAN3_Init 1 */
    hfdcan3.Instance = FDCAN3;
    hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan3.Init.AutoRetransmission = DISABLE;
    hfdcan3.Init.TransmitPause = DISABLE;
    hfdcan3.Init.ProtocolException = DISABLE;
    hfdcan3.Init.NominalPrescaler = 68;
    hfdcan3.Init.NominalSyncJumpWidth = 1;
    hfdcan3.Init.NominalTimeSeg1 = 5;
    hfdcan3.Init.NominalTimeSeg2 = 4;
    hfdcan3.Init.DataPrescaler = 1;
    hfdcan3.Init.DataSyncJumpWidth = 1;
    hfdcan3.Init.DataTimeSeg1 = 1;
    hfdcan3.Init.DataTimeSeg2 = 1;
    hfdcan3.Init.StdFiltersNbr = 0;
    hfdcan3.Init.ExtFiltersNbr = 0;
    hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN3_Init 2 */

    /* USER CODE END FDCAN3_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 679;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x0;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */
    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin : IsoFaultBoard_IsFault_Pin */
    GPIO_InitStruct.Pin = IsoFaultBoard_IsFault_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IsoFaultBoard_IsFault_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  255     //最大的接受字节数
#define TXBUFFERSIZE  255     //最大发送字节数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char RxBuffer[RXBUFFERSIZE];   //接受数据
char TxBuffer[TXBUFFERSIZE];   //发送数据
uint8_t aRxBuffer;            //接受中断缓冲
uint8_t Uart1_Rx_Cnt = 0;    //接受缓冲计数
uint8_t RxSucceeflag = 0;    //接受成功标志

// 传感器数据变量
float aX = 0, aY = 0, aZ = 0;          // 加速度数据
float wX = 0, wY = 0, wZ = 0;          // 角速度数据
float RollX = 0, PitchY = 0, YawZ = 0; // 姿态角数据

// Z轴角度归零指令
uint8_t hexData[] = {0xFF, 0xAA, 0x52};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  
  // 确保正确启动第一次接收
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 44) != HAL_OK)
  {
      Error_Handler();
  }
  
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(RxSucceeflag == 1)
    {
        HAL_UART_Transmit(&huart2, hexData, sizeof(hexData), HAL_MAX_DELAY);
        memset(TxBuffer, 0x00, sizeof(TxBuffer));
        sprintf(TxBuffer, "W_X%.2f,W_Y%.2f,W_Z%.2f,R%.2f,P%.2f,Y_A%.2f\r\n", 
                wX, wY, wZ, RollX, PitchY, YawZ);
        HAL_UART_Transmit(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer), 0xFFFF);
        
        // 等待发送完成
        while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
        
        // 最后再清除标志
        RxSucceeflag = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int checkSum(char RxBuffer[])
{
    int sum = 0;
    for(int i = 0; i < 10; i++)
    {
        sum += RxBuffer[i];
    }
    return (RxBuffer[10] == (char)(sum)) ? 1 : -1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        // 清除错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
        
        // 重新启动接收
        if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 44) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart != &huart2) return;

    // Check for valid packet header (0x55)
    if(RxBuffer[0] != 0x55) {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 44);
        return;
    }

    // Process acceleration data (0x51)
    if(RxBuffer[1] == 0x51) {
        int16_t rawAx = (RxBuffer[3] << 8) | RxBuffer[2];
        int16_t rawAy = (RxBuffer[5] << 8) | RxBuffer[4];
        int16_t rawAz = (RxBuffer[7] << 8) | RxBuffer[6];
        
        // Convert to physical units (m/s²)
        aX = (float)(rawAx) / 32768.0f * 16.0f * 9.81f;
        aY = (float)(rawAy) / 32768.0f * 16.0f * 9.81f;
        aZ = (float)(rawAz) / 32768.0f * 16.0f * 9.81f;
    }

    // Process angular velocity data (0x52)
    if(RxBuffer[12] == 0x52) {
        int16_t rawWx = (RxBuffer[14] << 8) | RxBuffer[13];
        int16_t rawWy = (RxBuffer[16] << 8) | RxBuffer[15];
        int16_t rawWz = (RxBuffer[18] << 8) | RxBuffer[17];
        
        // Convert to degrees per second
        wX = (float)(rawWx) / 32768.0f * 2000.0f;
        wY = (float)(rawWy) / 32768.0f * 2000.0f;
        wZ = (float)(rawWz) / 32768.0f * 2000.0f;
    }

    // Process angle data (0x53)
    if(RxBuffer[23] == 0x53) {
        int16_t rawRoll = (RxBuffer[25] << 8) | RxBuffer[24];
        int16_t rawPitch = (RxBuffer[27] << 8) | RxBuffer[26];
        int16_t rawYaw = (RxBuffer[29] << 8) | RxBuffer[28];
        
        // Convert to degrees
        RollX = (float)(rawRoll) / 32768.0f * 180.0f;
        PitchY = (float)(rawPitch) / 32768.0f * 180.0f;
        YawZ = (float)(rawYaw) / 32768.0f * 180.0f;
    }

    // 在函数末尾确保重新启动接收
    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 44) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 将RxSucceeflag的设置移到这里
    RxSucceeflag = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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

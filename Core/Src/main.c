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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"
#include "encoder.h"
//#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  255     //最大的接受字节数
#define TXBUFFERSIZE  255     //最大发送字节数
#define MAX_SPEED 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    float wheel_L; // 左前轮速度
    float wheel_R; // 右前轮速度
} WheelSpeeds;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

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

WheelSpeeds Currentspeeds;
bool Speedupdateflag = 0, Pidupdateflag = 0, debug = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MotorRun(int left_front_MotorPWM, int right_front_MotorPWM)
{
    // 左前轮 (FL)
    if (left_front_MotorPWM == 0) {
        // Brake - set both channels high
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_SPEED);
    } else if (left_front_MotorPWM > 0) {
        // Forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, left_front_MotorPWM);
    } else {
        // Backward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -left_front_MotorPWM);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }

    // 右前轮 (FR) 
    if (right_front_MotorPWM == 0) {
        // Brake - set both channels high
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MAX_SPEED);
    } else if (right_front_MotorPWM > 0) {
        // Forward
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, right_front_MotorPWM);
    } else {
        // Backward
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, -right_front_MotorPWM);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
}
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
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // 确保正确启动第一次接收
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 44) != HAL_OK)
  {
      Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(RxSucceeflag == 1)
    {
//        //HAL_UART_Transmit(&huart2, hexData, sizeof(hexData), HAL_MAX_DELAY);
//        memset(TxBuffer, 0x00, sizeof(TxBuffer));
//        sprintf(TxBuffer, "W_X%.2f,W_Y%.2f,W_Z%.2f,R%.2f,P%.2f,Y_A%.2f\r\n",
//                wX, wY, wZ, RollX, PitchY, YawZ);
//        HAL_UART_Transmit(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer), 0xFFFF);
//        // 等待发送完成
//        while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
//
//        // 最后再清除标志
        RxSucceeflag = 0;
    }
    if(debug == 1){

             /* ------------------调试用------------------*/
    //	        // 利用 sprintf 将结果附加到 info 数组后面
    //	     	 char info[100]="Target:";
    //	        sprintf(info + strlen(info), " FL: %6.2f", Targetspeeds.wheel_FL);
    //	        sprintf(info + strlen(info), " FR: %6.2f", Targetspeeds.wheel_FR);
    //	        sprintf(info + strlen(info), " RL: %6.2f", Targetspeeds.wheel_RL);
    //	        sprintf(info + strlen(info), " RR: %6.2f\n", Targetspeeds.wheel_RR);
    //	        HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 50);
    	     // 调试输出当前脉冲数据
    	     char info1[100] = "Pace:";
    	     sprintf(info1 + strlen(info1), " L: %6.2f", Currentspeeds.wheel_L);
    	     sprintf(info1 + strlen(info1), " R: %6.2f\n", Currentspeeds.wheel_R);
    	     HAL_UART_Transmit(&huart1, (uint8_t*)info1, strlen(info1), 100);
    	     debug = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&htim6) /*&& Pidupdateflag == 0*/)
  {
		 Currentspeeds.wheel_R = CalculatePulse(GetEncoderPulse_2());
		 Currentspeeds.wheel_L = -CalculatePulse(GetEncoderPulse_3());
	  Pidupdateflag = 1;
  }
  else if(htim == (&htim7))
  {
	  debug = 1;

  }
}

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

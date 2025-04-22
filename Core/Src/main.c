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
#include "adc.h"
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
#include "pid.h"
//#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  255     //最大的接受字节数
#define TXBUFFERSIZE  255     //最大发送字节数
#define MAX_DUTY 1000
//#define MAX_SPEED 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char RxBuffer1[RXBUFFERSIZE];   //接受数据
char TxBuffer1[TXBUFFERSIZE];   //发送数据
uint8_t RxSucceeflag1 = 0;    //接受成功标志

char RxBuffer2[RXBUFFERSIZE];   //接受数据
char TxBuffer2[TXBUFFERSIZE];   //发送数据
uint8_t RxSucceeflag2 = 0;    //接受成功标志

SensorData sensorData = {0};  // Initialize all values to 0
float Batvol=0;
// Z轴角度归零指令
uint8_t hexData[] = {0xFF, 0xAA, 0x52};


WheelSpeeds Currentspeeds = {0}; // Initialize all values to 0
int TargetSpeed = 0, TargetOmega = 0;
bool Speedupdateflag = 0, Pidupdateflag = 0, debug = 0,stop=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 * @param pid_out  PID 输出，范围 [0,1000]
 * @param voltage  当前电池电压，范围约 11.1–12.6 V
 * @return         PWM 占空比 duty，范围 [0,1000]
 */
float out2duty(float pid_out, float voltage) {
    // 1. 计算起动死区对应的最小占空比
    const float V_start = 2.2f;      // 起转电压
    float duty_min = V_start / voltage * 1000.0f;

    // 2. 计算满转对应的最大占空比（不超过 1000）
    const float V_full  = 12.0f;     // 满转电压
    float duty_max = V_full / voltage * 1000.0f;
    if (duty_max > 1000.0f) duty_max = 1000.0f;

    // 3. 线性映射
    float duty_range = duty_max - duty_min;
    return duty_min + (pid_out / 1000.0f) * duty_range;
}
void MotorRun(WheelSpeeds speeds)
{
	float voltage= Batvol;         // 获取转换结果

    // 左前轮 (FL)
    if (speeds.wheel_L == 0) {
        // Brake - set both channels high
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_DUTY);
    } else if (speeds.wheel_L > 0) {
        // Forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, out2duty(speeds.wheel_L,voltage));
    } else {
        // Backward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, out2duty(speeds.wheel_L,voltage));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }

    // 右前轮 (FR) 
    if (speeds.wheel_R == 0) {
        // Brake - set both channels high
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MAX_DUTY);
    } else if (speeds.wheel_R > 0) {
        // Forward
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, out2duty(speeds.wheel_R,voltage));
    } else {
        // Backward
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, out2duty(speeds.wheel_R,voltage));
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // 确保正确启动第一次接收
  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer1, 5) != HAL_OK) // 修改接收长度为5
  {
      Error_Handler();
  }
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer2, 44) != HAL_OK)
  {
        Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7); //debug
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

//  MotorRun(speed);
  TargetSpeed=0;

  HAL_ADC_PollForConversion(&hadc1, 50); // 等待转换完成
  Batvol=(HAL_ADC_GetValue(&hadc1) / 4095.0) * 3.3*11;           // 获取转换结果

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    if(RxSucceeflag2 == 1)
//    {
//      // HAL_UART_Transmit(&huart2, hexData, sizeof(hexData), HAL_MAX_DELAY);
//    	//上面其他作用
//
//        memset(TxBuffer1, 0x00, sizeof(TxBuffer1));
//        sprintf(TxBuffer1, "W_X%.2f,W_Y%.2f,W_Z%.2f,R%.2f,P%.2f,Y_A%.2f\r\n",
//                sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
//                sensorData.angle.roll, sensorData.angle.pitch, sensorData.angle.yaw);
//        HAL_UART_Transmit(&huart1, (uint8_t*)TxBuffer1, strlen(TxBuffer1), 100);
//
//        RxSucceeflag2 = 0;
//    }


//    if(Pidupdateflag == 1){
//
//    	if(stop==0){
//    	MotorRun(balance(sensorData, Currentspeeds,
//        TargetSpeed, TargetOmega,
//        &angle_pid, &speed_pid, &turn_pid, &wheel_pid_L, &wheel_pid_R));
//    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
//    	}
//    	else{
//    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_DUTY);
//    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_DUTY);
//    	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MAX_DUTY);
//    	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MAX_DUTY);
//    	}
//    	Pidupdateflag = 0;
//    }
    if(debug == 1){
//
//             /* ------------------调试用------------------*/
//    	        // 利用 sprintf 将结果附加到 info 数组后面
//    	     char info[100]="Target:";
//    	        sprintf(info + strlen(info), " S: %6d", TargetSpeed);
//    	        sprintf(info + strlen(info), " O: %6d\n", TargetOmega);
//    	        HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 50);
//    	     // 调试输出当前脉冲数据
//    	     char info1[100] = "Pace:";
//    	     sprintf(info1 + strlen(info1), " L: %6.2f", Currentspeeds.wheel_L);
//    	     sprintf(info1 + strlen(info1), " R: %6.2f\n", Currentspeeds.wheel_R);
//    	     HAL_UART_Transmit(&huart1, (uint8_t*)info1, strlen(info1), 100);

    	     debug = 0;
    	     if(sensorData.angle.pitch>=60||sensorData.angle.pitch<=-60){
    	    	 stop=1;
    	     }
    	     else
    	    	 stop=0;

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
		     	if(stop==0){
		     	MotorRun(balance(sensorData, Currentspeeds,
		         TargetSpeed, TargetOmega,
		         &angle_pid, &speed_pid, &turn_pid, &wheel_pid_L, &wheel_pid_R));
		     	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		     	}
		     	else
		     	{
		     	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_DUTY);
		     	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_DUTY);
		     	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MAX_DUTY);
		     	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MAX_DUTY);
		     	}
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
        if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer2, 44) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if(huart == &huart1)
    {
        // 清除错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);

        // 重新启动接收
        if(HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer1, 5) != HAL_OK) // 修改接收长度为5
        {
            Error_Handler();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1) {
        // Check for valid packet header (0x55)
        if(RxBuffer1[0] != 0x55) {
            HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer1, 5);
            return;
        }

        // Process target speed data
        TargetSpeed = (RxBuffer1[2] << 8) | RxBuffer1[1];
        TargetOmega = (RxBuffer1[4] << 8) | RxBuffer1[3];
        
        // 将速度更新标志设置为1
        Speedupdateflag = 1;

        // 在函数末尾确保重新启动接收
        if(HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer1, 5) != HAL_OK)
        {
            Error_Handler();
        }
        
        // 将RxSucceeflag的设置移到这里
        RxSucceeflag1 = 1;
    }
    
    if(huart == &huart2) {

        // Check for valid packet header (0x55)
        if(RxBuffer2[0] != 0x55) {
            HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer2, 44);
            return;
        }

        // Process acceleration data (0x51)
        if(RxBuffer2[1] == 0x51) {
            int16_t rawAx = (RxBuffer2[3] << 8) | RxBuffer2[2];
            int16_t rawAy = (RxBuffer2[5] << 8) | RxBuffer2[4];
            int16_t rawAz = (RxBuffer2[7] << 8) | RxBuffer2[6];

            // Convert to physical units (m/s²)
            sensorData.accel.x = (float)(rawAx) / 32768.0f * 16.0f * 9.81f;
            sensorData.accel.y = (float)(rawAy) / 32768.0f * 16.0f * 9.81f;
            sensorData.accel.z = (float)(rawAz) / 32768.0f * 16.0f * 9.81f;
        }

        // Process angular velocity data (0x52)
        if(RxBuffer2[12] == 0x52) {
            int16_t rawWx = (RxBuffer2[14] << 8) | RxBuffer2[13];
            int16_t rawWy = (RxBuffer2[16] << 8) | RxBuffer2[15];
            int16_t rawWz = (RxBuffer2[18] << 8) | RxBuffer2[17];

            // Convert to degrees per second
            sensorData.gyro.x = (float)(rawWx) / 32768.0f * 2000.0f;
            sensorData.gyro.y = (float)(rawWy) / 32768.0f * 2000.0f;
            sensorData.gyro.z = (float)(rawWz) / 32768.0f * 2000.0f;
        }

        // Process angle data (0x53)
        if(RxBuffer2[23] == 0x53) {
            int16_t rawRoll = (RxBuffer2[25] << 8) | RxBuffer2[24];
            int16_t rawPitch = (RxBuffer2[27] << 8) | RxBuffer2[26];
            int16_t rawYaw = (RxBuffer2[29] << 8) | RxBuffer2[28];

            // Convert to degrees
            sensorData.angle.roll = (float)(rawRoll) / 32768.0f * 180.0f;
            sensorData.angle.pitch = (float)(rawPitch) / 32768.0f * 180.0f;
            sensorData.angle.yaw = (float)(rawYaw) / 32768.0f * 180.0f;
        }

        // 在函数末尾确保重新启动接收
        if(HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer2, 44) != HAL_OK)
        {
            Error_Handler();
        }

        // 将RxSucceeflag的设置移到这里
        RxSucceeflag2 = 1;

        }
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

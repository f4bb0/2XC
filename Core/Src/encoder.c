/*
 * encoder.c
 *
 *  Created on: Apr 5, 2025
 *      Author: fabbo
 */
#include "encoder.h"
#include "tim.h"

//int GetEncoderPulse_0()
//{
//	  int encoderPulse = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COUNTER(&htim2,10000);   //计数值重新清零
//	    return encoderPulse;
//}
//int GetEncoderPulse_1()
//{
//	int encoderPulse = __HAL_TIM_GET_COUNTER(&htim3);
//	__HAL_TIM_SET_COUNTER(&htim3,10000);
//	    return encoderPulse;
//}
int GetEncoderPulse_2()
{
	int encoderPulse = __HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,30000);   //计数值重新清零
	    return encoderPulse;
}
int GetEncoderPulse_3()
{
	int encoderPulse= __HAL_TIM_GET_COUNTER(&htim5);
	__HAL_TIM_SET_COUNTER(&htim5,30000);
	    return encoderPulse;
}
//速度计算
float CalculatePulse(int pulse)
{
	return ( (float)(pulse-30000)*100.00/100.00 );
}


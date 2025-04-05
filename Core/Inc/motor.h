/*
 * motor.h
 *
 *  Created on: Apr 5, 2025
 *      Author: fabbo
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "pid.h"
////句柄
//#define MOTOR_TIM_CHANNEL1 TIM_CHANNEL_1
//#define MOTOR_TIM_CHANNEL2 TIM_CHANNEL_2
//#define MOTOR_TIM_CHANNEL3 TIM_CHANNEL_3
//#define MOTOR_TIM_CHANNEL3 TIM_CHANNEL_4
//快慢衰减枚举
typedef enum{
	SLOW_DECAY,
	FAST_DECAY
}DecayMode;//暂时默认快衰减

//函数声明

void Motor_Init(void);
void Motor_Stop(void);

void Motor1_Forward(uint8_t speed);
void Motor2_Forward(uint8_t speed);
void Motor1_Backward(uint8_t speed);
void Motor2_Backward(uint8_t speed);

void Motor1_Brake(void);
void Motor2_Brake(void);


#endif /* INC_MOTOR_H_ */

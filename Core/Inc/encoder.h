/*
 * encode.h
 *
 *  Created on: Apr 5, 2025
 *      Author: fabbo
 */


#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
//宏



//声明
int GetEncoderPulse_0(void);
int GetEncoderPulse_1(void);
int GetEncoderPulse_2(void);
int GetEncoderPulse_3(void);
float CalculatePulse(int pulse);

#endif /* INC_ENCODER_H_ */

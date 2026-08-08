#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

/* 函数说明：Motor_Init。 */

void Motor_Init(void);
void Motor_SetLeftPWM(int8_t Speed);
void Motor_SetRightPWM(int8_t Speed);

#endif

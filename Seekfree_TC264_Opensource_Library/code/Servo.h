#ifndef __SERVO_H__
#define __SERVO_H__

#include "zf_common_headfile.h"

/* 实车舵机校准参数，初始化和控制必须共用。 */
#define SERVO_CENTER_ANGLE  80U
#define SERVO_MIN_ANGLE      9U
#define SERVO_MAX_ANGLE    132U

/*
 * 舕机输出接口说明?50Hz PWM?角度范围为 0~132 度。
 */

void Servo_Init(void);
void Servo_SetAngleDeg(uint8_t angle_deg);

#endif

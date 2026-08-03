#ifndef __SERVO_H__
#define __SERVO_H__

#include "zf_common_headfile.h"

/* 实车舵机校准参数，初始化和控制必须共用。 */
#define SERVO_CENTER_ANGLE  150U
#define SERVO_MIN_ANGLE      100U
#define SERVO_MAX_ANGLE    175U
#if SERVO_MIN_ANGLE > SERVO_CENTER_ANGLE || SERVO_CENTER_ANGLE > SERVO_MAX_ANGLE
#error "SERVO_CENTER_ANGLE must be between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE"
#endif
#if SERVO_MAX_ANGLE > 180U
#error "SERVO_MAX_ANGLE must not exceed 180"
#endif

/*
 * 舵机输出接口说明：50Hz PWM，实车转向范围由上方限幅宏控制。
 */

void Servo_Init(void);
void Servo_SetAngleDeg(uint8_t angle_deg);

#endif

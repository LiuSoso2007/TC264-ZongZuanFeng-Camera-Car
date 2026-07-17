#ifndef __SERVO_H__
#define __SERVO_H__

#include "zf_common_headfile.h"

/* 实车舵机校准参数，初始化和控制必须共用。 */
#define SERVO_CENTER_ANGLE  80U
#define SERVO_MIN_ANGLE      9U
#define SERVO_MAX_ANGLE    132U

/*
 * Servo.h --- ?????? (50Hz PWM)
 *
 * ??: P33_9 (ATOM0_CH1)
 * ??: 20ms (50Hz)
 * ????: 0.5ms~2.5ms (????? 250~1250, ???? 0~132?)
 *
 * ??: Servo_SetAngleDeg() ???? 0~132? (??????)
 */

void Servo_Init(void);
void Servo_SetAngleDeg(uint8_t angle_deg);

#endif

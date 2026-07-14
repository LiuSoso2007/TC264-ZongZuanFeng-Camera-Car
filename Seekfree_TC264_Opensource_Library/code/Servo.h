#ifndef __SERVO_H__
#define __SERVO_H__

#include "zf_common_headfile.h"

/*
 * Servo.h --- 舵机驱动接口 (50Hz PWM)
 *
 * 引脚: P02_4 (ATOM0_CH4)
 * 周期: 20ms (50Hz)
 * 脉冲范围: 0.5ms~2.5ms (对应占空比 250~1250, 对应角度 0~180度)
 *
 * 输出限幅:
 *   Servo_SetAngleDeg() 已内置 0~180 度限幅
 *   PD_Update() 进一步限制在 9~140 度安全范围
 *
 * 调用流程:
 *   Servo_Init()           -- 初始化PWM, 输出中位(90度/750占空比)
 *   Servo_SetAngleDeg(deg) -- 设置角度 (0~180度)
 */

void Servo_Init(void);
void Servo_SetAngleDeg(uint8_t angle_deg);

#endif
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

/*
 * Motor.h --- 双路电机驱动接口 (双极PWM)
 *
 * 引脚:
 *   左电机: IN1=P21_2 (ATOM0_CH0), IN2=P21_3 (ATOM0_CH1)
 *   右电机: IN1=P21_4 (ATOM0_CH2), IN2=P21_5 (ATOM0_CH3)
 * PWM频率: 10kHz
 * 占空比:  0~10000 (对应0%~100%)
 *
 * 参数说明:
 *   Speed: 有符号速度值
 *     正数 = 正转 (Speed/100 = 占空比百分比)
 *     负数 = 反转
 *     范围: -100 ~ +100
 *
 * 输出限幅:
 *   内部 SpeedToDuty() 已将输入限幅在 -100~+100 范围
 *   确保不超出 PWM 合法占空比
 */

void Motor_Init(void);
void Motor_SetLeftPWM(int8_t Speed);
void Motor_SetRightPWM(int8_t Speed);

#endif
/******************************************************************************
 * Motor.c - TC264 电机驱动 (双极PWM)
 * 引脚: 左电机 IN1=P21_2, IN2=P21_4 / 右电机 IN1=P21_3, IN2=P21_5
 * PWM频率: 10kHz, 占空比范围: 0~10000 (0%~100%)
 ******************************************************************************/
#include "Motor.h"

/* ---------- 引脚定义 (按实际接线修改) ---------- */
#define MOTOR_LEFT_IN1   ATOM0_CH0_P21_2
#define MOTOR_LEFT_IN2   ATOM0_CH2_P21_4
#define MOTOR_RIGHT_IN1  ATOM1_CH1_P21_3
#define MOTOR_RIGHT_IN2  ATOM0_CH3_P21_5

#define MOTOR_PWM_FREQ    10000      /* 10kHz */
#define MOTOR_DUTY_MAX    10000      /* = PWM_DUTY_MAX */

uint32_t SpeedToDuty(int16_t speed)
{
    int32_t a = (speed >= 0) ? speed : -speed;
    /* 编码器速度可超过100；这里的100仅代表PWM物理占空比上限。 */
    if (a > 100) a = 100;
    return (uint32_t)((a * MOTOR_DUTY_MAX) / 100U);
}

void Motor_Init(void)
{
    pwm_init(MOTOR_LEFT_IN1,  MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_LEFT_IN2,  MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_IN1, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_IN2, MOTOR_PWM_FREQ, 0);
}

void Motor_SetLeftPWM(int16_t Speed)
{
    uint32_t duty = SpeedToDuty(Speed);
    /* 零速必须同时关闭两个桥臂，避免方向引脚残留导致电机继续转动。 */
    if (Speed == 0) {
        pwm_set_duty(MOTOR_LEFT_IN1, 0);
        pwm_set_duty(MOTOR_LEFT_IN2, 0);
        return;
    }
    if (Speed >= 0) {
        pwm_set_duty(MOTOR_LEFT_IN1, duty);
        pwm_set_duty(MOTOR_LEFT_IN2, 0);
    } else {
        pwm_set_duty(MOTOR_LEFT_IN1, duty);
        pwm_set_duty(MOTOR_LEFT_IN2, MOTOR_DUTY_MAX);
    }
}

void Motor_SetRightPWM(int16_t Speed)
{
    uint32_t duty = SpeedToDuty(Speed);
    /* 右电机接线极性相反，但零速同样必须将两个桥臂全部清零。 */
    if (Speed == 0) {
        pwm_set_duty(MOTOR_RIGHT_IN1, 0);
        pwm_set_duty(MOTOR_RIGHT_IN2, 0);
        return;
    }
    if (Speed >= 0) {
        pwm_set_duty(MOTOR_RIGHT_IN1, duty);
        pwm_set_duty(MOTOR_RIGHT_IN2, MOTOR_DUTY_MAX);
    } else {
        pwm_set_duty(MOTOR_RIGHT_IN1, duty);
        pwm_set_duty(MOTOR_RIGHT_IN2, 0);
    }
}

/******************************************************************************
 * Servo.c - TC264 舵机驱动 (50Hz PWM, 20ms周期)
 * 引脚: P33_9 (ATOM1_CH1_P33_9)
 * 脉冲范围: 500~2500 (0.5ms~2.5ms) 对应 0~180度
 ******************************************************************************/
#include "Servo.h"

/* ---------- 引脚定义 (按实际接线修改) ---------- */
#define SERVO_PWM_CH   ATOM0_CH1_P33_9

/* PWM_DUTY_MAX=10000, 50Hz周期20000us, 1%/degree映射:
 * 500/20000*10000=250, 2500/20000*10000=1250 */
#define SERVO_FREQ     50
#define SERVO_MIN      250
#define SERVO_MAX      1250

void Servo_Init(void)
{
    pwm_init(SERVO_PWM_CH, SERVO_FREQ, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
}

void Servo_SetAngleDeg(uint8_t angle_deg)
{
    uint32_t pulse;
    if (angle_deg > 132U) angle_deg = 132U;
    pulse = SERVO_MIN + ((uint32_t)angle_deg * (SERVO_MAX - SERVO_MIN)) / 180U;
    pwm_set_duty(SERVO_PWM_CH, pulse);
}

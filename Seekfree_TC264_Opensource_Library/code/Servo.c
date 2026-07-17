/******************************************************************************
 * Servo.c - TC264 èˆµæœºé©±åŠ¨ (50Hz PWM, 20mså‘¨æœŸ)
 * å¼•è„š: P33_9 (ATOM1_CH1_P33_9)
 * è„‰å†²èŒƒå›´: 500~2500 (0.5ms~2.5ms) å¯¹åº” 0~180åº¦
 ******************************************************************************/
#include "Servo.h"

/* ---------- å¼•è„šå®šä¹‰ (æŒ‰å®žé™…æŽ¥çº¿ä¿®æ”¹) ---------- */
#define SERVO_PWM_CH   ATOM0_CH1_P33_9

/* PWM_DUTY_MAX=10000, 50Hzå‘¨æœŸ20000us, 1%/degreeæ˜ å°„:
 * 500/20000*10000=250, 2500/20000*10000=1250 */
#define SERVO_FREQ     50
#define SERVO_MIN      250
#define SERVO_MAX      1250

void Servo_Init(void)
{
    pwm_init(SERVO_PWM_CH, SERVO_FREQ,
             SERVO_MIN + ((uint32_t)SERVO_CENTER_ANGLE
                        * (SERVO_MAX - SERVO_MIN)) / 180U);
}

void Servo_SetAngleDeg(uint8_t angle_deg)
{
    uint32_t pulse;
    /* Çý¶¯²ãÔÙ´ÎÖ´ÐÐË«ÏòÏÞ·ù£¬·ÀÖ¹ÉÏ²ãÒì³£Êä³ö¡£ */
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;
    if (angle_deg < SERVO_MIN_ANGLE) angle_deg = SERVO_MIN_ANGLE;
    pulse = SERVO_MIN + ((uint32_t)angle_deg * (SERVO_MAX - SERVO_MIN)) / 180U;
    pwm_set_duty(SERVO_PWM_CH, pulse);
}

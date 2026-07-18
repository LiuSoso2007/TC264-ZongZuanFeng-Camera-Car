/******************************************************************************
 * PID.c - PD舵机 + PI电机
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- PD ---- */
#define PD_ERR_DEAD_ZONE 3.0f  /* Err死区边界，范围内舵机回中。 */
static uint8_t s_pd_div = 1, s_pd_cnt = 0;
static float   s_pd_out = 0.0f, s_pd_err0 = 0.0f, s_pd_err1 = 0.0f;

void PD_Update(float Kp, float Kd)
{
    if (++s_pd_cnt < s_pd_div) return;
    s_pd_cnt = 0;

    s_pd_err1 = s_pd_err0;
    s_pd_err0 = Err;
    s_pd_out  = Kp * s_pd_err0 + Kd * (s_pd_err0 - s_pd_err1)
              + (float)SERVO_CENTER_ANGLE;
    if (s_pd_out > (float)SERVO_MAX_ANGLE) s_pd_out = (float)SERVO_MAX_ANGLE;
    if (s_pd_out < (float)SERVO_MIN_ANGLE) s_pd_out = (float)SERVO_MIN_ANGLE;

    if (Err >= -PD_ERR_DEAD_ZONE && Err <= PD_ERR_DEAD_ZONE)
        Servo_SetAngleDeg(SERVO_CENTER_ANGLE);
    else
        Servo_SetAngleDeg((uint8_t)s_pd_out);
}

/* ---- PI ---- */
void PI_Init(PI_t *pi, float kp, float ki, int16_t min_speed)
{
    pi->Kp          = kp;
    pi->Ki          = ki;
    pi->MinSpeed    = min_speed;
    pi->LastSpdErr  = 0;
    pi->Output      = 0.0f;
    pi->TargetSpeed = 0;
    pi->TargetBias  = 0;
}

int8_t PI_Update(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd)
{
    int16_t abs_err;
    if (pos_err < 0)
        abs_err = (int16_t)(-pos_err);
    else
        abs_err = (int16_t)pos_err;
    if (abs_err > 100) abs_err = 100;

    int16_t target;
    if (abs_err <= 5)
        target = str_spd;
    else
        target = pi->MinSpeed + (int16_t)((int32_t)(str_spd - pi->MinSpeed)
                                          * (100 - abs_err) / 100);
    target += pi->TargetBias;
    pi->TargetSpeed = target;

    int16_t err = target - act_spd;
    float   inc = pi->Kp * (float)(err - pi->LastSpdErr)
                + pi->Ki * (float)err;
    pi->Output += inc;
    pi->LastSpdErr = err;

    if (pi->Output > PI_OUT_MAX) pi->Output = PI_OUT_MAX;
    if (pi->Output < PI_OUT_MIN) pi->Output = PI_OUT_MIN;
    return (int8_t)pi->Output;
}

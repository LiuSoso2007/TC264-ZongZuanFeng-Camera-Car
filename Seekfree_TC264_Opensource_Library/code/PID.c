/******************************************************************************
 * PID.c - PD舵机 + PI电机
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- PD ---- */
#define PD_ERR_DEAD_ZONE 1.0f  /* Err死区边界，范围内舵机回中。 */
#define SERVO_MIN_SIDE_WEIGHT  0.78f  /* 100方向基础权重，负向误差越大时再按比例增强。 */
#define SERVO_MAX_SIDE_WEIGHT  0.78f  /* 175方向固定权重，以1为归一化基准。 */
static float   s_pd_out = 0.0f, s_pd_offset = 0.0f;
static float   s_pd_err0 = 0.0f, s_pd_err1 = 0.0f;

void PD_Update(float Kp, float Kd, float err)
{
    s_pd_err1 = s_pd_err0;
    s_pd_err0 = err;

    //以下是计算pd
    if(s_pd_err0 > 0)
        s_pd_offset = Kp * s_pd_err0 + (Kd - 0.2 * s_pd_err0)    * (s_pd_err0 - s_pd_err1);
    else
        s_pd_offset = Kp * s_pd_err0 + (Kd - 0.2 * (-s_pd_err0)) * (s_pd_err0 - s_pd_err1);

    //以下是计算偏移
    if (s_pd_offset < -7.0f)
        s_pd_offset *= SERVO_MIN_SIDE_WEIGHT * (1+(-7-s_pd_offset)/62);  //左偏增大
    else if(s_pd_offset > 4.0f)
        s_pd_offset *= SERVO_MAX_SIDE_WEIGHT * (1+(-4+s_pd_offset)/35);   //右偏增大
    else
        s_pd_offset *= SERVO_MAX_SIDE_WEIGHT;

    s_pd_out = (float)SERVO_CENTER_ANGLE + s_pd_offset;
    if (s_pd_out > (float)SERVO_MAX_ANGLE) s_pd_out = (float)SERVO_MAX_ANGLE;
    if (s_pd_out < (float)SERVO_MIN_ANGLE) s_pd_out = (float)SERVO_MIN_ANGLE;

    if (err >= -PD_ERR_DEAD_ZONE && err <= PD_ERR_DEAD_ZONE)
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

int8_t PI_Update(PI_t *pi, float pos_err, int16_t act_spd, float base_target_pulses)
{
    int16_t abs_err;
    if (pos_err != pos_err)
        abs_err = 100;
    else if (pos_err <= -100.0f || pos_err >= 100.0f)
        abs_err = 100;
    else if (pos_err < 0.0f)
        abs_err = (int16_t)(-pos_err);
    else
        abs_err = (int16_t)pos_err;

    float target_float;
    if (abs_err <= 2)
        target_float = base_target_pulses;
    else
        target_float = (float)pi->MinSpeed + (base_target_pulses - (float)pi->MinSpeed)
                     * (float)(100 - abs_err) / 100.0f;
    target_float += (float)pi->TargetBias;

    /* 所有速度比例完成后只量化一次，并防止异常浮点值或越界转换进入PI。 */
    int16_t target;
    if (target_float != target_float)       target = 0;
    else if (target_float >= 32767.0f)      target = 32767;
    else if (target_float <= -32768.0f)     target = -32768;
    else target = (int16_t)(target_float + (target_float >= 0.0f ? 0.5f : -0.5f));
    pi->TargetSpeed = target;

    int32_t err = (int32_t)target - (int32_t)act_spd;
    float   inc = pi->Kp * (float)(err - pi->LastSpdErr)
                + pi->Ki * (float)err;
    pi->Output += inc;
    pi->LastSpdErr = err;

    if (pi->Output != pi->Output)   pi->Output = 0.0f;
    if (pi->Output > PI_OUT_MAX)    pi->Output = PI_OUT_MAX;
    if (pi->Output < PI_OUT_MIN)    pi->Output = PI_OUT_MIN;
    return (int8_t)pi->Output;
}

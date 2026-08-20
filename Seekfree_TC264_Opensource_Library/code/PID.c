/******************************************************************************
 * PID.c - PD舵机 + PI电机
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- PD ---- */
#define PD_ERR_DEAD_ZONE 1.0f  /* Err死区边界，范围内舵机回中。 */
#define SERVO_MIN_SIDE_WEIGHT  1.25f  /* 100方向基础权重，负向误差越大时再按比例增强。 */
#define SERVO_MAX_SIDE_WEIGHT  1.25f  /* 175方向固定权重，以1为归一化基准。 */
static float   s_pd_out = 0.0f, s_pd_offset = 0.0f;
static float   s_pd_err0 = 0.0f, s_pd_err1 = 0.0f;

void PD_Update(float Kp, float Kd, float err)
{
    s_pd_err1 = s_pd_err0;
    s_pd_err0 = err;

    //以下是计算pd
    if(s_pd_err0 > PD_ERR_DEAD_ZONE)
        s_pd_offset = Kp * (s_pd_err0-PD_ERR_DEAD_ZONE) - Kd * (s_pd_err0 - s_pd_err1) * ((10-s_pd_err0)/10);
    else if(-s_pd_err0 > PD_ERR_DEAD_ZONE)
        s_pd_offset = Kp * (s_pd_err0+PD_ERR_DEAD_ZONE) - Kd * (s_pd_err0 - s_pd_err1) * ((10+s_pd_err0)/10);
    else
        s_pd_offset = 0;

    //以下是计算偏移
    if (s_pd_offset < -0.0f)
        s_pd_offset *= SERVO_MIN_SIDE_WEIGHT * (1+0.5*(-1-s_pd_offset)/29);  //左偏增大
    else if(s_pd_offset > 1.0f)
        s_pd_offset *= SERVO_MAX_SIDE_WEIGHT * (1+0.5*(-1+s_pd_offset)/35);   //右偏增大
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

/* 左轮独立PI更新：每个控制器状态只从自己的pi结构体读写，无共享变量。 */
int16_t PI_Update_Left(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd)
{
    int16_t abs_err = (pos_err < 0) ? (int16_t)(-pos_err) : (int16_t)pos_err;
    if (abs_err > 100) abs_err = 100;

    int16_t target;
    if (abs_err <= 2)
        target = str_spd;
    else
        target = pi->MinSpeed + (int16_t)((int32_t)(str_spd - pi->MinSpeed)
                                          * (200 - abs_err) / 200);
    target += pi->TargetBias;
    pi->TargetSpeed = target;

    int16_t err = target - act_spd;
    float   inc = pi->Kp * (float)(err - pi->LastSpdErr)
                + pi->Ki * (float)err;
    pi->Output += inc;
    pi->LastSpdErr = err;

    /* 按实车调试要求移除正负200软件限幅，保留完整PI累计输出。 */
    return (int16_t)pi->Output;
}

/* 右轮独立PI更新：每个控制器状态只从自己的pi结构体读写，无共享变量。 */
int16_t PI_Update_Right(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd)
{
    int16_t abs_err = (pos_err < 0) ? (int16_t)(-pos_err) : (int16_t)pos_err;
    if (abs_err > 100) abs_err = 100;

    int16_t target;
    if (abs_err <= 2)
        target = str_spd;
    else
        target = pi->MinSpeed + (int16_t)((int32_t)(str_spd - pi->MinSpeed)
                                          * (200 - abs_err) / 200);
    target += pi->TargetBias;
    pi->TargetSpeed = target;

    int16_t err = target - act_spd;
    float   inc = pi->Kp * (float)(err - pi->LastSpdErr)
                + pi->Ki * (float)err;
    pi->Output += inc;
    pi->LastSpdErr = err;

    /* 按实车调试要求移除正负200软件限幅，保留完整PI累计输出。 */
    return (int16_t)pi->Output;
}

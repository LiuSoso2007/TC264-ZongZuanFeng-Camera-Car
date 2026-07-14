/******************************************************************************
 * PID.c - 舵机PD控制器 + 电机PI控制器
 *
 * PD部分: 位置环(图像Err→目标角速度) + 角速度环(IMU实际角速度→舵机角度)
 *   target_yaw = Kp * Err                     -- 位置偏差→目标角速度
 *   yaw_error  = target_yaw - IMU_Yaw         -- 角速度偏差
 *   s_pd_out   = 50.0f + Kd * yaw_error       -- 中位 + 角速度修正
 *   输出限幅: 9~140度
 *
 * PI部分: 增量式PI (电机速度闭环)
 *   根据位置偏差动态调节目标速度 (弯道减速)
 *   增量公式: inc = Kp*(err[n] - err[n-1]) + Ki*err[n]
 *   输出限幅: PI_OUT_MIN ~ PI_OUT_MAX
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- 舵机PD: 位置(Err→目标角速度) + 角速度(实际角速度→舵角) ---- */
static float s_pd_out = 0.0f;

/*
 * PD_Update - 舵机PD控制更新
 * Kp: 位置环比例系数 (Err→目标角速度)
 * Kd: 角速度环微分系数 (角速度偏差→舵角修正)
 *
 * 逻辑:
 *   1. 图像偏差Err乘以Kp得到目标角速度
 *   2. 目标角速度减去IMU实测角速度得到角速度偏差
 *   3. 中位50度 + Kd*角速度偏差 = 最终舵角
 *   4. 限幅9~140度 (物理安全范围)
 *   5. 偏差小于5时不打角, 锁定中位50度
 */
void PD_Update(float Kp, float Kd)
{
    float target_yaw;               /* 目标角速度 (度/s) */
    float yaw_error;                /* 角速度偏差 */

    target_yaw = Kp * Err;                         /* 位置环: 偏差→目标角速度 */
    yaw_error  = target_yaw - IMU_Yaw;             /* 角速度环: 目标减实测 */
    s_pd_out   = 50.0f + Kd * yaw_error;           /* 中位50度 + 角速度修正 */

    /* 输出限幅: 舵机物理范围 0~180度, 实际安全范围 9~140度 */
    if (s_pd_out > 140.0f) s_pd_out = 140.0f;
    if (s_pd_out < 9.0f)   s_pd_out = 9.0f;

    /* 偏差小于5度时锁定中位, 避免直道抖动 */
    if (Err >= -5.0f && Err <= 5.0f)
        Servo_SetAngleDeg(50);
    else
        Servo_SetAngleDeg((uint8_t)s_pd_out);
}

/* ---- 电机PI: 增量式PI (速度闭环) ---- */

/*
 * PI_Init - PI控制器初始化
 * pi:        PI控制结构体指针
 * kp, ki:    PID参数 (可调)
 * min_speed: 最小目标速度 (用于弯道减速下限)
 */
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

/*
 * PI_Update - 增量式PI速度控制
 * pi:      PI控制结构体
 * pos_err: 位置偏差 (用于动态调速)
 * act_spd: 实际速度 (编码器读数)
 * str_spd: 直线基准速度
 *
 * 返回: PWM占空比 (-100 ~ +100, 负值=反转)
 *
 * 逻辑:
 *   1. 根据位置偏差动态调整目标速度 (偏差大→减速)
 *   2. 叠加TargetBias (差速/弯道偏置)
 *   3. 增量PI计算: inc = Kp*(err[n]-err[n-1]) + Ki*err[n]
 *   4. 输出限幅
 */
int8_t PI_Update(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd)
{
    int16_t abs_err;
    if (pos_err < 0)
        abs_err = (int16_t)(-pos_err);
    else
        abs_err = (int16_t)pos_err;
    if (abs_err > 100) abs_err = 100;

    /* 根据位置偏差动态调速: 偏差越大, 目标速度越低 */
    int16_t target;
    if (abs_err <= 5)
        target = str_spd;                         /* 偏差<5: 全速 */
    else
        target = pi->MinSpeed + (int16_t)((int32_t)(str_spd - pi->MinSpeed)
                                          * (100 - abs_err) / 100);
    target += pi->TargetBias;                     /* 叠加弯道差速偏置 */
    pi->TargetSpeed = target;

    /* 增量式PI计算 */
    int16_t err = target - act_spd;
    float   inc = pi->Kp * (float)(err - pi->LastSpdErr)
                + pi->Ki * (float)err;
    pi->Output += inc;
    pi->LastSpdErr = err;

    /* 输出限幅: 确保PWM占空比在合法范围 */
    if (pi->Output > PI_OUT_MAX) pi->Output = PI_OUT_MAX;
    if (pi->Output < PI_OUT_MIN) pi->Output = PI_OUT_MIN;
    return (int8_t)pi->Output;
}
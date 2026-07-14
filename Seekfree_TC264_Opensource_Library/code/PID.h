#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/*
 * PID.h --- 舵机PD + 电机PI 控制器接口
 *
 * PD (舵机): 位置环(Err→角速度) + 角速度环(IMU_Yaw→舵角)
 *   PD_Update(Kp, Kd) -- 每帧调用, 输出舵角到Servo
 *
 * PI (电机): 增量式速度闭环
 *   PI_Init()         -- 初始化参数
 *   PI_Update()       -- 每帧调用, 返回PWM占空比
 */

extern volatile float Err;       /* 图像偏差 (CPU0生产, CPU1消费) */
extern float IMU_Yaw;            /* IMU实测Z轴角速度 (度/s) */

/* ---- PI 控制参数 ---- */
#define PI_KP          0.55f     /* 比例系数 */
#define PI_KI          0.2f      /* 积分系数 */
#define CURVE_SPEED    0         /* 弯道最低速度 */
#define PI_OUT_MIN    -100       /* PWM输出下限 (反转) */
#define PI_OUT_MAX     100       /* PWM输出上限 (正转) */

/*
 * PD_Update - 舵机PD控制 (位置+角速度双环)
 * Kp: Err→目标角速度的比例系数
 * Kd: 角速度偏差→舵角修正的微分系数
 */
void PD_Update(float Kp, float Kd);

/* PI控制结构体 */
typedef struct {
    float   Kp;             /* 比例系数 */
    float   Ki;             /* 积分系数 */
    int16_t MinSpeed;       /* 弯道最低速度 */
    int16_t LastSpdErr;     /* 上次速度偏差 (增量PI需要) */
    float   Output;         /* 当前PI输出 */
    int16_t TargetSpeed;    /* 当前目标速度 (动态调速结果) */
    int16_t TargetBias;     /* 目标速度偏置 (弯道差速) */
} PI_t;

void  PI_Init(PI_t *pi, float kp, float ki, int16_t min_speed);
int8_t PI_Update(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd);

#endif
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#define PI_OUT_MIN    -200
#define PI_OUT_MAX     200

/* PD位置式 -- 舵机 */
void PD_Update(float Kp, float Kd, float err);

/* PI增量式 -- 电机 */
typedef struct {
    float   Kp;
    float   Ki;
    int16_t MinSpeed;
    int16_t LastSpdErr;
    float   Output;
    int16_t TargetSpeed;
    int16_t TargetBias;
} PI_t;

void PI_Init(PI_t *pi, float kp, float ki, int16_t min_speed);

/* 左右轮独立的PI更新入口，语义一致，各自独立，便于单独加专属补偿/限幅。 */
int16_t PI_Update_Left(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd);
int16_t PI_Update_Right(PI_t *pi, float pos_err, int16_t act_spd, int16_t str_spd);

#endif

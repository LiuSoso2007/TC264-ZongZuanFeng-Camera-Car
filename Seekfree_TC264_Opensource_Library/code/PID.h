#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/* 首次闭环验证保持原40%开环幅度，并预留正反向制动范围。 */
#define PI_OUT_MIN    (-40)
#define PI_OUT_MAX      40
#if (PI_OUT_MIN < -100) || (PI_OUT_MAX > 100) || (PI_OUT_MIN >= PI_OUT_MAX)
#error "PI output limits must stay within motor PWM range"
#endif

/* PD位置式 -- 舵机 */
void PD_Update(float Kp, float Kd, float err);

/* PI增量式 -- 电机 */
typedef struct {
    float   Kp;
    float   Ki;
    int16_t MinSpeed;
    int32_t LastSpdErr;
    float   Output;
    int16_t TargetSpeed;
    int16_t TargetBias;
} PI_t;

void PI_Init(PI_t *pi, float kp, float ki, int16_t min_speed);
int8_t PI_Update(PI_t *pi, float pos_err, int16_t act_spd, float base_target_pulses);

#endif

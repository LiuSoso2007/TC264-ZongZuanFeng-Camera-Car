/******************************************************************************
 * PID.c - PD舵机 + PI电机
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- PD ---- */
#define PD_DELTA_ERR_DEAD_ZONE  1.0f  /* 忽略中心线整数化造成的单像素差分抖动。 */
#define PD_D_FADE_ERR           50.0f /* Err越接近该值，模糊D越接近零。 */
#define PD_D_OFFSET_LIMIT       4.0f  /* D仅作瞬态修正，单次最多贡献正负4度。 */
static float s_pd_last_err = 0.0f;

void PD_Update(float Kp, float Kd, float err)
{
    float delta_err;
    float abs_err;
    float d_gain;
    float d_offset;
    float servo_out;

    delta_err = err - s_pd_last_err;
    s_pd_last_err = err;
    if (delta_err > PD_DELTA_ERR_DEAD_ZONE)
        delta_err -= PD_DELTA_ERR_DEAD_ZONE;
    else if (delta_err < -PD_DELTA_ERR_DEAD_ZONE)
        delta_err += PD_DELTA_ERR_DEAD_ZONE;
    else
        delta_err = 0.0f;

    /* 直道附近增强D抑制高速摆动，进入弯道后平滑减弱D，避免阻碍持续打角。 */
    abs_err = (err >= 0.0f) ? err : -err;
    if (abs_err < PD_D_FADE_ERR)
        d_gain = Kd * (PD_D_FADE_ERR - abs_err) / PD_D_FADE_ERR;
    else
        d_gain = 0.0f;

    d_offset = d_gain * delta_err;
    if (d_offset > PD_D_OFFSET_LIMIT)  d_offset = PD_D_OFFSET_LIMIT;
    if (d_offset < -PD_D_OFFSET_LIMIT) d_offset = -PD_D_OFFSET_LIMIT;

    /* P和模糊D统一使用同一套对称公式，机械差异只由最终安全限幅兜底。 */
    servo_out = (float)SERVO_CENTER_ANGLE + Kp * err + d_offset;
    if (servo_out > (float)SERVO_MAX_ANGLE) servo_out = (float)SERVO_MAX_ANGLE;
    if (servo_out < (float)SERVO_MIN_ANGLE) servo_out = (float)SERVO_MIN_ANGLE;
    Servo_SetAngleDeg((uint8_t)(servo_out + 0.5f));
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
    if (abs_err <= 2)
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

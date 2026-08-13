/******************************************************************************
 * PID.c - PD舵机 + PI电机
 ******************************************************************************/
#include "PID.h"
#include "Servo.h"

/* ---- PD ---- */
#define PD_ERR_DEAD_ZONE 0.0f  /* Err死区边界，范围内舵机回中。 */
#define SERVO_MIN_SIDE_WEIGHT  0.85f  /* 100方向基础权重，负向误差越大时再按比例增强。 */
#define SERVO_MAX_SIDE_WEIGHT  0.85f  /* 175方向固定权重，以1为归一化基准。 */
#define PD_DELTA_ERR_DEAD_ZONE  1.0f  /* 忽略中心线整数化造成的单像素差分抖动。 */
#define PD_D_OFFSET_LIMIT       4.0f  /* D仅作瞬态修正，单次最多贡献正负4度。 */
static float   s_pd_out = 0.0f, s_pd_offset = 0.0f;
static float   s_pd_p_offset = 0.0f, s_pd_d_offset = 0.0f;
static float   s_pd_err0 = 0.0f, s_pd_err1 = 0.0f;

void PD_Update(float Kp, float Kd, float err)
{
    float delta_err;
    float d_gain;

    s_pd_err1 = s_pd_err0;
    s_pd_err0 = err;

    /* P使用原始Err，保持稳态转向响应；D只处理帧间变化，避免一并进入物理补偿曲线。 */
    s_pd_p_offset = Kp * s_pd_err0;
    delta_err = s_pd_err0 - s_pd_err1;
    if (delta_err > PD_DELTA_ERR_DEAD_ZONE)
        delta_err -= PD_DELTA_ERR_DEAD_ZONE;
    else if (delta_err < -PD_DELTA_ERR_DEAD_ZONE)
        delta_err += PD_DELTA_ERR_DEAD_ZONE;
    else
        delta_err = 0.0f;

    /* 保留原有随Err增大而衰减的Kd规律，但D输出独立限幅，不再触发非线性放大。 */
    if (s_pd_err0 > 0.0f && s_pd_err0 < 50.0f)
        d_gain = Kd * (50.0f - s_pd_err0) / 50.0f;
    else if (s_pd_err0 < 0.0f && s_pd_err0 > -50.0f)
        d_gain = Kd * (50.0f + s_pd_err0) / 50.0f;
    else if (s_pd_err0 == 0.0f)
        d_gain = Kd;  /* S弯换向过零时保留D预判，避免舵机瞬间卸力回中。 */
    else
        d_gain = 0.0f;

    s_pd_d_offset = d_gain * delta_err;
    if (s_pd_d_offset > PD_D_OFFSET_LIMIT)  s_pd_d_offset = PD_D_OFFSET_LIMIT;
    if (s_pd_d_offset < -PD_D_OFFSET_LIMIT) s_pd_d_offset = -PD_D_OFFSET_LIMIT;

    /* 非线性曲线用于补偿舵机左右物理不对称，只作用于稳态P偏移。 */
    if (s_pd_p_offset < -8.0f)
        s_pd_p_offset *= SERVO_MIN_SIDE_WEIGHT * (1.0f + (-8.0f - s_pd_p_offset) / 33.0f);
    else if (s_pd_p_offset > 8.0f)
        s_pd_p_offset *= SERVO_MAX_SIDE_WEIGHT * (1.0f + (-8.0f + s_pd_p_offset) / 30.0f);
    else
        s_pd_p_offset *= SERVO_MAX_SIDE_WEIGHT;

    s_pd_offset = s_pd_p_offset + s_pd_d_offset;

    s_pd_out = (float)SERVO_CENTER_ANGLE + s_pd_offset;
    if (s_pd_out > (float)SERVO_MAX_ANGLE) s_pd_out = (float)SERVO_MAX_ANGLE;
    if (s_pd_out < (float)SERVO_MIN_ANGLE) s_pd_out = (float)SERVO_MIN_ANGLE;

    /* 仅在P、D都无修正时回中，过零帧允许受限的D预判继续作用。 */
    if (err >= -PD_ERR_DEAD_ZONE && err <= PD_ERR_DEAD_ZONE && s_pd_d_offset == 0.0f)
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

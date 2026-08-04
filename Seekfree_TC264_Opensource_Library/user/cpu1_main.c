/**
 * cpu1_main.c  ---  CPU1: Real-time Control + LCD + Keys + IMU
 *
 * Dual-core architecture:
 *   CPU0: System init, Camera (MT9V03X), Image processing -> Err & ImageFlags
 *   CPU1: Encoder, Servo PD, Motor PI speed loop (10ms)
 *         + IPS200 LCD display + Key scanning + IMU gyroscope
 *
 * Control period: CCU61_CH0 PIT 10ms (ISR in isr.c)
 *
 * Cross-core shared (from cpu0_main.c):
 *   Read: Err (track deviation), ImageStatus / ImageFlag (image element flags)
 */
// CPU1 local variables (this core only):
//   Target speed g_StraightSpeed, servo mid g_CalibAngle,
//   encoder values g_EncLeft / g_EncRight
//   Future: adjusted by keys / IMU

#include "zf_common_headfile.h"
#include "IPS200.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

/* PID_Flag: set by isr.c cc61_pit_ch0_isr, cleared here */
volatile uint8_t PID_Flag = 0;

/* CPU1本地变量, CPU0只读用于显示, 无需互斥锁 */
volatile int16_t EncLeft  = 0;
volatile int16_t EncRight = 0;

#pragma section all "cpu1_dsram"   /* ---- CPU1 private variables ---- */

/* ---- CPU1 local parameters (future: key / IMU control) ---- */
static int8_t   StraightSpeed = 30;
static int16_t  EncCount        = 0;

/* 进环保留速度百分比：50表示保留原速度50%，数值越大越快，越小越慢。 */
#define RING_ENTRY_SPEED_PERCENT 50
#if RING_ENTRY_SPEED_PERCENT < 0 || RING_ENTRY_SPEED_PERCENT > 100
#error "RING_ENTRY_SPEED_PERCENT must be between 0 and 100"
#endif

/* ---- PI参数 ---- */
#define PI_KP          0.4f
#define PI_KI          0.02f
#define CURVE_SPEED    0
/* ---- PD参数 ---- */
#define PD_KP          0.85f
#define PD_KD          1.15f

static PI_t s_PI_Left, s_PI_Right;   /* Left/Right motor PI controllers */

int core1_main(void)
{
    /* ---- Core1 init ---- */
    disable_Watchdog();
    interrupt_global_enable(0);

    int16_t  enc_left = 0, enc_right = 0;
    int8_t   pwm_left,  pwm_right;
    int16_t  motor_speed;
    float    position_err = 0.0f;
    float    new_position_err;


    /* ---- Peripheral init (CPU1 side) ---- */
    Key_Init();                          // 4-key button (placeholder, function TBD)
    Encoder_Init();                      // Quadrature encoder TIM6(L), TIM4(R)
    Motor_Init();                        // Motor dual-pole PWM (ATOM0)
    Servo_Init();                        // Servo 50Hz PWM (ATOM0)

    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);

    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);


    /* ---- Key scan PIT: 5ms (CPU1 PIT) ---- */
    pit_ms_init(CCU60_CH1, 5);

    /* ---- Control PIT timer: 10ms, ISR handled by CPU1 ---- */
    pit_ms_init(CCU61_CH0, 10);

    /* ---- IMU gyro init (future port) ---- */
    // TODO: IMU_Init();  // ICM20602 / IMU660RC

    /* ---- Wait for CPU0 ready ---- */
    cpu_wait_event_ready();

    while (TRUE)
    {


        /* ---- Key scanning (placeholder, function TBD) ---- */
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;   // Not yet bound to any function
            // TODO: Bind keys to servo angle, speed params
        }

        /* ---- IMU data processing (future port) ---- */
        // TODO: icm20602_get_gyro(); Yaw_Now = get_zangle(gyro); kalmanFilter();

        /* ---- Wait for control period ---- */
        if (!PID_Flag)
        {
            continue;
        }
        PID_Flag = 0;

        /* ---- Encoder acquisition ---- */

        EncCount ++;
        if(EncCount >= 8)
        {
             EncCount = 0;
             enc_left  = Encoder_Get_Left();
             enc_right = Encoder_Get_Right();
        }

        EncLeft  = enc_left;
        EncRight = enc_right;

        /* ---- Track error (CPU0图像输出，无新帧时保持上一份快照) ---- */
        uint8_t has_new_err = 0U;
        uint8_t Err_abs = 0U;
        if (Shared_TakeErr(&new_position_err))
        {
            position_err = new_position_err;
            has_new_err = 1U;
        }
        if(position_err>0)Err_abs=position_err;
        if(position_err<0)Err_abs=-position_err;

        /* CPU0识别到斑马线并锁定后, 依次置零PWM和PI偏置, 然后设置舵机中位停车。 */
        if (StopRequest != 0U)
        {
            pwm_left = 0;
            pwm_right = 0;
            s_PI_Left.TargetBias = 0;
            s_PI_Right.TargetBias = 0;
            Motor_SetLeftPWM(0);
            Motor_SetRightPWM(0);
            Servo_SetAngleDeg(SERVO_CENTER_ANGLE);
            continue;
        }

        /* ---- Differential compensation: adjust L/R target speed by error ---- */
        /*if      (position_err >   7.0f && position_err <  15.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-StraightSpeed * 0.0f);
            s_PI_Right.TargetBias = (int16_t)( StraightSpeed * 0.0f);
        }
        else if (position_err >= 15.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-StraightSpeed * 0.0f);
            s_PI_Right.TargetBias = (int16_t)( StraightSpeed * 0.0f);
        }
        else if (position_err <  -7.0f && position_err > -15.0f) {
            s_PI_Left.TargetBias  = (int16_t)( StraightSpeed * 0.0f);
            s_PI_Right.TargetBias = (int16_t)(-StraightSpeed * 0.0f);
        }
        else if (position_err <= -15.0f) {
            s_PI_Left.TargetBias  = (int16_t)( StraightSpeed * 0.0f);
            s_PI_Right.TargetBias = (int16_t)(-StraightSpeed * 0.0f);
        }
        else {
            s_PI_Left.TargetBias  = 0;
            s_PI_Right.TargetBias = 0;
        }*/

        /* ---- Speed PI closed-loop ---- */
        pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  StraightSpeed);
        pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, StraightSpeed);

        motor_speed = (int16_t)((float)StraightSpeed - 0.2f * (float)Err_abs);
        if (RingEntrySlowdown != 0U)
        {
            motor_speed = (int16_t)(motor_speed * RING_ENTRY_SPEED_PERCENT / 100);
        }

        /* 双向输出统一限制在-100~100，防止调参后越过电机PWM边界。 */
        if (motor_speed > 100)  motor_speed = 100;
        if (motor_speed < -100) motor_speed = -100;
        Motor_SetLeftPWM((int8_t)motor_speed);
        Motor_SetRightPWM((int8_t)motor_speed);

        /* 每个图像Err只执行一次PD，避免10ms控制周期重复覆盖微分输出。 */
        if (has_new_err != 0U)
        {
            PD_Update(PD_KP, PD_KD, position_err);
        }

    }
}

#pragma section all restore

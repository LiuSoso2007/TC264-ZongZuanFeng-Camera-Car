/**
 * CPU1: 运动控制
 *
 * CPU0: 图像采集与处理，输出赛道偏差Err与元素标志
 * CPU1: 编码器、舵机PD、电机PI速度环，控制周期10ms
 * 控制定时器: CCU61_CH0 PIT 10ms（中断在isr.c中）
 */

#include "zf_common_headfile.h"
#include "IPS200.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

/* PID_Flag：由isr.c中cc61_pit_ch0中断置1，本函数处理后清零 */
volatile uint8_t PID_Flag = 0;

/* CPU1本地变量：CPU0只读用于显示，无需互斥锁 */
volatile int16_t EncLeft  = 0;
volatile int16_t EncRight = 0;

#pragma section all "cpu1_dsram"   /* CPU1私有变量放入DSRAM段 */

/* CPU1本地参数（后续可用按键/IMU调整） */
static int8_t   StraightSpeed = 40;
static int16_t  EncCount        = 0;

/* 进环保留速度百分比：60表示保留原速度60%，数值越大越快，越小越慢。 */
#define RING_ENTRY_SPEED_PERCENT 60
#if RING_ENTRY_SPEED_PERCENT < 0 || RING_ENTRY_SPEED_PERCENT > 100
#error "RING_ENTRY_SPEED_PERCENT must be between 0 and 100"
#endif

/* PI参数 */
#define PI_KP          0.4f
#define PI_KI          0.02f
#define CURVE_SPEED    0

/* PD参数 */
#define PD_KP          0.85f
#define PD_KD          10.0f

/* 左右电机PI控制器 */
static PI_t s_PI_Left, s_PI_Right;

/* CPU1入口函数 */
int core1_main(void)
{
    /* CPU1初始化：关闭看门狗并开总中断 */
    disable_Watchdog();
    interrupt_global_enable(0);

    int16_t  enc_left = 0, enc_right = 0;
    int8_t   pwm_left,  pwm_right;
    int16_t  motor_speed;
    float    position_err = 0.0f;
    float    new_position_err;
    uint8_t  ring_entry_slowdown = 0U;
    uint8_t  new_ring_entry_slowdown;

    /* CPU1外设初始化 */
    Key_Init();                          /* 四键按键（功能预留） */
    Encoder_Init();                      /* 编码器：左TIM6/右TIM4 */
    Motor_Init();                        /* 电机双极性PWM(ATOM0) */
    Servo_Init();                        /* 舵机50Hz PWM(ATOM0) */

    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);

    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* 按键扫描定时器：5ms（CPU1 PIT） */
    pit_ms_init(CCU60_CH1, 5);

    /* 控制周期定时器：10ms，中断由CPU1处理 */
    pit_ms_init(CCU61_CH0, 10);

    /* 等待CPU0就绪 */
    cpu_wait_event_ready();

    while (TRUE)
    {
        {
            /* 按键扫描（暂未绑定功能） */
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;
        }

        /* 等待控制周期 */
        if (!PID_Flag)
        {
            continue;
        }
        PID_Flag = 0;

        /* 编码器读取：每8个控制周期采样一次 */
        EncCount ++;
        if(EncCount >= 8)
        {
             EncCount = 0;
             enc_left  = Encoder_Get_Left();
             enc_right = Encoder_Get_Right();
        }

        EncLeft  = enc_left;
        EncRight = enc_right;

        /* 赛道误差：CPU0图像输出，无新帧时保持上一份快照 */
        uint8_t has_new_err = 0U;
        uint8_t Err_abs = 0U;
        if (Shared_TakeErr(&new_position_err, &new_ring_entry_slowdown))
        {
            position_err = new_position_err;
            ring_entry_slowdown = new_ring_entry_slowdown;
            has_new_err = 1U;
        }
        if(position_err>0)Err_abs=position_err;
        if(position_err<0)Err_abs=-position_err;

        /* CPU0识别到斑马线并锁定后，依次置零PWM和PI偏置，然后设置舵机中位停车。 */
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

        /* 速度PI闭环 */
        pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  StraightSpeed);
        pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, StraightSpeed);

        motor_speed = (int16_t)((float)StraightSpeed - 0.3f * (float)Err_abs);
        /* 进入圆环时按保留比例降速 */
        if (ring_entry_slowdown != 0U)
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

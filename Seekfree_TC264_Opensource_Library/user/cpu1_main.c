/**
 * CPU1: 运动控制
 *
 * CPU0: 图像采集与处理，输出赛道偏差Err与元素标志
 * CPU1: 10ms调度舵机PD，电机PI按40ms编码器新样本更新
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

/* 电机PI每4个10ms调度周期更新；目标速度统一使用编码器脉冲/秒。 */
#define MOTOR_CONTROL_PERIOD_MS   10U
#define MOTOR_PI_SAMPLE_TICKS      4U
#define MOTOR_PI_SAMPLE_PERIOD_MS (MOTOR_CONTROL_PERIOD_MS * MOTOR_PI_SAMPLE_TICKS)
#if (MOTOR_PI_SAMPLE_TICKS == 0U) || (MOTOR_PI_SAMPLE_PERIOD_MS > 1000U)
#error "Motor PI sample period is invalid"
#endif
/* 直道目标速度（编码器脉冲/秒）：增大更快，减小更慢；500对应每40ms目标20脉冲，不能按PWM百分比填写。 */
static int16_t StraightSpeedPps = 500;
static uint8_t EncCount = 0U;

/* 进环保留速度百分比：60表示保留原速度60%，数值越大越快，越小越慢。 */
#define RING_ENTRY_SPEED_PERCENT 60
#if RING_ENTRY_SPEED_PERCENT < 0 || RING_ENTRY_SPEED_PERCENT > 100
#error "RING_ENTRY_SPEED_PERCENT must be between 0 and 100"
#endif

/* PI参数 */
#define PI_KP          0.8f
#define PI_KI          0.02f
#define CURVE_SPEED    0

/* PD参数 */
#define PD_KP          0.85f
#define PD_KD          10.8f

/* 左右电机PI控制器 */
static PI_t s_PI_Left, s_PI_Right;

/* CPU1入口函数 */
int core1_main(void)
{
    /* CPU1初始化：关闭看门狗并开总中断 */
    disable_Watchdog();
    interrupt_global_enable(0);

    int16_t  enc_left = 0, enc_right = 0;
    int8_t   pwm_left, pwm_right;
    float    position_err = 0.0f;
    float    new_position_err;
    uint8_t  ring_entry_slowdown = 0U;
    uint8_t  new_ring_entry_slowdown;
    uint8_t  encoder_sample_ready = 0U;

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
    pit_ms_init(CCU61_CH0, MOTOR_CONTROL_PERIOD_MS);

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

        /* 编码器每40ms产生一个新样本，PI也只在此时更新一次。 */
        encoder_sample_ready = 0U;
        EncCount ++;
        if(EncCount >= MOTOR_PI_SAMPLE_TICKS)
        {
             EncCount = 0;
             enc_left  = Encoder_Get_Left();
             enc_right = Encoder_Get_Right();
             encoder_sample_ready = 1U;
        }

        EncLeft  = enc_left;
        EncRight = enc_right;

        /* 赛道误差：CPU0图像输出，无新帧时保持上一份快照 */
        uint8_t has_new_err = 0U;
        if (Shared_TakeErr(&new_position_err, &new_ring_entry_slowdown))
        {
            position_err = new_position_err;
            ring_entry_slowdown = new_ring_entry_slowdown;
            has_new_err = 1U;
        }

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

        /* 新编码器样本到达时更新左右独立PI，其余周期保持上次PWM。 */
        if (encoder_sample_ready != 0U)
        {
            int16_t target_speed = (int16_t)((int32_t)StraightSpeedPps
                                   * (int32_t)MOTOR_PI_SAMPLE_PERIOD_MS / 1000);

            /* 进环减速作用于目标速度，避免闭环把减速量重新补回来。 */
            if (ring_entry_slowdown != 0U)
            {
                target_speed = (int16_t)(target_speed * RING_ENTRY_SPEED_PERCENT / 100);
            }

            pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  target_speed);
            pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, target_speed);
            Motor_SetLeftPWM(pwm_left);
            Motor_SetRightPWM(pwm_right);
        }

        /* 每个图像Err只执行一次PD，避免10ms控制周期重复覆盖微分输出。 */
        if (has_new_err != 0U)
        {
            PD_Update(PD_KP, PD_KD, position_err);
        }

    }
}

#pragma section all restore

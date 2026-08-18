/**
 * CPU1: 运动控制
 *
 * CPU0: 图像采集与处理，通过邮箱发布赛道偏差和进环减速标志，并可发出停车请求
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

/* CPU1编码器采样分频计数器。 */
static int16_t  EncCount        = 0;

/* 进环保留速度百分比：80表示保留原目标速度的80%，数值越大越快，越小越慢。 */
#define RING_ENTRY_SPEED_PERCENT 80
#if RING_ENTRY_SPEED_PERCENT < 0 || RING_ENTRY_SPEED_PERCENT > 100
#error "RING_ENTRY_SPEED_PERCENT must be between 0 and 100"
#endif

/* PI参数 */
#define PI_KP          0.4f
#define PI_KI          0.04f
#define INIT_SPEED     0
#define STRAIGHT_SPEED 80

/* VOFA电机PI临时调参：1=固定双轮目标并忽略赛道停车/差速，0=恢复正常赛道控制。 */
#define VOFA_PI_TUNING_MODE 1

/* PD参数 */
#define PD_KP          1.0f
#define PD_KD          0.13f

/* 左右电机PI控制器 */
static PI_t s_PI_Left, s_PI_Right;

/* VOFA FireWater文本帧缓存：左编码器、左目标、右编码器、右目标。 */
static int8 s_vofa_frame[48];

/* CPU1入口函数 */
int core1_main(void)
{
    /* CPU1初始化：关闭看门狗并开总中断 */
    disable_Watchdog();
    interrupt_global_enable(0);

    int16_t  enc_left = 0, enc_right = 0;
    int8_t   motor_left,  motor_right;
    int8_t   LeftSpeed = STRAIGHT_SPEED, RightSpeed = STRAIGHT_SPEED;
    float    position_err = 0.0f;
#if !VOFA_PI_TUNING_MODE
    float    new_position_err;
    uint8_t  ring_entry_slowdown = 0U;
    uint8_t  new_ring_entry_slowdown;
#endif
    uint8_t  encoder_updated;

    /* CPU1外设初始化 */
    Key_Init();                          /* 四键按键（功能预留） */
    Encoder_Init();                      /* 编码器：左TIM6/右TIM4 */
    Motor_Init();                        /* 电机双极性PWM(ATOM0) */
    Servo_Init();                        /* 舵机50Hz PWM(ATOM0) */

    PI_Init(&s_PI_Left,  PI_KP, PI_KI, INIT_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, INIT_SPEED);

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
        encoder_updated = 0U;
        EncCount ++;
        if(EncCount >= 8)
        {
             EncCount = 0;
             enc_left  = Encoder_Get_Left();
             enc_right = Encoder_Get_Right();
             encoder_updated = 1U;
        }

        EncLeft  = enc_left;
        EncRight = enc_right;

#if VOFA_PI_TUNING_MODE
        /* 临时速度环调参：固定目标速度，屏蔽视觉差速、圆环减速和停车请求。 */
        position_err = 0.0f;
        LeftSpeed = STRAIGHT_SPEED;
        RightSpeed = STRAIGHT_SPEED;
#else
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

        /* CPU0锁存斑马线或底部全黑停车请求后，置零PWM和PI偏置，并将舵机回中。 */
        if (StopRequest != 0U)
        {
            motor_left = 0;
            motor_right = 0;
            s_PI_Left.TargetBias = 0;
            s_PI_Right.TargetBias = 0;
            Motor_SetLeftPWM(0);
            Motor_SetRightPWM(0);
            Servo_SetAngleDeg(SERVO_CENTER_ANGLE);
            continue;
        }

        /* 根据赛道误差生成左右目标速度，再执行速度PI闭环。 */

        /* 左右轮差速：Err越大，对侧轮减速越多。
           用浮点乘法避免整数除法使 (Err_abs-2)/50 在小Err时恒为0。 */
        LeftSpeed  = STRAIGHT_SPEED;
        RightSpeed = STRAIGHT_SPEED;
        if (position_err >= 2.0f)
            RightSpeed = (int16_t)((float)STRAIGHT_SPEED
                                   * (1.0f - ((float)Err_abs - 2.0f) / 70.0f));
        else if (position_err <= -2.0f)
            LeftSpeed  = (int16_t)((float)STRAIGHT_SPEED
                                   * (1.0f - ((float)Err_abs - 2.0f) / 70.0f));

        /* 圆环减速：对速度值打折（原代码误用了上一帧motor_*，会使目标速度失真）。 */
        if (ring_entry_slowdown != 0U)
        {
            LeftSpeed  = (int16_t)((float)LeftSpeed  * (float)RING_ENTRY_SPEED_PERCENT / 100.0f);
            RightSpeed = (int16_t)((float)RightSpeed * (float)RING_ENTRY_SPEED_PERCENT / 100.0f);
        }
#endif

        /* 左右轮独立PI更新，各用各的结构体，互不影响。 */
        motor_left  = PI_Update_Left (&s_PI_Left,  position_err, enc_left,  LeftSpeed);
        motor_right = PI_Update_Right(&s_PI_Right, position_err, enc_right, RightSpeed);

        /* 双向输出统一限制在-100~100，左右轮对称。 */
        if (motor_left  >  100) motor_left  =  100;
        if (motor_left  < -100) motor_left  = -100;
        if (motor_right >  100) motor_right =  100;
        if (motor_right < -100) motor_right = -100;
        Motor_SetLeftPWM ((int8_t)motor_left);
        Motor_SetRightPWM((int8_t)motor_right);

        /* 仅在新编码器样本到达时发送，避免串口输出占用10ms控制周期。
           VOFA选择FireWater协议，通道顺序为左编码器、左目标、右编码器、右目标。 */
        if (encoder_updated != 0U)
        {
            uint32 vofa_len = zf_sprintf(s_vofa_frame, (const int8 *)"%d,%d,%d,%d\r\n",
                                         (int32)enc_left, (int32)s_PI_Left.TargetSpeed,
                                         (int32)enc_right, (int32)s_PI_Right.TargetSpeed);
            debug_send_buffer((const uint8 *)s_vofa_frame, vofa_len);
        }

#if !VOFA_PI_TUNING_MODE
        /* 每个图像Err只执行一次PD，避免10ms控制周期重复覆盖微分输出。 */
        if (has_new_err != 0U)
        {
            PD_Update(PD_KP, PD_KD, position_err);
        }
#endif

    }
}

#pragma section all restore

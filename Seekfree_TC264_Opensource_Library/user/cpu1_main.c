/**
 * cpu1_main.c  ---  CPU1: 运动控制(舵机PD + 差速PI电机)
 *
 * 双核架构:
 *   CPU0: 摄像头图像采集 + OTSU二值化 + 图像处理 + IPS200全屏显示
 *   CPU1: 舵机PD实时打角 + 编码器 + 电机PI差速控制 + 按键扫描
 *
 * 控制周期: 10ms (CCU61_CH0 PIT定时中断, isr.c中置PID_Flag)
 * 按键扫描: 5ms  (CCU60_CH1 PIT定时中断, isr.c中调用Key_Tick)
 *
 * 核间通信: Err (volatile, cpu0_main.c定义, Shared.h声明)
 *   Err: 图像偏差 (CPU0计算, CPU1读取, 用于舵机PD和差速PI)
 *
 * IPS200: 完全由CPU0独占管理 (显示原始图/二值图/阈值/元素)
 */

#include "zf_common_headfile.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile uint8_t PID_Flag = 0;       /* PID控制定时标志 (isr.c 10ms中断置位) */

#pragma section all "cpu1_dsram"

/* ---- CPU1本地变量 ---- */
static int8_t   g_StraightSpeed = 50;    /* 直线基准速度 */
static int16_t  g_EncLeft       = 0;     /* 左编码器累积值 (每周期) */
static int16_t  g_EncRight      = 0;     /* 右编码器累积值 (每周期) */
static int16_t  EncCount        = 0;     /* 编码器采样分频计数 */
static PI_t     s_PI_Left, s_PI_Right;   /* 左右电机PI控制器 */

/* ---- 舵机PD参数 (需根据实际车况调参) ---- */
#define PD_KP    0.8f       /* 舵机PD: 比例系数 */
#define PD_KD    0.4f       /* 舵机PD: 微分系数 */

int core1_main(void)
{
    int16_t  enc_left, enc_right;
    int8_t   pwm_left,  pwm_right;
    float    position_err;

    /* ---- CPU1初始化 ---- */
    disable_Watchdog();
    interrupt_global_enable(0);

    /* ---- 外设初始化 ---- */
    Key_Init();                          /* 5键GPIO输入: P10_7~P11_1 (预留调参) */
    Encoder_Init();                      /* 编码器: TIM6(左方向)/TIM4(右正交) */
    Motor_Init();                        /* 电机双极PWM: ATOM0_CH0/2(左) + ATOM1_CH1+ATOM0_CH3(右) */
    Servo_Init();                        /* 舵机50Hz PWM: ATOM0_CH1_P33_9 */

    /* ---- PI控制器初始化 ---- */
    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* ---- PIT定时器初始化 ---- */
    pit_ms_init(CCU60_CH1, 5);          /* 按键扫描: 5ms */
    pit_ms_init(CCU61_CH0, 10);         /* PID控制:  10ms */

    /* ---- 等待CPU0初始化完成 (IPS200在CPU0初始化) ---- */
    cpu_wait_event_ready();

    /* ---- 主循环 ---- */
    while (TRUE)
    {
        /* ---- 按键扫描 (预留调参接口) ---- */
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;               /* 暂未绑定功能 */
        }

        /* ---- 等待10ms控制周期 ---- */
        if (!PID_Flag)
        {
            continue;
        }
        PID_Flag = 0;

        /* ---- 编码器分频采集 (减少噪声, 8个周期读一次) ---- */
        EncCount++;
        if (EncCount >= 8)
        {
            EncCount = 0;
            enc_left  = Encoder_Get_Left();
            enc_right = Encoder_Get_Right();
        }
        g_EncLeft  = enc_left;
        g_EncRight = enc_right;

        /* ---- 获取图像偏差 (CPU0计算) ---- */
        position_err = Err;

        /* ---- 舵机PD控制 (根据Err实时打角, PD_Update内部调Servo_SetAngleDeg) ---- */
        PD_Update(PD_KP, PD_KD);

        /* ---- 差速控制: 根据位置偏差设定左右电机目标偏置 ---- */
        if      (position_err >   7.0f && position_err <  15.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 0.5f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.1f);
        }
        else if (position_err >= 15.0f && position_err <  25.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 1.1f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.5f);
        }
        else if (position_err >= 25.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 1.5f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.8f);
        }
        else if (position_err <  -7.0f && position_err > -14.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.2f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.4f);
        }
        else if (position_err <= -14.0f && position_err > -20.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.3f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.55f);
        }
        else if (position_err <= -20.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.3f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.65f);
        }
        else {
            s_PI_Left.TargetBias  = 0;
            s_PI_Right.TargetBias = 0;
        }

        /* ---- 电机PI速度闭环 ---- */
        pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  g_StraightSpeed);
        pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, g_StraightSpeed);

        /* ---- 输出PWM (PI_Update内部已限幅, Motor层再次限幅保护) ---- */
        Motor_SetLeftPWM(pwm_left);
        Motor_SetRightPWM(pwm_right);
    }
}

#pragma section all restore

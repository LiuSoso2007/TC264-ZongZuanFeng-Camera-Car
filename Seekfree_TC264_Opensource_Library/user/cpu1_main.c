/**
 * cpu1_main.c  ---  CPU1: 运动控制 + IPS200显示
 *
 * CPU1 负责除摄像头外的所有外设:
 *   - IMU660RB 陀螺仪数据采集
 *   - 编码器速度读取 (左/右)
 *   - 舵机PD控制 (位置环 + 角速度环)
 *   - 电机PI控制 (增量式速度闭环, 支持弯道差速)
 *   - 按键采集
 *   - IPS200 显示屏信息输出
 *
 * 控制周期: 10ms (由 CCU61_CH0 PIT 中断驱动, PID_Flag)
 * 按键扫描:  5ms (由 CCU60_CH1 PIT 中断驱动, Key_Tick)
 *
 * CPU0→CPU1通信: 通过共享变量 Err (volatile, Shared.h中定义)
 *   Err: 图像偏差 (正=右偏, 负=左偏)
 */

#include "zf_common_headfile.h"
#include "IPS200.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "imu.h"
#include "isr.h"

#define YAW_KP_ERR    2.0f          /* 舵机PD: Err→目标角速度系数 */
#define YAW_KD_D      0.5f          /* 舵机PD: 角速度偏差→舵角系数 */

volatile uint8_t PID_Flag = 0;       /* PID控制定时标志 (isr.c 10ms中断置位) */

#pragma section all "cpu1_dsram"

static int8_t   g_StraightSpeed = 18;    /* 直道目标速度 */
static int16_t  g_EncLeft       = 0;     /* 左编码器计数 (调试用) */
static int16_t  g_EncRight      = 0;     /* 右编码器计数 (调试用) */
static PI_t     s_PI_Left, s_PI_Right;   /* 左右电机PI控制器 */
static uint8_t  g_IMU_Ok        = 0;     /* IMU初始化成功标志 */

float IMU_Yaw = 0.0f;                    /* IMU Z轴角速度 (度/s) */

int core1_main(void)
{
    int16_t  enc_left, enc_right;
    int8_t   pwm_left,  pwm_right;
    float    position_err;

    disable_Watchdog();
    interrupt_global_enable(0);

    /* ---- 外设初始化 ---- */
    Encoder_Init();
    Motor_Init();
    Servo_Init();
    Key_Init();
    IPS200_Init();

    /* ---- IMU初始化 ---- */
    g_IMU_Ok = !IMU_Init();
    if (!g_IMU_Ok)
    {
        /* IMU未连接, 显示提示 */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 10, "IMU not connected");
    }
    else
    {
        system_delay_ms(50);
        IMU_Calibrate(200);              /* 采集200个样本校准陀螺仪零偏 */
    }

    /* ---- PI控制器初始化 ---- */
    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* ---- 定时器初始化 ---- */
    pit_ms_init(CCU60_CH1, 5);           /* 按键扫描: 5ms */
    pit_ms_init(CCU61_CH0, 10);          /* PID控制:  10ms */

    cpu_wait_event_ready();
    system_delay_ms(100);

    if (g_IMU_Ok)
    {
        IMU_Update();
        IMU_ClearAngleZ();               /* 清零Z轴积分角度 */
    }

    /* ---- 主循环 ---- */
    while (TRUE)
    {
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;                /* 按键值暂未使用, 预留调试接口 */
        }

        if (g_IMU_Ok)
        {
            IMU_Update();                /* 更新IMU数据 */
        }

        if (!PID_Flag) continue;         /* 等待10ms定时标志 */
        PID_Flag = 0;

        if (g_IMU_Ok)
        {
            IMU_Yaw = IMU_GetYawRate();   /* 获取Z轴角速度 */
            IMU_IntegrateZ(0.01f);        /* Z轴积分 (dt=10ms) */
        }

        /* ---- 读取编码器速度 ---- */
        enc_left  = Encoder_Get_Left();
        enc_right = Encoder_Get_Right();
        g_EncLeft  = enc_left;
        g_EncRight = enc_right;

        /* ---- 获取图像偏差 (CPU0→CPU1共享变量) ---- */
        position_err = Err;

        if (g_IMU_Ok)
        {
            /*
             * 弯道差速策略:
             *   根据位置偏差 position_err 调节左右电机速度偏置,
             *   实现差速辅助转弯。偏差越大, 差速越明显。
             *
             *   右偏 (position_err > 0):  左轮加速, 右轮减速
             *   左偏 (position_err < 0):  左轮减速, 右轮加速
             */
            if      (position_err >   7.0f && position_err <  15.0f) {
                s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 0.5f);
                s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.1f);
            }
            else if (position_err >= 15.0f) {
                s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 1.1f);
                s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.5f);
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

            /* ---- 电机PI控制 ---- */
            pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  g_StraightSpeed);
            pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, g_StraightSpeed);
            Motor_SetLeftPWM(pwm_left);
            Motor_SetRightPWM(pwm_right);

            /* ---- 舵机PD控制 ---- */
            PD_Update(YAW_KP_ERR, YAW_KD_D);
        }
    }
}

#pragma section all restore
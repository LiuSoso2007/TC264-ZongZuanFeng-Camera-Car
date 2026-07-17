/**
 * cpu1_main.c  ---  CPU1: 运动控制(差速电机PI + 固定舵机角度) + 外设管理
 *
 * 双核架构:
 *   CPU0: 摄像头图像采集 + OTSU二值化 + 图像处理 -> Err 共享
 *   CPU1: 编码器 + 电机PI差速控制 + 固定舵机角度
 *         + IPS200数据显示 + 按键扫描
 *
 * 控制周期: 10ms (CCU61_CH0 PIT定时中断, isr.c中置PID_Flag)
 * 按键扫描: 5ms  (CCU60_CH1 PIT定时中断, isr.c中调用Key_Tick)
 *
 * 核间通信: Err (volatile, Shared.h声明)
 *   Err: 图像偏差 (CPU0计算, CPU1读取用于差速PI)
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

volatile uint8_t PID_Flag = 0;       /* PID控制定时标志 (isr.c 10ms中断置位) */

#pragma section all "cpu1_dsram"

/* ---- CPU1本地变量 ---- */
static int8_t   g_StraightSpeed = 50;    /* 直线基准速度 (可通过按键调参) */
static uint8_t  g_CalibAngle    = 80;    /* 舵机中位角度 (实际车辆需校准) */
static int16_t  g_EncLeft       = 0;     /* 左编码器累积值 (每周期) */
static int16_t  g_EncRight      = 0;     /* 右编码器累积值 (每周期) */
static int16_t  EncCount        = 0;     /* 编码器采样分频计数 */
static PI_t     s_PI_Left, s_PI_Right;   /* 左右电机PI控制器 */

/* ---- LCD显示参数 ---- */
#define LCD_DIV      10
#define LCD_LABEL_X  10
#define LCD_VALUE_X  100
#define LCD_Y_BASE   62
#define LCD_ROW_H    16

int core1_main(void)
{
    int16_t  enc_left, enc_right;
    int8_t   pwm_left,  pwm_right;
    float    position_err;
    static uint8_t lcd_cnt = 0, lcd_row = 0, lcd_dirty = 0;

    /* ---- CPU1初始化 ---- */
    disable_Watchdog();
    interrupt_global_enable(0);

    /* ---- 外设初始化 ---- */
    IPS200_Init();                       /* IPS200 SPI显示屏: 320x240 */
    Key_Init();                          /* 5键GPIO输入: P10_7~P11_1 */
    Encoder_Init();                      /* 编码器: TIM6(左方向)/TIM4(右正交) */
    Motor_Init();                        /* 电机双极PWM: ATOM0_CH0/2(左) + ATOM1_CH1+ATOM0_CH3(右) */
    Servo_Init();                        /* 舵机50Hz PWM: ATOM0_CH1_P33_9 */

    /* ---- PI控制器初始化 ---- */
    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* ---- LCD静态文本 (时间片逐行刷新) ---- */
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*0, "L_Act:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*1, "L_Tar:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*2, "R_Act:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*3, "R_Tar:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*4, "Angle:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*5, "Speed:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*6, "Err:");

    /* ---- PIT定时器初始化 ---- */
    pit_ms_init(CCU60_CH1, 5);          /* 按键扫描: 5ms */
    pit_ms_init(CCU61_CH0, 10);         /* PID控制:  10ms */

    /* ---- 等待CPU0初始化完成, 避免IPS200冲突 ---- */
    cpu_wait_event_ready();

    /* ---- 主循环 ---- */
    while (TRUE)
    {
        /* ---- LCD时间片显示 ---- */
        if (lcd_dirty)
        {
            ips200_set_color(RGB565_BLUE, RGB565_WHITE);
            switch (lcd_row)
            {
                case 0: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*0, (int32)g_EncLeft,  5); break;
                case 1: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*1, (int32)g_StraightSpeed, 5); break;
                case 2: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*2, (int32)g_EncRight, 5); break;
                case 3: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*3, (int32)g_StraightSpeed, 5); break;
                case 4: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*4, (int32)g_CalibAngle, 5); break;
                case 5: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*5, (int32)g_StraightSpeed, 5); break;
                case 6: ips200_show_float(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*6, Err, 3, 2); break;
            }
            if (++lcd_row >= 7) { lcd_row = 0; lcd_dirty = 0; }
        }

        /* ---- 按键扫描 (预留调参接口) ---- */
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;               /* 暂未绑定功能 */
        }

        /* ---- 等待10ms控制周期 ---- */
        if (!PID_Flag)
        {
            /* 空闲时更新时间片LCD */
            if (++lcd_cnt >= LCD_DIV) { lcd_cnt = 0; lcd_row = 0; lcd_dirty = 1; }
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

        /* ---- 输出PWM (PI_Update内已限幅, Motor层再次限幅保护) ---- */
        Motor_SetLeftPWM(pwm_left);
        Motor_SetRightPWM(pwm_right);

        /* ---- 舵机固定中位角度 (后续可改为PD变角) ---- */
        Servo_SetAngleDeg(g_CalibAngle);

        /* ---- LCD时间片刷新计数 ---- */
        if (++lcd_cnt >= LCD_DIV) { lcd_cnt = 0; lcd_row = 0; lcd_dirty = 1; }
    }
}

#pragma section all restore

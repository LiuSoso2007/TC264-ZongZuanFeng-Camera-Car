/**
 * cpu1_main.c  ---  CPU1: 运动控制(舵机PD打角 + 电机固定占空比)
 *
 * 双核架构:
 *   CPU0: 摄像头图像采集 + OTSU二值化 + 图像处理 + IPS200全屏显示
 *   CPU1: 舵机PD实时打角 + 电机固定占空比 + 按键扫描(预留)
 *
 * 转向策略: 纯舵机PD控制, 电机不做差速
 *   Err -> PD_Update -> Servo_SetAngleDeg (舵机实时打角)
 *   左右电机始终同速驱动
 *
 * 控制周期: 10ms (CCU61_CH0 PIT定时中断, isr.c中置PID_Flag)
 * 按键扫描: 5ms  (CCU60_CH1 PIT定时中断, isr.c中调用Key_Tick)
 *
 * 核间通信: Err (volatile, cpu0_main.c定义)
 *   Err: 图像偏差 (CPU0计算 -> CPU1读取 -> 舵机PD打角)
 */

#include "zf_common_headfile.h"
#include "Key.h"
#include "Motor.h"
#include "Servo.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile uint8_t PID_Flag = 0;       /* PID控制定时标志 (isr.c 10ms中断置位) */

#pragma section all "cpu1_dsram"

/* ---- 调参区 (需根据实际车况调整) ---- */
#define MOTOR_SPEED    50      /* 电机固定占空比 (0~100, 双轮同速驱动) */
#define PD_KP          2.5f    /* 舵机PD: 比例系数 */
#define PD_KD          0.4f    /* 舵机PD: 微分系数 */

int core1_main(void)
{
    /* ---- CPU1初始化 ---- */
    disable_Watchdog();
    interrupt_global_enable(0);

    /* ---- 外设初始化 ---- */
    Key_Init();                          /* 5键GPIO输入: P10_7~P11_1 (预留调参) */
    Motor_Init();                        /* 电机双极PWM: ATOM0_CH0/2(左) + ATOM1_CH1+ATOM0_CH3(右) */
    Servo_Init();                        /* 舵机50Hz PWM: ATOM0_CH1_P33_9 */

    /* ---- PIT定时器初始化 ---- */
    pit_ms_init(CCU60_CH1, 5);          /* 按键扫描: 5ms */
    pit_ms_init(CCU61_CH0, 10);         /* PID控制:  10ms */

    /* ---- 等待CPU0初始化完成 ---- */
    cpu_wait_event_ready();

    /* ---- 主循环 ---- */
    while (TRUE)
    {
        /* 按键扫描 (预留调参接口) */
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;               /* 暂未绑定功能 */
        }

        /* 等待10ms控制周期 */
        if (!PID_Flag)
            continue;
        PID_Flag = 0;

        /* 舵机PD控制: 根据Err实时打角 (PD_Update内部调Servo_SetAngleDeg) */
        PD_Update(PD_KP, PD_KD);

        /* 电机固定占空比: 双轮同速驱动, 转向完全由舵机负责 */
        Motor_SetLeftPWM((int8_t)MOTOR_SPEED);
        Motor_SetRightPWM((int8_t)MOTOR_SPEED);
    }
}

#pragma section all restore

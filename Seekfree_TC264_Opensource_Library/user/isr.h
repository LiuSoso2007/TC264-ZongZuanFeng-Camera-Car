#ifndef _isr_h
#define _isr_h

#include "zf_common_headfile.h"

/*
 * isr.h --- 中断服务函数声明与共享变量
 *
 * 本文件声明:
 *   1. 跨核共享变量 (PID_Flag, Err)
 *   2. 跨核共享回调 (Key_Tick)
 *   3. 来自逐飞设备库的回调函数引用 (extern callback_function)
 *
 * 中断分配:
 *   CPU0: 摄像头 ERU/DMA + UART1(摄像头配置) + UART0(调试)
 *   CPU1: CCU60_CH1 (按键扫描 5ms) + CCU61_CH0 (PID定时 10ms)
 */

/* ---- 跨核共享变量 ---- */

/*
 * PID_Flag - PID定时标志
 * 生产者: isr.c cc61_pit_ch0_isr (10ms定时中断, CPU1)
 * 消费者: cpu1_main.c (CPU1 主循环)
 * 用途: 每10ms触发一次PID控制计算
 */
extern volatile uint8_t PID_Flag;

/*
 * Err - 图像偏差共享变量
 * 生产者: cpu0_main.c (CPU0 图像处理后)
 * 消费者: cpu1_main.c (CPU1 PD/电机控制)
 * 含义: 赛车中心线偏离图像中线的像素偏差 (正=右偏, 负=左偏)
 */
extern volatile float    Err;

/* ---- 跨核共享回调 ---- */

/*
 * Key_Tick - 按键扫描定时回调
 * 生产者: cpu1_main.c 定义
 * 消费者: isr.c cc60_pit_ch1_isr (5ms定时中断, CPU1)
 * 用途: 每5ms执行一次按键状态扫描
 */
extern void Key_Tick(void);

/*
 * 以下回调函数由逐飞 zf_device_type.h 中的 callback_function 结构体定义,
 * 实际实现在逐飞设备库中 (zf_device_mt9v03x / zf_device_wireless_uart 等),
 * 本处仅声明引用, 不修改实现:
 *
 *   camera_vsync_handler         - 摄像头场同步中断 (ERU_CH3)
 *   camera_dma_handler           - 摄像头DMA传输完成中断 (DMA_CH5)
 *   camera_uart_handler          - 摄像头UART接收中断 (UART1 RX)
 *   wireless_module_uart_handler - 无线模块UART接收中断 (UART2 RX)
 *   tof_module_exti_handler      - TOF测距模块外部中断 (ERU_CH1)
 */

#endif
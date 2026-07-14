/*********************************************************************************************************************
 * isr.c - TC264 中断服务函数 (ISR)
 *
 * 本文件包含所有外设中断服务函数, 按功能分为:
 *   PIT定时器中断:
 *     CCU60_CH0 -- 预留
 *     CCU60_CH1 -- 按键扫描 (5ms, CPU1)
 *     CCU61_CH0 -- PID控制标志 (10ms, CPU1)
 *     CCU61_CH1 -- 预留
 *
 *   ERU外部中断:
 *     ERU_CH0/CH4 -- IMU660RC数据就绪 / 预留
 *     ERU_CH1/CH5 -- TOF测距模块 / 预留
 *     ERU_CH3/CH7 -- 摄像头场同步VSYNC / 预留
 *
 *   DMA中断:
 *     DMA_CH5 -- 摄像头图像数据DMA传输完成
 *
 *   UART串口中断:
 *     UART0 -- 调试串口 (TX+RX+Error)
 *     UART1 -- 摄像头配置串口 (TX+RX+Error)
 *     UART2 -- 无线模块串口 (TX+RX+Error)
 *     UART3 -- GNSS模块串口 (TX+RX+Error)
 *
 * 注意:
 *   - 摄像头PCLK触发使用ERU_CH2, 由逐飞库内部处理, 不在此文件中
 *   - 按键Tick和PID_Flag仅CPU1使用 (CCU60_CH1 / CCU61_CH0)
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

/*
 * 中断服务函数总览
 *
 * 包含内容:
 *   CPU0: 摄像头 ERU/DMA + UART1(摄像头配置) + UART0(调试) + IMU EXT
 *   CPU1: CCU60_CH1 (按键扫描 5ms) + CCU61_CH0 (PID定时 10ms)
 *
 * 摄像头 PCLK 触发为 ERU 第2通道, DMA 为第5通道。
 * 按键扫描使用 CPU1 的 CCU60_CH1 定时器。
 */

// =========================== PIT 定时器中断 ===========================
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    /* 预留: 暂无功能 */
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    Key_Tick();                                    /* 按键扫描 (5ms) -- CPU1专用 */
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    PID_Flag = 1;                                  /* CPU1 PID控制标志 (10ms) */
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
    /* 预留: 暂无功能 */
}
// =========================== PIT 结束 ===============================


// **************************** ERU 外部中断 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

        imu660rc_callback();                       /* IMU660RC 数据就绪 */
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);

        /* 预留: ERU_CH4 暂无功能 */

    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                 /* TOF测距模块中断 */

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        /* 预留: ERU_CH5 暂无功能 */
    }
}

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                    /* 摄像头场同步VSYNC */
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);

        /* 预留: ERU_CH7 暂无功能 */

    }
}
// **************************** ERU 结束 ****************************


// **************************** DMA 中断 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();                          /* 摄像头图像DMA传输完成 */
}
// **************************** DMA 结束 ****************************


// **************************** 串口中断 ****************************
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);

    /* UART0 TX: 调试串口发送完成, 暂无附加处理 */
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();                 /* 调试串口接收 */
#endif
}

IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);

    /* UART1 TX: 摄像头配置串口发送完成, 暂无附加处理 */
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_uart_handler();                         /* 摄像头UART接收 */
}

IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);

    /* UART2 TX: 无线模块发送完成, 暂无附加处理 */
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();                /* 无线模块UART接收 */

}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);

    /* UART3 TX: GNSS模块发送完成, 暂无附加处理 */
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    gnss_uart_callback();                          /* GNSS模块UART接收 */

}

/* 串口错误中断 */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
// **************************** 串口结束 ****************************
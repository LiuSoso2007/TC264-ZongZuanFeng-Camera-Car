/*********************************************************************************************************************
* TC264 Opensourec Library
* Copyright (c) 2022 SEEKFREE
* ...
* ????          isr
* ????          ??????????
* ????          ?? libraries/doc ???? version ?? ????
* ????          ADS v1.10.2
* ????          TC264D
* ????          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

/*
 * ?????? (ISR)
 *
 * ????:
 *   CPU0: ??? ERU/DMA + UART1(?????) + UART0(??) + IMU EXT
 *   CPU1: CCU60_CH1 (???? 5ms) + CCU61_CH0 (???? 10ms)
 *
 * ??? PCLK ???? ERU ?2??, DMA ???5??.
 * ?????? CPU0 ??? CPU1 (CCU60_CH1).
 */

// =========================== PIT ?????? ===========================
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    /* ???? */
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    Key_Tick();                                    // ???? (5ms) -- CPU1 ?
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    PID_Flag = 1;                                  // CPU1 ?????? (10ms)
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
    /* ???? */
}
// =========================== PIT ???? ===============================

// **************************** ERU ???? ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

        imu660rc_callback();
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);



    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);



    }
}
// **************************** ERU ?????? ****************************


// **************************** DMA ?? ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}
// **************************** DMA ???? ****************************


// **************************** ???? ****************************
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);


}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
}

IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);


}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_uart_handler();
}

IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);


}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();


}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);


}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    gnss_uart_callback();


}

// ??????
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
// **************************** ?????? ****************************

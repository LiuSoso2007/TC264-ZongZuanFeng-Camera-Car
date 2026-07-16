/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + IPS200调试显示
 *
 * 每帧: 二值化 + 全量显示 (原始图 + OTSU阈值 + 二值图)
 * 注意: IPS200 由 CPU1 初始化, CPU0 不重复初始化 (避免 SPI GPIO 竞态)
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();           /* 图像压缩初始化 (仅一次) */

    /*
     * cpu_wait_event_ready() 等待 CPU1 完成所有初始化,
     * 包括 CPU1 中的 IPS200_Init()。之后 CPU0 才能安全使用显示屏.
     */
    cpu_wait_event_ready();

    /* 启动时显示初始信息, 即使摄像头未产生帧也能看到屏幕 */
    ips200_full(RGB565_BLACK);

    {
        uint32 fc = 0, lc = 0;
        while (TRUE)
        {
            lc++;
            if (Camera_IsFrameReady())
            {
                fc++;
                ips200_set_color(RGB565_GREEN, RGB565_BLACK);
                ips200_show_string(2, 2, "F:");  ips200_show_uint(25, 2, fc, 5);
                Camera_GetBinaryImage();
                ips200_set_color(RGB565_CYAN, RGB565_BLACK);
                ips200_show_string(2, 18, "T="); ips200_show_uint(35, 18, Camera_Threshold, 3);
                Camera_ShowDebug();
                ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
                ips200_show_string(2, 34, "OK");
            }
            if ((lc % 5000) == 0) {
                ips200_set_color(RGB565_WHITE, RGB565_BLACK);
                ips200_show_string(2, 230, "L:"); ips200_show_uint(25, 230, lc/1000, 4);
                ips200_show_string(65, 230, "K");
            }
        }
    }
}

#pragma section all restore

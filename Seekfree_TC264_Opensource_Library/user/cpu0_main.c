/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + IPS200调试显示
 *
 * 帧率优化: 每帧只调用 Camera_ShowBinaryFast() (仅二值图, SPI传输量最小),
 *           每5帧调用一次 Camera_ShowDebug() (全量: 原始图+阈值+二值图)
 *
 * CPU0 -> CPU1 通信: 通过共享变量 Err (Shared.h 中定义)
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;    /* 图像偏差 (CPU0 -> CPU1) */

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    uint8 frame_cnt = 0;             /* 帧计数器: 每5帧全量显示一次 */

    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();            /* 图像压缩初始化 (仅一次) */

    cpu_wait_event_ready();

    while (TRUE)
    {
        if (Camera_IsFrameReady())
        {
            /*
             * 每帧都做二值化 (算阈值, 更新 Pixle)
             */
            Camera_GetBinaryImage();

            /*
             * 显示策略:
             *   每5帧: 全量调试 (原始图+阈值+二值图) —— 慢但信息全
             *   其余帧: 仅二值图 —— 快, 帧率优先
             */
            if (++frame_cnt >= 5) {
                frame_cnt = 0;
                Camera_ShowDebug();
            } else {
                Camera_ShowBinaryFast();
            }
        }
    }
}

#pragma section all restore
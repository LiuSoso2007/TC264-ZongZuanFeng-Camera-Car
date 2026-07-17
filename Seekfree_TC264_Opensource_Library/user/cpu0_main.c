/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + IPS200调试显示
 *
 * 每帧: 二值化 + 全量显示 (原始图 + OTSU阈值 + 二值图)
 * IPS200显示由CPU0独占管理, CPU1不操作显示屏
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

        ips200_full(RGB565_BLACK);  /* clear screen to black */
    /* IPS200初始化在CPU0, 与摄像头同核,避免双核SPI冲突 */
    IPS200_Init();
    cpu_wait_event_ready();
        if (Camera_IsFrameReady())
        {
            Camera_GetBinaryImage();

            /* ---- 图像处理主流程: 二值化 -> 元素识别 ---- */
            Flag_init();
            Get_BaseLine();
            Get_AllLine();
            Scan_Element();
            Element_Handle();

            /* ---- 计算 Err (图像偏差) 给 CPU1 使用 ---- */
            if (ImageStatus.OFFLine < 55)
            {
                Err = (float)(ImageDeal[SCAN_BASE_START_ROW].Center - ImageSensorMid)
                    / (float)ImageSensorMid;
            }
            Camera_ShowDebug();
        }
    }
}

#pragma section all restore
/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + IPS200全屏显示
 *
 * 每帧: 二值化 + 全屏显示 (原始图 + OTSU阈值 + 二值图 + 元素识别)
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
    Camera_CompressInit();           /* 图像压缩初始化 (仅调用一次) */

    /*
     * IPS200初始化放在CPU0, 和摄像头共享同一核,
     * 避免双核同时操作SPI导致冲突.
     */
    IPS200_Init();

    cpu_wait_event_ready();

    ips200_full(RGB565_BLACK);  /* 清屏为黑色 */

    while (TRUE)
    {
        if (Camera_IsFrameReady())
        {
            Camera_GetBinaryImage();

            /* ---- 图像处理流水线: 二值化 -> 元素识别 ---- */
            Flag_init();
            Get_BaseLine();
            Get_AllLine();
            Scan_Element();
            Element_Handle();

            /* ---- 计算 Err (图像偏差) 供 CPU1 使用 ---- */
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

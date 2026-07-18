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
            /* 近场3行平均，Err保持像素单位，与CPU1的PD参数一致。 */
            if (ImageStatus.OFFLine < 48)
            {
                Err = (float)((ImageDeal[52].Center + ImageDeal[51].Center + ImageDeal[50].Center) / 3
                     - ImageSensorMid);
            }
            else if (ImageStatus.OFFLine < SCAN_BASE_START_ROW
                  && ImageDeal[ImageStatus.OFFLine + 1].Wide > 8)
            {
                Err = (float)(ImageDeal[ImageStatus.OFFLine + 1].Center - ImageSensorMid);
            }
            else
            {
                Err = 0.0f;
            }
            /* 每帧只走QSPI2寄存器连续直刷，禁止回到逐字节等待的调试显示路径。 */
            IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        }
    }
}

#pragma section all restore

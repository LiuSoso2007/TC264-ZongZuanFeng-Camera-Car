/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + IPS200调试显示
 *
 * CPU0 专注于图像处理算法, 不处理任何外设控制:
 *   - MT9V03X 摄像头图像采集 (DMA+ERU中断驱动)
 *   - 图像压缩 (188x120 -> 94x60)
 *   - OTSU大津法二值化
 *   - IPS200 屏幕调试显示 (原始图+压缩图+阈值)
 *
 * 后续扩展:
 *   - 搜线算法 (边线检测)
 *   - 元素识别 (弯道/坡道/斑马线/圆环/路障/断路)
 *   - 偏差计算 (Err 输出给CPU1做运动控制)
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
    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();       // 图像压缩初始化 (仅需调用一次)

    cpu_wait_event_ready();

    while (TRUE)
    {
        if (Camera_IsFrameReady())
        {
            /*
             * 图像处理管线: 压缩 -> OTSU二值化 -> 调试显示
             * Camera_ShowDebug() 包含:
             *   上部: 原始灰度图 188x120
             *   中部: OTSU阈值数值
             *   下部: 二值化图像 94x60
             */
            Camera_GetBinaryImage();
            Camera_ShowDebug();
        }
    }
}

#pragma section all restore
/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + 可选IPS200显示
 *
 * 每帧: 二值化 + 元素识别；仅在显示开关启用时刷新IPS200
 * IPS200显示由CPU0独占管理, CPU1不操作显示屏
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;
volatile uint8_t StopRequest = 0U;

/* 压缩图行号越小前瞻越远；40～42行兼顾弯道提前量和远场稳定性。 */
#define STEERING_LOOKAHEAD_ROW 40
/* 比赛默认关闭IPS200，调试时改为1；关闭后编译器移除全部屏幕调用。 */
#define IPS200_DISPLAY_ENABLE 1
#if IPS200_DISPLAY_ENABLE
/* 摄像头50帧时每5帧刷新一次编码器数值，避免文字刷新拖慢画面。 */
#define ENCODER_DISPLAY_DIV 5U
#endif
/* 连续两帧确认可滤除单帧误判，确认后保持全速8帧再停车越过终点线。 */

/* 斑马线停车延迟帧数：检测到斑马线后延迟N帧再停车，
   让车模通过斑马线(终点)后再停下。50fps下1帧=20ms。 */
#define ZEBRA_STOP_DELAY_FRAMES  15

/* 斑马线检测到后立即停车，不再需要确认和延迟帧 */

#pragma section all "cpu0_dsram"

int core0_main(void)
{
#if IPS200_DISPLAY_ENABLE
    static uint8_t encoder_display_cnt = 0U;
#endif

    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();           /* 图像压缩初始化 (仅调用一次) */

#if IPS200_DISPLAY_ENABLE
    /*
     * IPS200初始化放在CPU0, 和摄像头共享同一核,
     * 避免双核同时操作SPI导致冲突.
     */
    IPS200_Init();
#endif

    cpu_wait_event_ready();

#if IPS200_DISPLAY_ENABLE
    ips200_full(RGB565_BLACK);  /* 清屏为黑色 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(2U, 128U, "L_Enc:");
    ips200_show_string(2U, 144U, "R_Enc:");
    ips200_show_string(2U, 170U, "Err:");
    ips200_show_string(2U, 190U, "Ring:");
    ips200_show_string(2U, 208U, "Thr:");
    ips200_show_string(2U, 222U, "Zebra:");
#endif

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

            /* 斑马线检测后延迟停车：触发后倒数N帧，让车完整通过斑马线终点 */
            {
                static uint8_t zebra_triggered = 0;  /* 是否已触发斑马线 */
                static uint8_t zebra_delay_cnt = 0;  /* 触发后累计帧数 */

                if (ImageFlag.Zebra_Flag != 0 && zebra_triggered == 0
                 && StopRequest == 0U)
                {
                    zebra_triggered = 1;     /* 锁存触发状态 */
                    zebra_delay_cnt = 0;     /* 开始倒数 */
                }

                if (zebra_triggered == 1)
                {
                    zebra_delay_cnt++;
                    if (zebra_delay_cnt >= ZEBRA_STOP_DELAY_FRAMES)
                    {
                        StopRequest = 1U;
                        zebra_triggered = 0;
                    }
                }
            }

            /* ---- 计算 Err (图像偏差) 供 CPU1 使用 ---- */
            /* 前瞻3行平均，Err保持像素单位，与CPU1的PD参数一致。 */
            if (ImageStatus.OFFLine < STEERING_LOOKAHEAD_ROW)
            {
                Err = (float)((ImageDeal[STEERING_LOOKAHEAD_ROW].Center
                     + ImageDeal[STEERING_LOOKAHEAD_ROW + 1].Center
                     + ImageDeal[STEERING_LOOKAHEAD_ROW + 2].Center) / 3
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
#if IPS200_DISPLAY_ENABLE
            /* 每帧只走QSPI2寄存器连续直刷，禁止回到逐字节等待的调试显示路径。 */
            /* 显示二值化图像，赛道中线(蓝)与车身中线(红)叠加在二值图上 */
            Camera_ShowBinaryFast();
            /* 赛道中线：蓝色折线，坐标与二值图对齐(xo+Center, row)，每帧自然覆盖 */
            {
                uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);
                int row;
                for (row = SCAN_BASE_START_ROW; (row - 1) > ImageStatus.OFFLine; row--)
                {
                    if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
                    if (ImageDeal[row-1].Center < 0 || ImageDeal[row-1].Center >= LCDW) continue;
                    ips200_draw_line(
                        xo + (uint16)ImageDeal[row].Center,   (uint16)row,
                        xo + (uint16)ImageDeal[row-1].Center, (uint16)(row-1),
                        RGB565_BLUE);
                }
            }
            /* ??????????????????? */
            {
                int row;
                for (row = SCAN_BASE_START_ROW; (row - 1) > ImageStatus.OFFLine; row--)
                {
                    if (ImageDeal[row].LeftBorder >= 0 && ImageDeal[row].LeftBorder < LCDW
                     && ImageDeal[row-1].LeftBorder >= 0 && ImageDeal[row-1].LeftBorder < LCDW)
                    {
                        ips200_draw_line(
                            xo + (uint16)ImageDeal[row].LeftBorder,   (uint16)row,
                            xo + (uint16)ImageDeal[row-1].LeftBorder, (uint16)(row-1),
                            RGB565_RED);
                    }
                    if (ImageDeal[row].RightBorder >= 0 && ImageDeal[row].RightBorder < LCDW
                     && ImageDeal[row-1].RightBorder >= 0 && ImageDeal[row-1].RightBorder < LCDW)
                    {
                        ips200_draw_line(
                            xo + (uint16)ImageDeal[row].RightBorder,   (uint16)row,
                            xo + (uint16)ImageDeal[row-1].RightBorder, (uint16)(row-1),
                            RGB565_GREEN);
                    }
                }
            }
            /* 车身中线：红色竖线固定在图像水平中心 */
            ips200_draw_line(94, 0, 94, 59, RGB565_RED);
            if (++encoder_display_cnt >= ENCODER_DISPLAY_DIV)
            {
                encoder_display_cnt = 0U;
                ips200_show_int(58U, 128U, (int32)EncLeft, 5U);
                ips200_show_int(58U, 144U, (int32)EncRight, 5U);
                ips200_show_int(58U, 170U, (int32)Err, 5U);
                /* 圆环方向和阶段标志位显示，便于调试状态机切换 */
                {
                    static const char *ring_st_name[] = {"IDLE","CNFM","APRC","ENTR","INSD","EXIT","RECV"};
                    uint8 ring_st = (uint8)ImageFlag.image_element_rings_flag;
                    if (ImageFlag.image_element_rings == 1U && ring_st < 7U)
                    {
                        ips200_show_string(50U, 190U, "L-");
                        ips200_show_string(68U, 190U, ring_st_name[ring_st]);
                    }
                    else if (ImageFlag.image_element_rings == 2U && ring_st < 7U)
                    {
                        ips200_show_string(50U, 190U, "R-");
                        ips200_show_string(68U, 190U, ring_st_name[ring_st]);
                    }
                    else
                    {
                        ips200_show_string(50U, 190U, "---   ");
                    }
                }
                ips200_show_int(50U, 208U, (int32)Camera_Threshold, 3U);
                ips200_show_int(50U, 222U, (int32)g_ZebraSum, 3U);
            }
                /* ringflag: 0=?? 1=CONFIRM 2=APPROACH 3=ENTRY 4=INSIDE 5=EXIT */
                {
                    uint8 rf = 0;
                    if (ImageFlag.image_element_rings != 0)
                    {
                        switch ((uint8)ImageFlag.image_element_rings_flag)
                        {
                            case 1: rf = 1; break;
                            case 2: rf = 2; break;
                            case 3: rf = 3; break;
                            case 4: rf = 4; break;
                            default: rf = 5; break;
                        }
                    }
                    ips200_show_string(2U, 236U, "RF:");
                    ips200_show_uint(28U, 236U, rf, 1U);
                }
#endif
        }
    }
}

#pragma section all restore

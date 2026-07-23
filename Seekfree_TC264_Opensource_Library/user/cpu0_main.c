/**
 * cpu0_main.c  ---  CPU0: 摄像头图像采集 + 图像处理 + 可选IPS200显示
 *
 * 每帧: 二值化 + 元素识别; 将结果显示到定时刷新IPS200
 * IPS200显示由CPU0独占控制, CPU1不参与显示。
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;
volatile uint8_t StopRequest = 0U;

/* 压缩图中行号越小前瞻越远，40比42中间稍近但更稳定。 */
#define STEERING_LOOKAHEAD_ROW 40

/* 文字仪表盘开关(轻量)，1=开 0=关。关闭后屏幕全黑，但会失去DMA同步延迟。 */
#define IPS200_TEXT_DISPLAY_ENABLE 1
#define IPS200_DISPLAY_IMAGE_ENABLE 0


#if IPS200_TEXT_DISPLAY_ENABLE
/* 摄像头50帧时每5帧刷新一次编码器数值，避免刷新拖慢主循环。 */
#define ENCODER_DISPLAY_DIV 5U
#endif
/* 斑马线帧确认: 退出斑马线帧计数，确认后保持全局8帧后停止，以越过终点线。 */

/* 斑马线停止延迟帧数: 检测到斑马线后延迟N帧后停车。
   斑马线同时充当终点线，延迟需略长以确保车体完全过线后再刹停。
   50fps，1帧=20ms，22帧 ≈ 440ms。 */
#define ZEBRA_STOP_DELAY_FRAMES  22
/* 斑马线检测到第3次确认后延迟帧数 */

#pragma section all "cpu0_dsram"

int core0_main(void)
{
#if IPS200_TEXT_DISPLAY_ENABLE
    static uint8_t encoder_display_cnt = 0U;
#endif

    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();           /* 图像压缩初始化 (调用一次即可) */

#if IPS200_TEXT_DISPLAY_ENABLE
    /*
     * IPS200初始化放在CPU0, 与摄像头不在同一核,
     * 避免双核同时操作SPI导致冲突.
     */
    IPS200_Init();
#endif

    cpu_wait_event_ready();

#if IPS200_TEXT_DISPLAY_ENABLE
    ips200_set_color(RGB565_BLACK, RGB565_WHITE); /* 黑字白底 */
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

            /* ---- 图像处理流水线: 二值图 -> 元素识别 ---- */
            Flag_init();
            Get_BaseLine();
            Get_AllLine();
            Scan_Element();
            Element_Handle();

            /* 斑马线检测延迟停车：收到N帧后让车辆通过终点线 */
            {
                static uint8_t zebra_triggered = 0;  /* 是否已触发斑马线 */
                static uint8_t zebra_delay_cnt = 0;  /* 检测后累计帧数 */

                if (ImageFlag.Zebra_Flag != 0 && zebra_triggered == 0
                 && StopRequest == 0U)
                {
                    zebra_triggered = 1;     /* 锁存触发状态 */
                    zebra_delay_cnt = 0;     /* 开始计数 */
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
            /* 前瞻3行平均Err，减少抖动，供给CPU1的PD舵机控制一环。 */
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
#if IPS200_DISPLAY_IMAGE_ENABLE
            /* 每帧只写QSPI2到达，避免直接刷新防止闪烁，节约带宽以显示路径线 */
            /* 显示二值图和赛道中线(蓝)车左侧边界(红)右侧边界(绿)的二值图 */
            Camera_ShowBinaryImage();
            /* 绘制赛道中线：蓝色线从底部向上逐行画 (xo+Center, row)，每帧自然刷新 */
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
            /* 绘制左右边界线：红=左边界，绿=右边界 */
            {
                uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);
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
            /* 绘制图像水平中线：红色参考线 */
            ips200_draw_line(94, 0, 94, 59, RGB565_RED);
#endif



#if IPS200_TEXT_DISPLAY_ENABLE
            if (++encoder_display_cnt >= ENCODER_DISPLAY_DIV)
            {
                encoder_display_cnt = 0U;
                ips200_show_int(58U, 128U, (int32)EncLeft, 5U);
                ips200_show_int(58U, 144U, (int32)EncRight, 5U);
                ips200_show_int(58U, 170U, (int32)Err, 5U);
                /* 圆环阶段状态位显示：左环L-/右环R- + 阶段缩写 */
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
                /* ringflag: 0=无 1=CONFIRM 2=APPROACH 3=ENTRY 4=INSIDE 5=EXIT */
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
                ips200_show_string(50U, 236U, "CB:");
                ips200_show_uint(74U, 236U, (uint32)g_corner_black_max, 2U);
                ips200_show_string(100U, 236U, "BW:");
                ips200_show_uint(124U, 236U, (uint32)g_bottom_black_width, 2U);
                ips200_show_string(150U, 236U, "MS");
                ips200_show_uint(168U, 236U, (uint32)g_ring_miss_cnt, 2U);
                ips200_show_string(2U, 250U, "ML");
                ips200_show_uint(20U, 250U, (uint32)ImageStatus.Miss_Left_lines, 2U);
                ips200_show_string(50U, 250U, "MR");
                ips200_show_uint(68U, 250U, (uint32)ImageStatus.Miss_Right_lines, 2U);
                ips200_show_string(100U, 250U, "LJ:");
                ips200_show_uint(124U, 250U, (uint32)g_left_jump_count, 2U);
                ips200_show_string(150U, 250U, "RJ:");
                ips200_show_uint(174U, 250U, (uint32)g_right_jump_count, 2U);
                ips200_show_string(2U, 264U, "SQ:");
                ips200_show_uint(24U, 264U, (uint32)g_edge_squeezed_dbg, 1U);
                ips200_show_string(50U, 264U, "VR:");
                ips200_show_int(74U, 264U, (int32)g_approach_valley_row, 3U);
                ips200_show_string(100U, 264U, "PH:");
                ips200_show_uint(122U, 264U, (uint32)g_ring_phase_dbg, 1U);

                 }
#endif
        }
    }
}

#pragma section all restore

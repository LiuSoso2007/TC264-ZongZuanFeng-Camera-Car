/**
 * cpu0_main.c  ---  CPU0: ����ͷͼ��ɼ� + ͼ���� + ��ѡIPS200��ʾ
 *
 * ÿ֡: ��ֵ�� + Ԫ��ʶ�𣻽�����ʾ��������ʱˢ��IPS200
 * IPS200��ʾ��CPU0��ռ����, CPU1��������ʾ��
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;
volatile uint8_t StopRequest = 0U;

/* ѹ��ͼ�к�ԽСǰհԽԶ��40��42�м�������ǰ����Զ���ȶ��ԡ� */
#define STEERING_LOOKAHEAD_ROW 40

/* ����Ĭ�Ϲر�IPS200������ʱ��Ϊ1���رպ�������Ƴ�ȫ����Ļ���á� */
#define IPS200_DISPLAY_ENABLE 1
#define IPS200_DISPLAY_ENABLE2 0


#if IPS200_DISPLAY_ENABLE
/* ����ͷ50֡ʱÿ5֡ˢ��һ�α�������ֵ����������ˢ���������档 */
#define ENCODER_DISPLAY_DIV 5U
#endif
/* ������֡ȷ�Ͽ��˳���֡���У�ȷ�Ϻ󱣳�ȫ��8֡��ͣ��Խ���յ��ߡ� */

/* ������ͣ���ӳ�֡������⵽�����ߺ��ӳ�N֡��ͣ����
   �ó�ģͨ��������(�յ�)����ͣ�¡�50fps��1֡=20ms�� */
#define ZEBRA_STOP_DELAY_FRAMES  15

/* �����߼�⵽������ͣ����������Ҫȷ�Ϻ��ӳ�֡ */

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
    Camera_CompressInit();           /* ͼ��ѹ����ʼ�� (������һ��) */

#if IPS200_DISPLAY_ENABLE
    /*
     * IPS200��ʼ������CPU0, ������ͷ����ͬһ��,
     * ����˫��ͬʱ����SPI���³�ͻ.
     */
    IPS200_Init();
#endif

    cpu_wait_event_ready();

#if IPS200_DISPLAY_ENABLE
    ips200_full(RGB565_BLACK);  /* ����Ϊ��ɫ */
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

            /* ---- ͼ������ˮ��: ��ֵ�� -> Ԫ��ʶ�� ---- */
            Flag_init();
            Get_BaseLine();
            Get_AllLine();
            Scan_Element();
            Element_Handle();

            /* �����߼����ӳ�ͣ������������N֡���ó�����ͨ���������յ� */
            {
                static uint8_t zebra_triggered = 0;  /* �Ƿ��Ѵ��������� */
                static uint8_t zebra_delay_cnt = 0;  /* �������ۼ�֡�� */

                if (ImageFlag.Zebra_Flag != 0 && zebra_triggered == 0
                 && StopRequest == 0U)
                {
                    zebra_triggered = 1;     /* ���津��״̬ */
                    zebra_delay_cnt = 0;     /* ��ʼ���� */
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

            /* ---- ���� Err (ͼ��ƫ��) �� CPU1 ʹ�� ---- */
            /* ǰհ3��ƽ����Err�������ص�λ����CPU1��PD����һ�¡� */
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
#if IPS200_DISPLAY_ENABLE2
            /* ÿֻ֡��QSPI2�Ĵ�������ֱˢ����ֹ�ص����ֽڵȴ��ĵ�����ʾ·���� */
            /* ��ʾ��ֵ��ͼ����������(��)�복������(��)�����ڶ�ֵͼ�� */
            Camera_ShowBinaryFast();
            /* �������ߣ���ɫ���ߣ��������ֵͼ����(xo+Center, row)��ÿ֡��Ȼ���� */
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
            /* �������ߣ���ɫ���߹̶���ͼ��ˮƽ���� */
            ips200_draw_line(94, 0, 94, 59, RGB565_RED);
#endif



#if IPS200_DISPLAY_ENABLE
            if (++encoder_display_cnt >= ENCODER_DISPLAY_DIV)
            {
                encoder_display_cnt = 0U;
                ips200_show_int(58U, 128U, (int32)EncLeft, 5U);
                ips200_show_int(58U, 144U, (int32)EncRight, 5U);
                ips200_show_int(58U, 170U, (int32)Err, 5U);
                /* Բ������ͽ׶α�־λ��ʾ�����ڵ���״̬���л� */
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

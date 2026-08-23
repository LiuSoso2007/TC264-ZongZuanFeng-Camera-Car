#include "Camera.h"
#include "Shared.h"
/* 圆环识别与状态机相关变量 */
static uint16 s_ring_state_frames = 0U;
static uint8 s_ring_confirm_count = 0U;
static uint8 s_ring_feature_count = 0U;
static uint8 s_ring_state7_latched = 0U;       /* 到达状态7后保持为1，开放后续斑马线停车。 */
static uint8 s_ring_exit_loss_seen = 0U;     /* 出环时是否已观察到对侧丢线 */
static int s_ring_entry_corner_row = -1;
static int s_ring_entry_corner_col = -1;
static uint8 s_ring_edge_squeezed = 0U;
static uint8 s_ring_edge_released = 0U;
static int s_ring_prev_valley_row = -1;
static int s_ring_exit1_corner1_row = -1;
static int s_ring_exit1_corner1_col = -1;
static int s_ring_exit1_corner2_row = -1;
static int s_ring_exit1_corner2_col = -1;
static uint8 s_ring_exit1_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
static uint8 s_ring_exit2_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
static int s_ring_recovery_valley_row = -1;     /* RECOVERY出环拐点 */
static int s_ring_recovery_valley_col = -1;
static int s_ring_prev_recovery_valley_row = -1; /* RECOVERY上一帧出环拐点行 */
static uint16 s_ring_exit_cooldown = 0U;           /* 上一帧谷底行号, APPROACH阶段用 */
volatile int g_corner_black_max = 0;
volatile int g_bottom_black_width = 0;
volatile int g_ring_miss_cnt = 0;
volatile uint8 g_left_jump_count = 0;
volatile uint8 g_right_jump_count = 0;
static int s_left_jump_other_lost_count = 0;   /* 左侧断点同行右侧丢线数 */
static int s_right_jump_other_lost_count = 0;  /* 右侧断点同行左侧丢线数 */
volatile int g_approach_valley_row = -99;
volatile uint8 g_edge_squeezed_dbg = 0;
volatile uint8 g_ring_phase_dbg = 0;
/* 图像与元素处理全局变量 */
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
int16_t g_ZebraSum = 0;
ImageDealDatatypedef ImageDeal[LCDH];
ImageStatustypedef ImageStatus;
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

/* 摄像头初始化 */
void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

/* 查询一帧图像是否采集完成 */
uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

/* 获取原始图像指针 */
uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/* 压缩图像指针初始化，建立MT9V03X到LCD尺寸的映射 */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/* 灰度直方图+OTSU大津法计算自适应阈值 */
/* 返回自适应二值化阈值 */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row)
{
    uint32 hist[256] = {0};
    uint16 i, j;
    uint16 t;
    uint32 total = (uint32)col * row;
    uint64 totalSum = 0;
    uint32 w0 = 0;
    uint64 sum0 = 0;
    float maxVar = 0.0f;
    uint8 bestThr = 128;
    uint8 pmin = 255, pmax = 0;
    uint8 range;

    /* 第一遍扫描全图，找到最小和最大灰度 */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++) {
            uint8 v = *image[i][j];
            if (v < pmin) pmin = v;
            if (v > pmax) pmax = v;
        }
    range = pmax - pmin;
    if (range > 30) {
        for (i = 0; i < row; i++)
            for (j = 0; j < col; j++) {
                uint8 v = (uint8)(((uint16)(*image[i][j] - pmin) * 255U) / range);
                hist[v]++;
            }
    } else {
        for (i = 0; i < row; i++)
            for (j = 0; j < col; j++)
                hist[*image[i][j]]++;
    }

    /* OTSU算法核心：遍历阈值，最大化类间方差 */
    for (t = 0; t < 256; t++)
        totalSum += (uint64)t * hist[t];

    for (t = 0; t < 255; t++) {
        w0 += hist[t];
        if (w0 == 0) continue;
        if (w0 == total) break;
        sum0 += (uint64)t * hist[t];
        float m0 = (float)sum0 / (float)w0;
        float m1 = (float)(totalSum - sum0) / (float)(total - w0);
        float var = (float)w0 * (float)(total - w0) * (m0 - m1) * (m0 - m1);
        if (var > maxVar) {
            maxVar = var;
            bestThr = (uint8)t;
        }
    }

    if (range > 30) {
        bestThr = (uint8)(pmin + ((uint16)bestThr * range) / 255U);
    }

    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    bestThr += OTSU_BIAS;
    return bestThr;
}

/* 生成二值图像：同步等待DMA后按OTSU阈值二值化 */
void Camera_GetBinaryImage(void) {
    { volatile uint16 _sync; for (_sync = 0; _sync < 5000U; _sync++) {} }
    uint8 thr = Camera_OTSU_GetThreshold(Image_Use, LCDW, LCDH);
    Camera_Threshold = thr;
    uint8 i, j;
    for (i = 0; i < LCDH; i++)
        for (j = 0; j < LCDW; j++)
            Pixle[i][j] = (*Image_Use[i][j] > thr) ? 1 : 0;
}

/* 显示二值图像到IPS200 */
void Camera_ShowBinaryImage(void) {
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 0, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
}

/* 在IPS200上绘制中线与赛道边界 */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);

    ips200_draw_line(94, 0, 94, 119, RGB565_RED);

    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

/* 摄像头调试画面：原图、二值图与元素状态 */
void Camera_ShowDebug(void) {
    uint16 xo;

    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);

    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    Camera_ShowElementStatus();

    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}

/* 基础巡线：从起始行向下逐行扫描左右边界 */
void Get_BaseLine(void)
{
    uint8 *PicTemp;
    int   Xsite;
    int   row;

    PicTemp = Pixle[SCAN_BASE_START_ROW];

    for (Xsite = ImageSensorMid; Xsite < (LCDW - 1); Xsite++)
    {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
        {
            ImageDeal[SCAN_BASE_START_ROW].RightBorder = Xsite;
            break;
        }
        else if (Xsite == (LCDW - 2))
        {
            ImageDeal[SCAN_BASE_START_ROW].RightBorder = LCDW - 1;
            break;
        }
    }

    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)
    {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
        {
            ImageDeal[SCAN_BASE_START_ROW].LeftBorder = Xsite;
            break;
        }
        else if (Xsite == 1)
        {
            ImageDeal[SCAN_BASE_START_ROW].LeftBorder = 0;
            break;
        }
    }

    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;

    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        for (Xsite = ImageDeal[row + 1].Center; Xsite < (LCDW - 1); Xsite++)
        {
            if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
            {
                ImageDeal[row].RightBorder = Xsite;
                break;
            }
            else if (Xsite == (LCDW - 2))
            {
                ImageDeal[row].RightBorder = LCDW - 1;
                ImageDeal[row].IsRightFind = 'F';
                break;
            }
        }

        for (Xsite = ImageDeal[row + 1].Center; Xsite > 0; Xsite--)
        {
            if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
            {
                ImageDeal[row].LeftBorder = Xsite;
                break;
            }
            else if (Xsite == 1)
            {
                ImageDeal[row].LeftBorder = 0;
                ImageDeal[row].IsLeftFind = 'F';
                break;
            }
        }

        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }
}

/* 单行边界跳变点检测：T=跳变，W=全白，H=全黑 */
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;

    LimitL(L);
    LimitH(H);

    if (type == 'L')
    {
        for (i = H; i >= L; i--)
        {
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;
                Q->type  = 'T';
                break;
            }
            else if (i == L)
            {
                if (*(p + (L + H) / 2) != 0)
                {
                    Q->point = (L + H) / 2;
                    Q->type  = 'W';
                }
                else
                {
                    Q->point = (L + H) / 2;
                    Q->type  = 'H';
                }
                break;
            }
        }
    }
    else if (type == 'R')
    {
        for (i = L; i <= H; i++)
        {
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;
                Q->type  = 'T';
                break;
            }
            else if (i == H)
            {
                if (*(p + (L + H) / 2) != 0)
                {
                    Q->point = (L + H) / 2;
                    Q->type  = 'W';
                }
                else
                {
                    Q->point = (L + H) / 2;
                    Q->type  = 'H';
                }
                break;
            }
        }
    }
}

/* 全图巡线：逐行检测左右边界并统计丢失与白色行 */
void Get_AllLine(void)
{
    uint8 *PicTemp;
    int   row;
    int   IntervalLow, IntervalHigh;
    int   i;

    ImageStatus.OFFLine          = 2;
    ImageStatus.Miss_Left_lines  = 0;
    ImageStatus.Miss_Right_lines = 0;
    ImageStatus.WhiteLine        = 0;
    ImageStatus.WhiteLine_L      = 0;
    ImageStatus.WhiteLine_R      = 0;
    ImageStatus.OFFLineBoundary  = 0;
    ImageStatus.Det_True         = 0;

    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];
        PicTemp = Pixle[row];

        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        if (JumpPoint[0].type == 'W')
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;
            ImageStatus.Miss_Left_lines++;
        }
        else
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;
        }

        if (JumpPoint[1].type == 'W')
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder;
            ImageStatus.Miss_Right_lines++;
        }
        else
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;
        }

        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }

        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))
        {
            ImageStatus.OFFLine = row;
            break;
        }

        if (ImageDeal[row].Wide <= 8)
        {
            ImageStatus.OFFLine = row + 1;
            break;
        }
        else if (ImageDeal[row].RightBorder <= 12
              || ImageDeal[row].LeftBorder >= 82)
        {
            ImageStatus.OFFLine = row + 1;
            break;
        }
    }
}

/* 直道半宽表：按行给出赛道一半宽度 */
const uint8 Half_Road_Wide[60] = {
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* 元素标志全局变量 */
ImageFlagtypedef ImageFlag;

/* 直线拟合误差：dir=1检查左边线，dir=2检查右边线 */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1:
        k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder)
          / (float)(start - end);
        for (i = 0; i < (int)(end - start); i++)
        {
            Err = (ImageDeal[start].LeftBorder + k * i
                 - ImageDeal[i + start].LeftBorder);
            Sum += Err * Err;
        }
        S = Sum / (float)(end - start);
        break;
    case 2:
        k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder)
          / (float)(start - end);
        for (i = 0; i < (int)(end - start); i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i
                 - ImageDeal[i + start].RightBorder);
            Sum += Err * Err;
        }
        S = Sum / (float)(end - start);
        break;
    }
    return S;
}

/* 长直道判定 */
void Straight_long_judge(void)
{
    if (ImageFlag.Bend_Road || ImageFlag.Zebra_Flag
        || ImageFlag.image_element_rings)
        return;

    if ((Straight_Judge(1, 10, SCAN_BASE_START_ROW) < 1.0f)
     && (Straight_Judge(2, 10, SCAN_BASE_START_ROW) < 1.0f)
     && ImageStatus.OFFLine < 3
     && ImageStatus.Miss_Left_lines < 2
     && ImageStatus.Miss_Right_lines < 2)
    {
        ImageFlag.straight_long = 1;
    }
}

/* 长直道处理：检测退出条件 */
void Straight_long_handle(void)
{
    if (!ImageFlag.straight_long) return;

    if ((Straight_Judge(1, 10, SCAN_BASE_START_ROW) > 1.0f)
     || (Straight_Judge(2, 10, SCAN_BASE_START_ROW) > 1.0f)
     || ImageStatus.OFFLine >= 3
     || ImageStatus.Miss_Left_lines >= 2
     || ImageStatus.Miss_Right_lines >= 2)
    {
        ImageFlag.straight_long = 0;
    }
}

/* 斜入直道判定 */
void Straight_xie_judge(void)
{
    float S, Sum, Err, midd_k;
    int i;

    if (ImageFlag.Zebra_Flag != 0 || ImageFlag.image_element_rings != 0)
        return;

    ImageFlag.straight_xie = 0;

    if (ImageStatus.OFFLine >= 10) return;

    midd_k = (float)(ImageDeal[SCAN_BASE_END_ROW].Center - ImageDeal[ImageStatus.OFFLine + 1].Center)
           / (float)(SCAN_BASE_END_ROW - ImageStatus.OFFLine - 1);
    Sum = 0.0f;
    for (i = 0; i < SCAN_BASE_END_ROW - ImageStatus.OFFLine - 1; i++)
    {
        Err = (ImageDeal[ImageStatus.OFFLine + 1].Center + midd_k * i
             - ImageDeal[i + ImageStatus.OFFLine + 1].Center);
        Sum += Err * Err;
    }
    S = Sum / (float)(SCAN_BASE_END_ROW - ImageStatus.OFFLine - 1);

    if (S < 1.0f && ImageStatus.OFFLine < 10
     && (ImageStatus.Miss_Left_lines > 30 || ImageStatus.Miss_Right_lines > 30))
    {
        ImageFlag.straight_xie = 1;
    }
}

/* 弯道判定 */
void Element_Judgment_Bend(void)
{
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag)
        return;
    /* 若OFFLine<5, 强制全图扫描以保证Miss计数准确 */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;

    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;
    }

    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;
    }
}

/* 弯道处理：按弯道方向推算Center */
void Element_Handle_Bend(void)
{
    int row;

    /* OFFLine过小时清空弯道标志并返回 */
    if (ImageStatus.OFFLine < 5)
        { ImageFlag.Bend_Road = 0; return; }

    if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4)
        { ImageFlag.Bend_Road = 0; return; }

if (ImageFlag.Bend_Road == 1)             /* 左弯道 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* 限幅 >= 0 */
        }
    }
else if (ImageFlag.Bend_Road == 2)        /* 右弯道 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* 限幅 <= 93 */
        }
    }
}

/* 设置圆环状态并复位相关计数器 */
static void Ring_Set_State(uint8 state)
{
    ImageFlag.image_element_rings_flag = state;
    s_ring_state_frames = 0U;
    s_ring_feature_count = 0U;

    if (state == RING_STATE_RECOVERY)
        s_ring_state7_latched = 1U;  /* 进入出环恢复阶段后，允许斑马线触发停车。 */

    if (state == RING_STATE_CONFIRM)
    {
        s_ring_confirm_count = 0U;
        s_ring_prev_valley_row = -1;  /* 新圆环不得沿用上一圆环的谷底行。 */
        s_ring_exit_loss_seen = 0U;
        s_ring_exit1_corner1_row = -1;
        s_ring_exit1_corner1_col = -1;
        s_ring_exit1_corner2_row = -1;
        s_ring_exit1_corner2_col = -1;
        s_ring_exit1_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
        s_ring_exit2_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
        s_ring_entry_corner_row = -1;
        s_ring_entry_corner_col = -1;
        s_ring_recovery_valley_row = -1;
        s_ring_recovery_valley_col = -1;
        s_ring_prev_recovery_valley_row = -1;
    }
    else if (state == RING_STATE_INSIDE)
    {
        s_ring_exit_loss_seen = 0U;
        s_ring_exit1_corner1_row = -1;
        s_ring_exit1_corner1_col = -1;
        s_ring_exit1_corner2_row = -1;
        s_ring_exit1_corner2_col = -1;
        s_ring_exit1_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
        s_ring_exit2_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
    }
    else if (state == RING_STATE_RECOVERY)
    {
        s_ring_recovery_valley_row = -1;
        s_ring_recovery_valley_col = -1;
        s_ring_prev_recovery_valley_row = -1;
    }
}

/* 清空圆环状态并启动出环冷却 */
static void Ring_Clear_State(void)
{
    ImageFlag.image_element_rings = 0;
    ImageFlag.image_element_rings_flag = RING_STATE_IDLE;
    ImageFlag.ring_big_small = 0;
    s_ring_state_frames = 0U;
    s_ring_confirm_count = 0U;
    s_ring_feature_count = 0U;
    s_ring_exit_loss_seen = 0U;
    s_ring_exit1_corner1_row = -1;
    s_ring_exit1_corner1_col = -1;
    s_ring_exit1_corner2_row = -1;
    s_ring_exit1_corner2_col = -1;
    s_ring_exit1_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
    s_ring_exit2_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
    s_ring_entry_corner_row = -1;
    s_ring_entry_corner_col = -1;
    s_ring_recovery_valley_row = -1;
    s_ring_recovery_valley_col = -1;
    s_ring_prev_recovery_valley_row = -1;
    s_ring_edge_squeezed = 0U;
    s_ring_edge_released = 0U;
    s_ring_exit_cooldown = 50U;  /* 出环后等待50帧，避免重复识别刚离开的圆环。 */
}

/* 拐角黑洞检测：检查左下/右下角黑色像素 */
static uint8 BlackHole_Check_Corner(uint8 direction)
{
    int row, col, black_cnt;
    int start_col, end_col;

    if (direction == 1U) { start_col = 0; end_col = 9; }
    else                 { start_col = LCDW - 10; end_col = LCDW - 1; }

    for (row = LCDH - 1; row >= LCDH - 6; row--)
    {
        black_cnt = 0;
        for (col = start_col; col <= end_col; col++)
        {
            if (Pixle[row][col] == IMG_BLACK)
                black_cnt++;
        }

        if (black_cnt > g_corner_black_max) g_corner_black_max = black_cnt;
        if (black_cnt >= 3)
            return 1;
    }
    return 0;
}
/* 底部黑洞检测：统计底部黑色区域宽度 */
static uint8 BlackHole_Check_Bottom(uint8 direction)
{
    int row, col;
    int state;
    int black_cnt;
    int start_col, end_col, step;

    for (row = LCDH - 1; row >= LCDH - 6; row--)
    {
        state = 0;
        black_cnt = 0;

        if (direction == 1U) { start_col = 0; end_col = LCDW - 1; step = 1; }
        else                 { start_col = LCDW - 1; end_col = 0; step = -1; }

        for (col = start_col; col != end_col; col += step)
        {
            if (Pixle[row][col] == IMG_WHITE)
            {
                                    g_bottom_black_width = black_cnt;
                if (state == 2 && black_cnt >= 3)
                    return 1;
                state = 1;
                black_cnt = 0;
            }
            else
            {
                if (state >= 1)
                    black_cnt++;
                if (state == 1)
                    state = 2;
            }
        }
    }
    return 0;
}

/* 上方黑洞检测：确认拐点上方存在黑色区域 */
static uint8 BlackHole_Check_Above(int inflection_row, int inflection_col)
{
    int row;

    for (row = inflection_row - 2; row > BH_BOTTOM_START_ROW + 10; row--)
    {
        if (Pixle[row][inflection_col] == IMG_WHITE
            && Pixle[row + 1][inflection_col] == IMG_BLACK)
        {
            for (; row > BH_BOTTOM_START_ROW + 5; row--)
            {
                if (Pixle[row][inflection_col] == IMG_BLACK
                    && Pixle[row + 1][inflection_col] == IMG_WHITE)
                {
                    return 1;
                }
            }
            break;
        }
    }
    return 0;
}

/* 追踪黑洞谷底：从扫描列向下/侧向跟踪黑色区域 */
static int BlackHole_Track_Valley(uint8 direction, int *valley_row, int *valley_col, int scan_start, int min_row)
{
    int row, col;
    int moved;
    int scan_col;

    scan_col = (direction == 1U) ? VALLEY_SCAN_COL_LEFT : VALLEY_SCAN_COL_RIGHT;

    for (row = scan_start; row > min_row; row--)
    {
        if (Pixle[row][scan_col] == IMG_WHITE
            && Pixle[row - 1][scan_col] == IMG_BLACK)
        {
            col = scan_col;

            if (direction == 1U)
            {
                for (; col + 1 < LCDW - 1; col++)
                {
                    if (Pixle[row][col + 1] == IMG_WHITE) break;
                }
                do {
                    moved = 0;
                    if (col + 1 < LCDW - 1 && row + 1 < LCDH - 1
                        && Pixle[row][col + 1] == IMG_BLACK)
                    {
                        col++;
                        moved = 1;
                    }
                    if (row + 1 < LCDH - 1
                        && Pixle[row + 1][col] == IMG_BLACK)
                    {
                        row++;
                        moved = 1;
                    }
                } while (moved);
            }
            else
            {
                for (; col - 1 > 0; col--)
                {
                    if (Pixle[row][col - 1] == IMG_WHITE) break;
                }
                do {
                    moved = 0;
                    if (col - 1 > 0 && row + 1 < LCDH - 1
                        && Pixle[row][col - 1] == IMG_BLACK)
                    {
                        col--;
                        moved = 1;
                    }
                    if (row + 1 < LCDH - 1
                        && Pixle[row + 1][col] == IMG_BLACK)
                    {
                        row++;
                        moved = 1;
                    }
                } while (moved);
            }

            if (row > min_row && row < VALLEY_MAX_ROW
                && col > 0 && col < LCDW - 1)
            {
                *valley_row = row;
                *valley_col = col;
                return 1;
            }
            continue;
        }
    }
    return 0;
}

/* 稳定赛道判定：丢线少且边界平直 */
static uint8 Ring_Is_Stable_Road(void)
{
    return (uint8)(ImageStatus.OFFLine <= 2
                && ImageStatus.Miss_Left_lines < 4
                && ImageStatus.Miss_Right_lines < 4
                && Straight_Judge(1, 5, SCAN_BASE_END_ROW) < 2.0f
                && Straight_Judge(2, 5, SCAN_BASE_END_ROW) < 2.0f);
}

/* 圆环候选判定：底部/拐角黑洞加对侧丢线条件 */
static uint8 Ring_Is_Candidate(uint8 direction)
{
    if (ImageStatus.OFFLine > 10)
        return 0U;
    if ((!BlackHole_Check_Corner(direction) && !BlackHole_Check_Bottom(direction)))
        return 0U;
    if (direction == 1U)
        return (uint8)(ImageStatus.Miss_Left_lines >= 5);
    if (direction == 2U)
        return (uint8)(ImageStatus.Miss_Right_lines >= 5);
    return 0U;
}

/* 查找圆环谷底点 */
static int Ring_Find_Valley_Point(uint8 direction, int *valley_col)
{
    int valley_row = -1;
    int vcol = -1;

    if (BlackHole_Track_Valley(direction, &valley_row, &vcol, VALLEY_SCAN_START_ROW, VALLEY_MIN_ROW) == 0)
    {
        *valley_col = -1;
        return -1;
    }
    *valley_col = vcol;
    return valley_row;
}

static int Ring_Check_Border_Jump(uint8 direction, int threshold, int min_row,
                                  int max_row, int *other_lost_count);

/* ---- ENTRY阶段方案: 从对侧边缘横向扫描入口拐点 ----
 * 从下往上逐行扫描，从对侧边缘出发向环岛方向扫，
 * 找黑色区域的远侧边界跳变点作为拐点，记录跳变点与起点的横向距离。
 * 相邻两行距离差绝对值>10时，取靠上(行数小)那行的跳变点作为入口拐点。
 * direction=1(左圆环): 从右边缘向左扫，找黑色区域左边缘(黑->白)，左边的跳变点作为拐点
 * direction=2(右圆环): 从左边缘向右扫，找黑色区域右边缘(黑->白)，右边的跳变点作为拐点
 * 返回: 1=找到拐点, 0=未找到; 拐点坐标通过corner_row/corner_col输出
 */
static uint8 Ring_Find_Entry_Corner(uint8 direction,
                                    int *corner_row, int *corner_col)
{
    int row;
    int scan_min_row;
    int prev_dist = -1, curr_dist;
    int jump_col;
    int col;
    int upper_other_lost_count;
    uint8 in_black;

    /* ENTRY固定扫描59~15行，RECOVERY继续服从本帧巡线截止行。 */
    scan_min_row = ImageStatus.OFFLine + 1;
    for (row = SCAN_BASE_START_ROW; row >= scan_min_row; row--)
    {
        if (direction == 2U)
        {
            /* 跳变点位置(入环拐角列) */
            jump_col = LCDW - 1;
            in_black = 0U;
            for (col = 0; col < LCDW - 1; col++)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col + 1] == IMG_WHITE)
                    {
        jump_col = col;       /* 记录跳变列 */
                        break;
                    }
                }
                else
                {
                    if (Pixle[row][col] == IMG_WHITE && Pixle[row][col + 1] == IMG_BLACK)
                    {
                        in_black = 1U;       /* 进入黑色区域 */
                    }
                }
            }
        curr_dist = jump_col;         /* 当前跳变距离 = 跳变列 */
        }
        else
        {
            /* 左圆环：从右向左扫描黑色区域左边缘 */
            jump_col = 0;
            in_black = 0U;
            for (col = LCDW - 1; col > 0; col--)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col - 1] == IMG_WHITE)
                    {
        jump_col = col;       /* 记录跳变列 */
                        break;
                    }
                }
                else
                {
                    if (Pixle[row][col] == IMG_WHITE && Pixle[row][col - 1] == IMG_BLACK)
                    {
                        in_black = 1U;       /* 进入黑色区域 */
                    }
                }
            }
        curr_dist = (LCDW - 1) - jump_col;  /* 当前跳变距离(从右边算) */
        }

        /* 比较相邻行跳变距离，距离突变处即为入口拐点 */
        if (prev_dist >= 0
            && (curr_dist - prev_dist > 10 || prev_dist - curr_dist > 10))
        {
            if (row - 2 <= ImageStatus.OFFLine)
            {
                prev_dist = curr_dist;
                continue;
            }
            *corner_row = row;
            *corner_col = jump_col;
            return 1U;
        }
        prev_dist = curr_dist;
    }
    return 0U;
}

/* 按RECOVERY规则在指定行查找第一个有效的白变黑点。 */
static uint8 Ring_Find_Recovery_Corner_On_Row(uint8 direction, int row,
                                              int *corner_col)
{
    int col;

    *corner_col = -1;

    if (direction == 2U)
    {
        /* 右环：左边缘为白时略过，从左向右取第一个白变黑点。 */
        if (Pixle[row][0] == IMG_WHITE)
            return 0U;

        for (col = 1; col < LCDW; col++)
        {
            if (Pixle[row][col - 1] == IMG_WHITE
                && Pixle[row][col] == IMG_BLACK)
            {
                if (col > RING_RECOVERY_CORNER_COL_LIMIT)
                {
                    *corner_col = col;
                    return 1U;
                }
                return 0U;
            }
        }
    }
    else if (direction == 1U)
    {
        /* 左环：右边缘为白时略过，从右向左取第一个白变黑点。 */
        if (Pixle[row][LCDW - 1] == IMG_WHITE)
            return 0U;

        for (col = LCDW - 2; col >= 0; col--)
        {
            if (Pixle[row][col + 1] == IMG_WHITE
                && Pixle[row][col] == IMG_BLACK)
            {
                if (col < LCDW - 1 - RING_RECOVERY_CORNER_COL_LIMIT)
                {
                    *corner_col = col;
                    return 1U;
                }
                return 0U;
            }
        }
    }

    return 0U;
}

/* RECOVERY阶段从50行向上扫描到10行，并用紧邻上一行确认列连续性。 */
static uint8 Ring_Find_Recovery_Corner(uint8 direction,
                                       int *corner_row, int *corner_col)
{
    int row;
    int current_col, upper_col;

    *corner_row = -1;
    *corner_col = -1;

    for (row = RING_RECOVERY_SCAN_MAX_ROW;
         row >= RING_RECOVERY_SCAN_MIN_ROW; row--)
    {
        if (!Ring_Find_Recovery_Corner_On_Row(direction, row, &current_col))
            continue;

        /* 上一行找不到拐点或两行列差超过1，都放弃当前候选继续向上。 */
        if (!Ring_Find_Recovery_Corner_On_Row(direction, row - 1, &upper_col)
            || upper_col - current_col > RING_RECOVERY_CORNER_MAX_COL_DIFF
            || current_col - upper_col > RING_RECOVERY_CORNER_MAX_COL_DIFF)
            continue;

        *corner_row = row;
        *corner_col = current_col;
        return 1U;
    }

    return 0U;
}

/* ---- 统计相邻行边界断点数量 ---- */
static int Ring_Check_Border_Jump(uint8 direction, int threshold, int min_row, int max_row,
                                  int *other_lost_count)
{
    int row;
    int prev_col = -1, curr_col;
    int count = 0;

    *other_lost_count = 0;

    for (row = max_row; row >= min_row; row--)
    {
        if (direction == 1U)
        {
            if (ImageDeal[row].IsLeftFind != 'T')
            {
                if (prev_col >= 0 && ImageDeal[row + 1].IsLeftFind == 'T')
                {
                    curr_col = ImageDeal[row + 1].LeftBorder;
                    if (curr_col - prev_col > threshold || curr_col - prev_col < -threshold)
                    {
                        count++;
                        if (ImageDeal[row + 1].IsRightFind != 'T')
                            (*other_lost_count)++;
                    }
                }
                prev_col = -1;
                continue;
            }
            curr_col = ImageDeal[row].LeftBorder;
        }
        else
        {
            if (ImageDeal[row].IsRightFind != 'T')
            {
                if (prev_col >= 0 && ImageDeal[row + 1].IsRightFind == 'T')
                {
                    curr_col = ImageDeal[row + 1].RightBorder;
                    if (curr_col - prev_col > threshold || curr_col - prev_col < -threshold)
                    {
                        count++;
                        if (ImageDeal[row + 1].IsLeftFind != 'T')
                            (*other_lost_count)++;
                    }
                }
                prev_col = -1;
                continue;
            }
            curr_col = ImageDeal[row].RightBorder;
        }

        if (prev_col >= 0)
        {
            if (curr_col - prev_col > threshold || curr_col - prev_col < -threshold)
            {
                count++;
                /* 目标侧本行是断点时，把同行另一侧丢线计入联合限制。 */
                if ((direction == 1U && ImageDeal[row].IsRightFind != 'T')
                    || (direction == 2U && ImageDeal[row].IsLeftFind != 'T'))
                    (*other_lost_count)++;
            }
        }
        prev_col = curr_col;
    }
    return count;
}

/* APPROACH阶段谷底检测：先判断第50行是否贴边，再找外移后的回落点 */
static int Ring_Find_Approach_Valley(uint8 direction, int *valley_col)
{
    int row;
    int prev_col = 0, curr_col;
    uint8 moved_away = 0U;
    uint8 is_lost;

    int row50_at_edge;
    if (direction == 1U)
    {
        row50_at_edge = (ImageDeal[50].IsLeftFind != 'T')
                      || (ImageDeal[50].LeftBorder <= 3);
    }
    else
    {
        row50_at_edge = (ImageDeal[50].IsRightFind != 'T')
                      || (ImageDeal[50].RightBorder >= LCDW - 4);
    }

    /* ENTRY: 入环阶段 - 确认拐角行有效后进入环中 */
    if (row50_at_edge)
    {
        uint8 phase = 1U;
        s_ring_edge_squeezed = 1U;
        g_ring_phase_dbg = 1U;
        moved_away = 0U;

        for (row = 50; row >= 11; row--)
        {
            if (direction == 1U)
            {
                if (ImageDeal[row].IsLeftFind != 'T') { continue; }
                curr_col = ImageDeal[row].LeftBorder;
            }
            else
            {
                if (ImageDeal[row].IsRightFind != 'T') { continue; }
                curr_col = ImageDeal[row].RightBorder;
            }

            int at_edge = (direction == 1U) ? (curr_col <= 4) : (curr_col >= LCDW - 5);

            if (phase == 1U)
            {
                if (!at_edge) { phase = 2U; prev_col = curr_col; }
                continue;
            }

            int diff = curr_col - prev_col;
            if (!moved_away)
            {
                if ((direction == 1U && diff > 0) || (direction == 2U && diff < 0))
                    moved_away = 1U;
            }
            else
            {
                if ((direction == 1U && diff <= 0) || (direction == 2U && diff >= 0))
                {
                    /* APPROACH: 接近阶段，检测到边界回落点后进入入环 */
                    if (row - 1 >= 5)
                    {
                        int k;
                        is_lost = 0U;
                        for (k = 1; k <= 5; k++)
                        {
                            if (row - k < 5) break;
                            if (direction == 1U)
                            {
                                if (ImageDeal[row - k].IsLeftFind != 'T') is_lost++;
                            }
                            else
                            {
                                if (ImageDeal[row - k].IsRightFind != 'T') is_lost++;
                            }
                        }
                        if (is_lost >= 3)
                        {
                            /* 假拐点仅跳过当前候选，保留外移趋势以继续向上寻找真拐点。 */
                            prev_col = curr_col;
                            continue;
                        }
                    }
                    *valley_col = prev_col;
                    return row;
                }
            }
            prev_col = curr_col;
        }
        return -1;
    }

    /* CONFIRM: 确认阶段 - 持续检测特征帧数后进入接近 */
    if (s_ring_edge_squeezed)
    {
        g_ring_phase_dbg = 2U;
        moved_away = 0U;

        for (row = 50; row >= 5; row--)
        {
            if (direction == 1U)
            {
                if (ImageDeal[row].IsLeftFind != 'T') { continue; }
                curr_col = ImageDeal[row].LeftBorder;
            }
            else
            {
                if (ImageDeal[row].IsRightFind != 'T') { continue; }
                curr_col = ImageDeal[row].RightBorder;
            }

            if (row == 50) { prev_col = curr_col; continue; }

            int diff = curr_col - prev_col;
            if (!moved_away)
            {
                if ((direction == 1U && diff > 0) || (direction == 2U && diff < 0))
                    moved_away = 1U;
            }
            else
            {
                if ((direction == 1U && diff <= 0) || (direction == 2U && diff >= 0))
                {
                    /* INSIDE: 环中阶段，等待边界恢复稳定后出环 */
                    if (row - 1 >= 5)
                    {
                        int k;
                        is_lost = 0U;
                        for (k = 1; k <= 5; k++)
                        {
                            if (row - k < 5) break;
                            if (direction == 1U)
                            {
                                if (ImageDeal[row - k].IsLeftFind != 'T') is_lost++;
                            }
                            else
                            {
                                if (ImageDeal[row - k].IsRightFind != 'T') is_lost++;
                            }
                        }
                        if (is_lost >= 3)
                        {
                            /* 假拐点仅跳过当前候选，保留外移趋势以继续向上寻找真拐点。 */
                            prev_col = curr_col;
                            continue;
                        }
                    }
                    *valley_col = prev_col;
                    return row;
                }
            }
            prev_col = curr_col;
        }
        return -1;
    }

    g_ring_phase_dbg = 0U;
    return -1;
}

/* 出环特征检测：对侧边界丢线或上方存在黑洞 */
static uint8 Ring_Has_Exit_Feature(uint8 direction)
{
    int row;

    if ((direction == 1U && ImageStatus.Miss_Right_lines > 4)
        || (direction == 2U && ImageStatus.Miss_Left_lines > 4))
        return 0U;

    for (row = SCAN_BASE_START_ROW - 1; row > 5; row--)
    {
        if (direction == 1U
            && ImageDeal[row].IsRightFind == 'T'
            && ImageDeal[row - 1].IsRightFind != 'T'
            && ImageDeal[row - 2].IsRightFind != 'T')
        {
            if (BlackHole_Check_Above(row, ImageDeal[row].RightBorder))
                return 1U;
        }
        if (direction == 2U
            && ImageDeal[row].IsLeftFind == 'T'
            && ImageDeal[row - 1].IsLeftFind != 'T'
            && ImageDeal[row - 2].IsLeftFind != 'T')
        {
            if (BlackHole_Check_Above(row, ImageDeal[row].LeftBorder))
                return 1U;
        }
    }
    return Ring_Is_Stable_Road();
}

/* 线性补线并同步更新ImageDeal边界与中心 */
static void Ring_DrawAndUpdate(uint8 direction, int s_row, int s_col,
                               int e_row, int e_col, uint8 border_side)
{
    int row, col;
    float k;
    int b, r_start, r_end;

    if (s_row < e_row) { r_start = s_row; r_end = e_row; }
    else               { r_start = e_row; r_end = s_row; }

    if (s_row != e_row)
    {
        k = (float)(e_col - s_col) / (float)(e_row - s_row);
        b = s_col - (int)(k * s_row);
    }
    else { k = 0.0f; b = s_col; }

    for (row = r_start; row <= r_end; row++)
    {
        col = (int)(k * row) + b;
        if (col >= 0 && col < LCDW)
            Pixle[row][col] = IMG_WHITE;

        if (row <= SCAN_BASE_START_ROW && row > ImageStatus.OFFLine)
        {
            if (border_side == 'L')
            {
                ImageDeal[row].LeftBorder = col;
                LimitL(ImageDeal[row].LeftBorder);
            }
            else
            {
                ImageDeal[row].RightBorder = col;
                LimitH(ImageDeal[row].RightBorder);
            }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder
                                   + ImageDeal[row].RightBorder) / 2;
        }
    }
}

/* 查找EXIT1阶段的两个出环拐点 */
static uint8 Ring_Find_Exit1_Corners(uint8 direction,
    int *corner1_row, int *corner1_col,
    int *corner2_row, int *corner2_col)
{
    int row, col;
    int prev_col;
    int increasing_seen;
    *corner1_row = -1; *corner1_col = -1;
    *corner2_row = -1; *corner2_col = -1;

    prev_col = -1;
    increasing_seen = 0;
    for (row = 55; row >= 15; row--)
    {
        if (direction == 2U) col = ImageDeal[row].LeftBorder;
        else                 col = ImageDeal[row].RightBorder;
        if (col < 1 || col >= LCDW - 1) continue;
        if (prev_col >= 0)
        {
            if (direction == 2U)
            {
                if (col > prev_col + 1) increasing_seen = 1;
                if (increasing_seen && col < prev_col - 1)
                { if (prev_col < 60) { *corner1_row = row + 1; *corner1_col = prev_col; break; } }
            }
            else
            {
                if (col < prev_col - 1) increasing_seen = 1;
                if (increasing_seen && col > prev_col + 1)
                { if (prev_col > 33) { *corner1_row = row + 1; *corner1_col = prev_col; break; } }
            }
        }
        prev_col = col;
    }

    for (row = 45; row >= 1; row--)
    {
        if (direction == 2U)
        {
            for (col = LCDW - 1; col > LCDW - 8; col--)
            {
                /* EXIT2是侧边黑区下端点，下面必须连续为白色赛道。 */
                if (Pixle[row][col] == IMG_BLACK
                    && Pixle[row + 1][col] == IMG_WHITE
                    && Pixle[row + 2][col] == IMG_WHITE)
                { *corner2_row = row; *corner2_col = col; break; }
            }
        }
        else
        {
            for (col = 0; col < 8; col++)
            {
                if (Pixle[row][col] == IMG_BLACK
                    && Pixle[row + 1][col] == IMG_WHITE
                    && Pixle[row + 2][col] == IMG_WHITE)
                { *corner2_row = row; *corner2_col = col; break; }
            }
        }
        if (*corner2_row >= 0) break;
    }
    if (*corner1_row >= 0 && *corner2_row >= 0) return 1U;
    return 0U;
}

/* 更新关键点并限制旧坐标最多保留指定帧数。 */
/* 更新出环关键点，允许跨帧保持最多RING_EXIT_POINT_HOLD_FRAMES帧 */
static void Ring_Update_Exit_Point(int row, int col,
    int *cached_row, int *cached_col, uint8 *miss_frames)
{
    if (row >= 0)
    {
        *cached_row = row;
        *cached_col = col;
        *miss_frames = 0U;
    }
    else if (*miss_frames <= RING_EXIT_POINT_HOLD_FRAMES)
    {
        (*miss_frames)++;
        if (*miss_frames > RING_EXIT_POINT_HOLD_FRAMES)
        {
            *cached_row = -1;
            *cached_col = -1;
        }
    }
}

/* 按圆环状态返回中心偏移量 */
static int Ring_Get_Fill_Offset(uint8 ring_state)
{
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
    case RING_STATE_APPROACH: return FILL_ENTRY_OFFSET;
    case RING_STATE_ENTRY:    return FILL_ENTRY_OFFSET;
    case RING_STATE_INSIDE:   return FILL_INSIDE_OFFSET;
    case RING_STATE_EXIT1:    return FILL_EXIT1_OFFSET;
    case RING_STATE_EXIT2:    return FILL_EXIT2_OFFSET;
    case RING_STATE_RECOVERY: return FILL_RECOVERY_OFFSET;
    default:                  return 0;
    }
}

/* RECOVERY按出环拐点重建内侧边线，左右圆环完全镜像。 */
static void Ring_Rebuild_Recovery_Border(uint8 direction)
{
    int row, col;
    int black_segment_seen;

    if (s_ring_recovery_valley_row <= ImageStatus.OFFLine)
        return;

    if (direction == 2U)
        Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                           s_ring_recovery_valley_row, s_ring_recovery_valley_col, 'R');
    else
        Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                           s_ring_recovery_valley_row, s_ring_recovery_valley_col, 'L');

    for (row = s_ring_recovery_valley_row - 1; row > ImageStatus.OFFLine; row--)
    {
        if (direction == 2U && ImageDeal[row].RightBorder == LCDW - 1)
        {
            black_segment_seen = 0;
            for (col = LCDW - 2;
                 col >= 0 && col >= ImageDeal[row].LeftBorder; col--)
            {
                if (!black_segment_seen
                    && Pixle[row][col] == IMG_BLACK
                    && Pixle[row][col + 1] == IMG_WHITE)
                {
                    black_segment_seen = 1;
                }
                else if (black_segment_seen
                         && Pixle[row][col] == IMG_WHITE
                         && Pixle[row][col + 1] == IMG_BLACK)
                {
                    ImageDeal[row].RightBorder = col;
                    ImageDeal[row].IsRightFind = 'T';
                    break;
                }
            }
        }
        else if (direction == 1U && ImageDeal[row].LeftBorder == 0)
        {
            black_segment_seen = 0;
            for (col = 1;
                 col < LCDW && col <= ImageDeal[row].RightBorder; col++)
            {
                if (!black_segment_seen
                    && Pixle[row][col] == IMG_BLACK
                    && Pixle[row][col - 1] == IMG_WHITE)
                {
                    black_segment_seen = 1;
                }
                else if (black_segment_seen
                         && Pixle[row][col] == IMG_WHITE
                         && Pixle[row][col - 1] == IMG_BLACK)
                {
                    ImageDeal[row].LeftBorder = col;
                    ImageDeal[row].IsLeftFind = 'T';
                    break;
                }
            }
        }

        ImageDeal[row].Wide = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
    }
}

/* 按圆环状态重建巡线边界并填充Center */
static void Ring_Rebuild_Fill(uint8 direction)
{
    int row, col;
    int black_segment_seen;
    uint8 ring_state = (uint8)ImageFlag.image_element_rings_flag;
    int fill_offset = Ring_Get_Fill_Offset(ring_state);

    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
        /* CONFIRM采用与普通巡线/INSIDE一致的Center计算(左右边线中点)，
           不再做单侧推算和fill_offset偏置，避免蓝线怪异和ERR放大。 */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder
                                   + ImageDeal[row].RightBorder) / 2;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_APPROACH:
        if (s_ring_entry_corner_row >= 0)
        {
            if (direction == 1U)
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'L');
            else
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'R');
        }
        else if (s_ring_edge_squeezed)
        {
            if (direction == 1U)
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   50, 0, 'L');
            else
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   50, LCDW - 1, 'R');
        }
        else
        {
            for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    /* 左圆环与右圆环严格镜像，使用右边线向左恢复中心。 */
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] * 2 / 3 - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        break;
    case RING_STATE_ENTRY:
        /* 先连接图像底部与入环拐点。 */
        if (s_ring_entry_corner_row > 0)
        {
            if (direction == 1U)
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'R');
            else
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'L');

            /* 拐点上方重新寻找环内侧边线，左右圆环完全镜像。 */
            for (row = s_ring_entry_corner_row - 1; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 2U)
                {
                    black_segment_seen = 0;
                    for (col = ImageDeal[row].LeftBorder + 1;
                         col <= ImageDeal[row].RightBorder; col++)
                    {
                        if (!black_segment_seen
                            && Pixle[row][col] == IMG_BLACK
                            && Pixle[row][col - 1] == IMG_WHITE)
                        {
                            black_segment_seen = 1;
                        }
                        else if (black_segment_seen && Pixle[row][col] == IMG_WHITE
                                 && Pixle[row][col - 1] == IMG_BLACK)
                        {
                            ImageDeal[row].LeftBorder = col;
                            ImageDeal[row].IsLeftFind = 'T';
                            break;
                        }
                    }
                }
                else
                {
                    black_segment_seen = 0;
                    for (col = ImageDeal[row].RightBorder - 1;
                         col >= ImageDeal[row].LeftBorder; col--)
                    {
                        if (!black_segment_seen
                            && Pixle[row][col] == IMG_BLACK
                            && Pixle[row][col + 1] == IMG_WHITE)
                        {
                            black_segment_seen = 1;
                        }
                        else if (black_segment_seen && Pixle[row][col] == IMG_WHITE
                                 && Pixle[row][col + 1] == IMG_BLACK)
                        {
                            ImageDeal[row].RightBorder = col;
                            ImageDeal[row].IsRightFind = 'T';
                            break;
                        }
                    }
                }
                ImageDeal[row].Wide = ImageDeal[row].RightBorder
                                    - ImageDeal[row].LeftBorder;
                ImageDeal[row].Center = (ImageDeal[row].LeftBorder
                                       + ImageDeal[row].RightBorder) / 2;
            }
        }
        else
        {
            /* 未找到入口拐点时，按弯道宽度推算Center */
            for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    ImageDeal[row].Center = ImageSensorMid
                                          - Half_Bend_Wide[row] * 2 / 3 - fill_offset / 2;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        break;
    case RING_STATE_EXIT1:
        if (s_ring_exit1_corner1_row >= 0 && s_ring_exit1_corner2_row >= 0)
        {
            if (direction == 1U)
                Ring_DrawAndUpdate(direction, s_ring_exit1_corner1_row, s_ring_exit1_corner1_col,
                                   s_ring_exit1_corner2_row, s_ring_exit1_corner2_col, 'R');
            else
                Ring_DrawAndUpdate(direction, s_ring_exit1_corner1_row, s_ring_exit1_corner1_col,
                                   s_ring_exit1_corner2_row, s_ring_exit1_corner2_col, 'L');
        }
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = ImageDeal[row].RightBorder
                                      - Half_Bend_Wide[row] * 2 / 3 - fill_offset;
            else
                ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                      + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_EXIT2:
        /* 状态6：EXIT1离开视野后，从屏幕角连接EXIT2拐点。 */
        if (s_ring_exit1_corner2_row >= 0)
        {
            int corner2_r = s_ring_exit1_corner2_row;
            int corner2_c = s_ring_exit1_corner2_col;
            if (direction == 1U)
            {
                /* 左环从右下角(59, 93)连到corner2；写二值图真正的角列保证视觉不偏。 */
                if (corner2_r >= 0)
                {
                    Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                       corner2_r, corner2_c, 'R');
                    if (Pixle[SCAN_BASE_START_ROW][LCDW - 1] != IMG_WHITE)
                        Pixle[SCAN_BASE_START_ROW][LCDW - 1] = IMG_WHITE;
                }
            }
            else
            {
                /* 右环从左下角(59, 0)连到corner2；写二值图真正的角列保证视觉不偏。 */
                if (corner2_r >= 0)
                {
                    Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                       corner2_r, corner2_c, 'L');
                    if (Pixle[SCAN_BASE_START_ROW][0] != IMG_WHITE)
                        Pixle[SCAN_BASE_START_ROW][0] = IMG_WHITE;
                }
            }
            /* corner2上方继续用弯道半宽推center；corner2及以下行由画线已写入，不要再覆盖。 */
            for (row = corner2_r - 1; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] * 2 / 3 - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        else
        {
            /* corner2无效时兜底：全量推算。 */
            for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] * 2 / 3 - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        break;
        case RING_STATE_INSIDE:
        /* INSIDE采用与普通直道巡线一致的Center计算(左右边线中点)，
           不再使用Half_Bend_Wide弯道推算和fill_offset偏置。 */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder
                                   + ImageDeal[row].RightBorder) / 2;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_RECOVERY:
        Ring_Rebuild_Recovery_Border(direction);
        break;
    default:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = (ImageDeal[row].RightBorder > 0)
                    ? ImageDeal[row].RightBorder - Half_Bend_Wide[row] - fill_offset
                    : ImageSensorMid;
            else
                ImageDeal[row].Center = (ImageDeal[row].LeftBorder < LCDW - 1)
                    ? ImageDeal[row].LeftBorder + Half_Bend_Wide[row] + fill_offset
                    : ImageSensorMid;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    }
}

/* 圆环状态机更新 */
static void Ring_State_Update(void)
{
    uint8 direction = (uint8)ImageFlag.image_element_rings;
    int valley_col = -1;
    int valley_row;

    if (direction != 1U && direction != 2U)
    {
        Ring_Clear_State();
        return;
    }
    if (s_ring_state_frames < 65535U)
        s_ring_state_frames++;

    switch (ImageFlag.image_element_rings_flag)
    {
    case RING_STATE_CONFIRM:
        valley_row = Ring_Find_Approach_Valley(direction, &valley_col);
        g_approach_valley_row = valley_row;
        g_edge_squeezed_dbg = s_ring_edge_squeezed;
        if (valley_row >= 0)
        {
            s_ring_entry_corner_row = valley_row;
            s_ring_entry_corner_col = valley_col;
            Ring_Set_State(RING_STATE_APPROACH);
        }
        else if (s_ring_state_frames >= RING_CONFIRM_MAX_FRAMES)
            Ring_Set_State(RING_STATE_APPROACH);
        break;

    case RING_STATE_APPROACH:
        valley_row = Ring_Find_Approach_Valley(direction, &valley_col);
        g_approach_valley_row = valley_row;
        if (valley_row >= 0)
        {
            s_ring_entry_corner_row = valley_row;
            s_ring_entry_corner_col = valley_col;
            /* APPROACH完整处理1帧，谷底下移或向远处突变后进入ENTRY。 */
            if (s_ring_state_frames >= 1U
                && (valley_row > 34
                    || (s_ring_prev_valley_row >= 0
                        && s_ring_prev_valley_row - valley_row > 3)))
                Ring_Set_State(RING_STATE_ENTRY);
        }
        if(valley_row) s_ring_prev_valley_row = valley_row;
        break;

    case RING_STATE_ENTRY:
        valley_row = -1;
        if (Ring_Find_Entry_Corner(direction, &valley_row, &valley_col))
        {
            s_ring_entry_corner_row = valley_row;
            s_ring_entry_corner_col = valley_col;
        }
        /* 入环处理3帧后，找到拐点的情况下检测进入INSIDE */
        if (s_ring_state_frames >= 7U && s_ring_entry_corner_row >= 0)
        {
            /* 连续两帧拐点行数相差大于20 */
            if (valley_row >= 0 && s_ring_prev_valley_row >= 0
                && (valley_row - s_ring_prev_valley_row > 20
                    || s_ring_prev_valley_row - valley_row > 20))
            {
                Ring_Set_State(RING_STATE_INSIDE);
                break;
            }
            /* 下一帧突然找不到拐点 */
            if (valley_row < 0 && s_ring_prev_valley_row >= 0)
            {
                Ring_Set_State(RING_STATE_INSIDE);
                break;
            }
        }
        s_ring_prev_valley_row = valley_row;
        /* 超时保护 */
        if (s_ring_state_frames >= RING_ENTRY_MAX_FRAMES)
            Ring_Set_State(RING_STATE_INSIDE);
        break;

    case RING_STATE_INSIDE:
    {
        int c1r, c1c, c2r, c2c;
        (void)Ring_Find_Exit1_Corners(direction, &c1r, &c1c, &c2r, &c2c);

        /* 两个关键点允许短暂跨帧保持，避免单帧漏检导致多绕一圈。 */
        Ring_Update_Exit_Point(c1r, c1c,
            &s_ring_exit1_corner1_row, &s_ring_exit1_corner1_col,
            &s_ring_exit1_miss_frames);
        Ring_Update_Exit_Point(c2r, c2c,
            &s_ring_exit1_corner2_row, &s_ring_exit1_corner2_col,
            &s_ring_exit2_miss_frames);

        if (s_ring_exit1_corner1_row >= 0
            && s_ring_exit1_corner2_row >= 0
            && s_ring_exit1_miss_frames <= RING_EXIT_POINT_HOLD_FRAMES
            && s_ring_exit2_miss_frames <= RING_EXIT_POINT_HOLD_FRAMES)
        {
            Ring_Set_State(RING_STATE_EXIT1);
            break;
        }

        /* 超时只重新采集关键点，严禁绕过状态5、6从错误方向出环。 */
        if (s_ring_state_frames >= RING_INSIDE_MAX_FRAMES)
        {
            s_ring_state_frames = 0U;
            s_ring_exit1_corner1_row = -1;
            s_ring_exit1_corner1_col = -1;
            s_ring_exit1_corner2_row = -1;
            s_ring_exit1_corner2_col = -1;
            s_ring_exit1_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
            s_ring_exit2_miss_frames = RING_EXIT_POINT_HOLD_FRAMES + 1U;
        }
        break;
    }

    case RING_STATE_EXIT1:
    {
        int c1r, c1c, c2r, c2c;
        uint8 exit1_passed = 0U;
        (void)Ring_Find_Exit1_Corners(direction, &c1r, &c1c, &c2r, &c2c);

        Ring_Update_Exit_Point(c2r, c2c,
            &s_ring_exit1_corner2_row, &s_ring_exit1_corner2_col,
            &s_ring_exit2_miss_frames);

        if (c1r < 0 || c1r > 50
            || (direction == 2U && s_ring_exit1_corner1_col >= 0
                && c1c > s_ring_exit1_corner1_col)
            || (direction == 1U && s_ring_exit1_corner1_col >= 0
                && c1c < s_ring_exit1_corner1_col))
            exit1_passed = 1U;

        if (c1r >= 0)
        {
            s_ring_exit1_corner1_row = c1r;
            s_ring_exit1_corner1_col = c1c;
        }

        if (exit1_passed)
        {
            if (s_ring_feature_count < RING_EXIT_CONFIRM_FRAMES)
                s_ring_feature_count++;
        }
        else
        {
            s_ring_feature_count = 0U;
        }

        /* 连续确认EXIT1离开后再进入状态6，单帧漏检不切状态。 */
        if (s_ring_feature_count >= RING_EXIT_CONFIRM_FRAMES
            && s_ring_exit1_corner2_row >= 0
            && s_ring_exit2_miss_frames <= RING_EXIT_POINT_HOLD_FRAMES)
        {
            Ring_Set_State(RING_STATE_EXIT2);
            break;
        }

        if (s_ring_state_frames >= RING_EXIT1_MAX_FRAMES)
            Ring_Set_State(RING_STATE_EXIT2);
        break;
    }

    case RING_STATE_EXIT2:
    {
        int c1r, c1c, c2r, c2c;
        uint8 exit2_row_increased = 0U;
        (void)Ring_Find_Exit1_Corners(direction, &c1r, &c1c, &c2r, &c2c);

        /* 当前帧拐点行号大于15且比上一有效帧大，才进入状态7。 */
        if (c2r <= RING_EXIT2_PASS_ROW
            && s_ring_exit1_corner2_row >= 0
            && s_ring_exit2_miss_frames == 0U
            && c2r > s_ring_exit1_corner2_row)
            exit2_row_increased = 1U;

        Ring_Update_Exit_Point(c2r, c2c,
            &s_ring_exit1_corner2_row, &s_ring_exit1_corner2_col,
            &s_ring_exit2_miss_frames);

        /* EXIT2实车诊断：CB=点行号，BW=丢失帧，MS=状态累计帧。 */
        g_corner_black_max = (c2r >= 0) ? c2r : 99;
        g_bottom_black_width = (int)s_ring_exit2_miss_frames;
        g_ring_miss_cnt = (int)s_ring_state_frames;

        if (exit2_row_increased)
            Ring_Set_State(RING_STATE_RECOVERY);
        break;
    }

    case RING_STATE_RECOVERY:
        valley_row = -1;
        valley_col = -1;
        if (Ring_Find_Recovery_Corner(direction, &valley_row, &valley_col))
        {
            s_ring_feature_count = 0U;

            /* 拐点到达图像近端或左右出口边缘时，结束阶段7。 */
            if (valley_row > RING_RECOVERY_EXIT_ROW
                && (  (direction == 2U
                        && valley_col >= RING_RECOVERY_RIGHT_EXIT_COL)
                   || (direction == 1U
                        && valley_col <= RING_RECOVERY_LEFT_EXIT_COL)))
            {
                Ring_Clear_State();
                break;
            }

            s_ring_recovery_valley_row = valley_row;
            s_ring_recovery_valley_col = valley_col;
            s_ring_prev_recovery_valley_row = valley_row;
        }
        else if (s_ring_prev_recovery_valley_row < 0)
        {
            /* 仅首次找点允许等待10帧，找到过拐点后单帧漏检不结束圆环。 */
            if (s_ring_feature_count < RING_RECOVERY_ACQUIRE_MAX_FRAMES)
                s_ring_feature_count++;
            if (s_ring_feature_count >= RING_RECOVERY_ACQUIRE_MAX_FRAMES)
            {
                Ring_Clear_State();
                break;
            }
        }
        break;

    default:
        Ring_Clear_State();
        break;
    }
}

/* EXIT2切换帧及RECOVERY首次找到拐点前沿用上一帧Err，避免舵机突变。 */
/* 返回是否沿用上一帧Err */
uint8 Ring_Should_Hold_Err(void)
{
    return (uint8)(ImageFlag.image_element_rings_flag == RING_STATE_RECOVERY
        && (s_ring_state_frames == 0U
            || s_ring_prev_recovery_valley_row < 0));
}

/* 返回车辆是否已进入过出环恢复阶段，仅用于开放斑马线停车。 */
uint8 Ring_Has_Exited_Once(void)
{
    return s_ring_state7_latched;
}

/* ---- 对侧贴边行数检查 ----
 * 检查10~42行范围，边缘margin=3像素，贴边行>3则返回1
 * direction=1: 检查右边线是否太多行挤到右边缘
 * direction=2: 检查左边线是否太多行挤到左边缘
 */
static uint8 Ring_OtherSide_Too_Much_Edge(uint8 direction)
{
    int row;
    int edge_rows = 0;
    const int margin = 3;

    for (row = 30; row >= 15; row--)
    {
        if (direction == 1U)
        {
            if (ImageDeal[row].IsRightFind != 'T'
                || ImageDeal[row].RightBorder >= LCDW - margin)
                edge_rows++;
        }
        else
        {
            if (ImageDeal[row].IsLeftFind != 'T'
                || ImageDeal[row].LeftBorder <= margin)
                edge_rows++;
        }
    }
    return (uint8)(edge_rows > 3);
}

/* ---- 本侧丢线行数检查 ----
 * 扫描行 min_row~max_row（含两端），统计本侧边线丢线行数。
 * direction=1: 检查左边线(IsLeftFind!='T')行数
 * direction=2: 检查右边线(IsRightFind!='T')行数
 * 返回实际丢线行数（0~(max_row-min_row+1)）
 */
static int Ring_HomeSide_Lost_Count(uint8 direction, int min_row, int max_row)
{
    int row;
    int lost = 0;
    for (row = max_row; row >= min_row; row--)
    {
        if (direction == 1U)
        {
            if (ImageDeal[row].IsLeftFind != 'T') lost++;
        }
        else
        {
            if (ImageDeal[row].IsRightFind != 'T') lost++;
        }
    }
    return lost;
}



/* 左圆环初判 */
void Element_Judgment_Left_Rings(void)
{
    if (ImageStatus.Miss_Right_lines > 30
        || ImageStatus.OFFLine > 30
        || ImageFlag.image_element_rings)
        return;

    if (g_left_jump_count >= 3
        && g_right_jump_count + s_left_jump_other_lost_count <= RING_JUMP_OTHER_MAX
        && !Ring_OtherSide_Too_Much_Edge(1U)
        && s_ring_exit_cooldown == 0U
        && Ring_HomeSide_Lost_Count(1U, RING_HOMESIDE_MIN_ROW, RING_HOMESIDE_MAX_ROW)
           >= RING_HOMESIDE_LOST_THRESH)
    {
        g_ring_miss_cnt = ImageStatus.Miss_Left_lines;
        ImageFlag.image_element_rings = 1;
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* 右圆环初判 */
void Element_Judgment_Right_Rings(void)
{
    if (ImageStatus.Miss_Left_lines > 15
        || ImageStatus.OFFLine > 16
        || ImageFlag.image_element_rings)
        return;

    if (g_right_jump_count >= 3
        && g_left_jump_count + s_right_jump_other_lost_count <= RING_JUMP_OTHER_MAX
        && !Ring_OtherSide_Too_Much_Edge(2U)
        && s_ring_exit_cooldown == 0U
        && Ring_HomeSide_Lost_Count(2U, RING_HOMESIDE_MIN_ROW, RING_HOMESIDE_MAX_ROW)
           >= RING_HOMESIDE_LOST_THRESH)
    {
        g_ring_miss_cnt = ImageStatus.Miss_Right_lines;
        ImageFlag.image_element_rings = 2;
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* 左圆环处理 */
void Element_Handle_Left_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 1)
    {
        Ring_Rebuild_Fill(1U);
    }
}

/* 右圆环处理 */
void Element_Handle_Right_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 2)
    {
        Ring_Rebuild_Fill(2U);
    }
}

/* 斑马线判定 */
void Element_Judgment_Zebra(void)
{
    int Ysite, Xsite;
    int trans_count;        /* 当前行跳变计数 */
    int valid_rows = 0;

    /* 活动圆环已由Scan_Element提前返回，不会进入本判定。 */
    if (ImageFlag.Zebra_Flag != 0)
        return;

    /* 扫描窗口: 行44~57, 每行统计黑->白(0->1)跳变次数 */
    for (Ysite = 44; Ysite < 58 ; Ysite++)
    {
        trans_count = 0;
        for (Xsite = ZEBRA_SCAN_LEFT; Xsite < ZEBRA_SCAN_RIGHT; Xsite++)
        {
            if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
                trans_count++;
        }

        if (trans_count >= 5) valid_rows++;
    }

    g_ZebraSum = valid_rows;

    /* 单帧满足空间特征即确认；停车是否允许由出环锁存单独判断。 */
    if (valid_rows >= 5)
        ImageFlag.Zebra_Flag = 1;
}

/* 斑马线处理：检测状态+强制直道巡线 */
void Element_Handle_Zebra(void)
{
    int row, Ysite, Xsite;
    int trans_count;
    int exit_rows = 0;
    static int lost_cnt = 0;        /* 斑马线丢失计数器 */

    for (Ysite = 20; Ysite < 33; Ysite++)
    {
        trans_count = 0;
        for (Xsite = ZEBRA_SCAN_LEFT; Xsite < ZEBRA_SCAN_RIGHT; Xsite++)
        {
            if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
                trans_count++;
        }
        if (trans_count >= 5) exit_rows++;
    }

    g_ZebraSum = exit_rows;

    if (exit_rows < 4)
    {
        lost_cnt++;
        if (lost_cnt >= 3)
        {
            ImageFlag.Zebra_Flag = 0;
            lost_cnt = 0;
            return;
        }
    }
    else
    {
        lost_cnt = 0;
    }

    for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
    {
        ImageDeal[row].Center      = ImageSensorMid;
        ImageDeal[row].LeftBorder  = ImageSensorMid - Half_Road_Wide[row];
        ImageDeal[row].RightBorder = ImageSensorMid + Half_Road_Wide[row];
        ImageDeal[row].Wide        = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        LimitL(ImageDeal[row].LeftBorder);
        LimitH(ImageDeal[row].RightBorder);
    }
}

/* 十字弯道扫描参数 */
#define CROSS_SCAN_BOTTOM_ROW       53
#define CROSS_SCAN_TOP_ROW          9
#define CROSS_STABLE_MIN_ROWS        5     //拐点前连续稳定的最少行数
#define CROSS_STABLE_COL_TOLERANCE   1
#define CROSS_JUMP_MIN_COLS          3     //跳变确认列数
#define CROSS_UPPER_CONFIRM_ROWS     1     //向上确认行数
#define CROSS_CORNER_MAX_ROW_DIFF    10    //左右相隔行数
#define CROSS_EXIT_DELAY_FRAMES      5U    //十字最后一次识别后继续屏蔽圆环初判的帧数

#if (CROSS_SCAN_TOP_ROW < 0) || (CROSS_SCAN_BOTTOM_ROW >= LCDH) || (CROSS_SCAN_TOP_ROW >= CROSS_SCAN_BOTTOM_ROW)
#error "CROSS_SCAN_ROW range is invalid"
#endif
#if (CROSS_EXIT_DELAY_FRAMES > 65535U)
#error "CROSS_EXIT_DELAY_FRAMES is too large"
#endif

static uint8 s_cross_detected = 0U;  /* 当前帧左右拐点有效并已完成补线 */
static uint16 s_cross_exit_delay_frames = 0U; /* 十字消失后剩余的圆环屏蔽帧数 */

/* 候选点上方连续两行均无跳变，才确认候选点为十字拐点。 */
static uint8 Cross_Upper_Rows_Have_No_Jump(int row, uint8 check_left)
{
    int offset;
    int previous_border;
    int current_border;
    int delta;

    if (row < CROSS_UPPER_CONFIRM_ROWS)
        return 0U;

    previous_border = check_left ? ImageDeal[row].LeftBorder : ImageDeal[row].RightBorder;
    if (previous_border < 0 || previous_border >= LCDW)
        return 0U;

    for (offset = 1; offset <= CROSS_UPPER_CONFIRM_ROWS; offset++)
    {
        current_border = check_left ? ImageDeal[row - offset].LeftBorder
                                    : ImageDeal[row - offset].RightBorder;
        if (current_border < 0 || current_border >= LCDW)
            return 0U;

        delta = current_border - previous_border;
        if (delta <= -CROSS_JUMP_MIN_COLS || delta >= CROSS_JUMP_MIN_COLS)
            return 0U;

        previous_border = current_border;
    }

    return 1U;
}

/* 从扫描下边界向上同步扫描左右画线，找到稳定直线结束处的十字拐点。 */
static uint8 Cross_Find_Corners(int *left_row, int *left_col,
                                int *right_row, int *right_col)
{
    int row;
    int left_stable_col = 0;
    int right_stable_col = 0;
    int left_stable_rows = 0;
    int right_stable_rows = 0;
    int border;
    int delta;
    uint8 left_found = 0U;
    uint8 right_found = 0U;

    *left_row = -1;
    *left_col = -1;
    *right_row = -1;
    *right_col = -1;

    /* 按屏幕画线坐标扫描，OFFLine和边界找到标志不再过滤。 */
    for (row = CROSS_SCAN_BOTTOM_ROW; row >= CROSS_SCAN_TOP_ROW; row--)
    {
        if (!left_found)
        {
            border = ImageDeal[row].LeftBorder;
            if (border < 0 || border >= LCDW)
            {
                left_stable_rows = 0;
            }
            else
            {
                if (left_stable_rows == 0)
                {
                    left_stable_col = border;
                    left_stable_rows = 1;
                }
                else
                {
                    delta = border - left_stable_col;
                    if (left_stable_rows >= CROSS_STABLE_MIN_ROWS
                        && delta >= CROSS_JUMP_MIN_COLS)
                    {
                        if (Cross_Upper_Rows_Have_No_Jump(row, 1U))
                        {
                            /* 左边界向右突跳且上方两行连续，确认当前点为左十字拐点。 */
                            *left_row = row;
                            *left_col = border;
                            left_found = 1U;
                        }
                        /* 上方仍有跳变时略过候选，保留稳定直线基准继续向上搜索。 */
                    }
                    else if (delta >= -CROSS_STABLE_COL_TOLERANCE
                             && delta <= CROSS_STABLE_COL_TOLERANCE)
                    {
                        left_stable_col = (left_stable_col * left_stable_rows + border)
                                        / (left_stable_rows + 1);
                        left_stable_rows++;
                    }
                    else
                    {
                        left_stable_col = border;
                        left_stable_rows = 1;
                    }
                }
            }
        }

        if (!right_found)
        {
            border = ImageDeal[row].RightBorder;
            if (border < 0 || border >= LCDW)
            {
                right_stable_rows = 0;
            }
            else
            {
                if (right_stable_rows == 0)
                {
                    right_stable_col = border;
                    right_stable_rows = 1;
                }
                else
                {
                    delta = border - right_stable_col;
                    if (right_stable_rows >= CROSS_STABLE_MIN_ROWS
                        && delta <= -CROSS_JUMP_MIN_COLS)
                    {
                        if (Cross_Upper_Rows_Have_No_Jump(row, 0U))
                        {
                            /* 右边界向左突跳且上方两行连续，确认当前点为右十字拐点。 */
                            *right_row = row;
                            *right_col = border;
                            right_found = 1U;
                        }
                        /* 上方仍有跳变时略过候选，保留稳定直线基准继续向上搜索。 */
                    }
                    else if (delta >= -CROSS_STABLE_COL_TOLERANCE
                             && delta <= CROSS_STABLE_COL_TOLERANCE)
                    {
                        right_stable_col = (right_stable_col * right_stable_rows + border)
                                         / (right_stable_rows + 1);
                        right_stable_rows++;
                    }
                    else
                    {
                        right_stable_col = border;
                        right_stable_rows = 1;
                    }
                }
            }
        }

        if (left_found && right_found)
            break;
    }

    return (uint8)(left_found && right_found);
}

/* 十字补线 */
void Get_ExtensionLine(void)
{
    int row;
    int top_row;
    int row_diff;
    int left_row, left_col;
    int right_row, right_col;

    s_cross_detected = 0U;

    /* 圆环已锁定后必须完成出环，禁止交汇处的十字补线覆盖圆环中线。 */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.image_element_rings_flag != RING_STATE_IDLE)
        return;

    if (!Cross_Find_Corners(&left_row, &left_col, &right_row, &right_col))
        return;

    row_diff = left_row - right_row;
    if (row_diff < 0) row_diff = -row_diff;
    if (row_diff > CROSS_CORNER_MAX_ROW_DIFF
        || left_col < 1 || right_col > LCDW - 2
        || left_col >= right_col)
        return;

    /* 复用现有线性补线工具：左下角、右下角分别连接到对应十字拐点。 */
    Ring_DrawAndUpdate(0U, SCAN_BASE_START_ROW, 0,
                       left_row, left_col, 'L');
    Ring_DrawAndUpdate(0U, SCAN_BASE_START_ROW, LCDW - 1,
                       right_row, right_col, 'R');

    top_row = (left_row < right_row) ? left_row : right_row;
    for (row = SCAN_BASE_START_ROW; row >= top_row; row--)
    {
        LimitL(ImageDeal[row].LeftBorder);
        LimitH(ImageDeal[row].RightBorder);
        ImageDeal[row].Wide = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
    }

    s_cross_detected = 1U;
}

/* 元素扫描：按优先级判定斑马线、十字、圆环和长直道。 */
void Scan_Element(void)
{
    s_cross_detected = 0U;

    /* 每帧递减出环冷却，归零后允许识别下一个圆环。 */
    if (s_ring_exit_cooldown > 0U)
        s_ring_exit_cooldown--;

    /* 每帧只统计20~59行断点，降低近端噪声对圆环初判的影响。 */
    g_left_jump_count  = (uint8)Ring_Check_Border_Jump(1U, RING_JUMP_THRESHOLD, RING_JUMP_SCAN_MIN_ROW, RING_JUMP_SCAN_MAX_ROW,
                                                       &s_left_jump_other_lost_count);
    g_right_jump_count = (uint8)Ring_Check_Border_Jump(2U, RING_JUMP_THRESHOLD, RING_JUMP_SCAN_MIN_ROW, RING_JUMP_SCAN_MAX_ROW,
                                                       &s_right_jump_other_lost_count);

    /* 圆环锁定后只推进圆环状态，不再识别斑马线、十字或其他新元素。 */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.image_element_rings_flag != RING_STATE_IDLE)
        return;

    /* 无圆环时按斑马线、十字、圆环初判的顺序识别。 */
    Element_Judgment_Zebra();
    if (ImageFlag.Zebra_Flag != 0)
    {
        return;
    }

    Get_ExtensionLine();
    if (s_cross_detected)
    {
        s_cross_exit_delay_frames = CROSS_EXIT_DELAY_FRAMES;
        return;
    }

    /* 十字拐点离开视野后继续等待N帧，避免车体仍在十字时提前触发圆环减速。 */
    if (s_cross_exit_delay_frames > 0U)
    {
        s_cross_exit_delay_frames--;
        return;
    }

    if (ImageFlag.image_element_rings == 0)
    {
        Element_Judgment_Left_Rings();
        Element_Judgment_Right_Rings();
        /* 弯道沿用基础巡线中线，不再单独识别或覆盖Center。 */
        Straight_long_judge();
    }

    if (ImageFlag.Bend_Road)
    {
    }

    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* 元素处理：按优先级依次调用 */
void Element_Handle(void)
{
    if (ImageFlag.Zebra_Flag != 0)
        Element_Handle_Zebra();
    else if (s_cross_detected)
    {
        /* 十字已在Scan_Element中完成补线。 */
    }
    else if (ImageFlag.image_element_rings == 1)
        Element_Handle_Left_Rings();
    else if (ImageFlag.image_element_rings == 2)
        Element_Handle_Right_Rings();
    else
    {
        if (ImageFlag.straight_long)
            Straight_long_handle();
    }
}

/* 元素标志初始化 */
void Flag_init(void)
{
    ImageFlag.Bend_Road              = 0;
    ImageFlag.Zebra_Flag             = 0;
    ImageFlag.straight_xie           = 0;
    ImageFlag.straight_long          = 0;
}

/* 显示当前元素与圆环状态 */
void Camera_ShowElementStatus(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");
    }
    else if (s_cross_detected)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");
    }

    {
        static const char *rst_name[] = {"IDLE","CNFM","APRC","ENTR","INSD","EX1T","EX2T","RECV"};
        uint8 rst = (uint8)ImageFlag.image_element_rings_flag;
        if (ImageFlag.image_element_rings == 1 && rst < 8)
        {
            ips200_show_string(2, 210, "Ring:L-");
            ips200_show_string(58, 210, rst_name[rst]);
        }
        else if (ImageFlag.image_element_rings == 2 && rst < 8)
        {
            ips200_show_string(2, 210, "Ring:R-");
            ips200_show_string(58, 210, rst_name[rst]);
        }
        else
        {
            ips200_show_string(2, 210, "Ring:---   ");
        }
    }

    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}

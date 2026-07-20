/* 摄像头图像处理模块说明 */
#include "Camera.h"
#include "Shared.h"
static uint16 s_ring_state_frames = 0U;      /* 当前阶段已经持续的图像帧数 */
static uint8 s_ring_confirm_count = 0U;      /* 连续识别确认帧数 */
static uint8 s_ring_feature_count = 0U;      /* 入口或出口特征确认帧数 */
static uint8 s_ring_stable_count = 0U;       /* 出环后稳定直道帧数 */
static uint8 s_ring_exit_loss_seen = 0U;     /* 环内是否见过出口侧丢线 */
static int s_ring_entry_corner_row = -1;     /* 最近一次入口拐点行 */
static int s_ring_entry_corner_col = -1;     /* 最近一次入口拐点列 */
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
int16_t g_ZebraSum = 0;                 /* 斑马线检测差值之和 */
ImageDealDatatypedef ImageDeal[LCDH];        // 更新当前扫描行数据
ImageStatustypedef ImageStatus;              // 更新图像识别状态
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/* 函数说明：Camera_CompressInit。 */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/* 函数说明：Camera_OTSU_GetThreshold，带直方图拉伸的OTSU大津法。 */
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

    /* 第一遍扫描全图找灰度最小最大值。 */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++) {
            uint8 v = *image[i][j];
            if (v < pmin) pmin = v;
            if (v > pmax) pmax = v;
        }

    range = pmax - pmin;

    /* 统计拉伸后直方图：对比度足时拉伸到[0,255]再做OTSU。 */
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

    /* OTSU类间方差遍历找最佳分隔阈值。 */
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

    /* 拉伸后阈值映射回原始灰度范围供二值化使用。 */
    if (range > 30) {
        bestThr = (uint8)(pmin + ((uint16)bestThr * range) / 255U);
    }

    /* 钳位到安全范围。 */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    bestThr += OTSU_BIAS;
    return bestThr;
}

/* 函数说明：Camera_GetBinaryImage。 */
void Camera_GetBinaryImage(void) {
    uint8 thr = Camera_OTSU_GetThreshold(Image_Use, LCDW, LCDH);
    Camera_Threshold = thr;
    uint8 i, j;
    for (i = 0; i < LCDH; i++)
        for (j = 0; j < LCDW; j++)
            Pixle[i][j] = (*Image_Use[i][j] > thr) ? 1 : 0;
}

void Camera_ShowBinaryFast(void) {
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 0, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
}

/* 执行当前图像处理步骤。 */
/* 函数说明：Camera_DrawCenterLines。 */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);  /* 执行当前图像处理步骤。 */

    /* 执行当前图像处理步骤。 */
    /* 执行当前图像处理步骤。 */
    ips200_draw_line(94, 0, 94, 119, RGB565_RED);
    /* 执行当前图像处理步骤。 */
    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    /* 更新图像识别状态。 */
    /* 更新图像识别状态。 */
    /* 两个端点都必须位于OFFLine以上的有效搜线区域。 */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        /* 处理当前扫描行的边线数据。 */
        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        /* 处理当前扫描行的边线数据。 */
        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

void Camera_ShowDebug(void) {
    uint16 xo;
    /* 执行当前图像处理步骤。 */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* 执行当前图像处理步骤。 */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* 处理当前扫描行的边线数据。 */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* 执行当前图像处理步骤。 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    /* legend removed */
    Camera_ShowElementStatus();
    
    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // 记录当前处理步骤
    int   Xsite;                                // 记录当前处理步骤
    int   row;                                  // 记录当前处理步骤

    /* 处理当前扫描行的边线数据。 */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // 更新当前扫描行数据

    // 记录当前处理步骤
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

    // 记录当前处理步骤
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

    // 记录当前处理步骤
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* 处理当前扫描行的边线数据。 */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* 处理当前扫描行的边线数据。 */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // 记录当前处理步骤
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
                ImageDeal[row].IsRightFind = 'F';   // 更新当前扫描行数据
                break;
            }
        }

        // 记录当前处理步骤
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
                ImageDeal[row].IsLeftFind = 'F';    // 更新当前扫描行数据
                break;
            }
        }

        // 记录当前处理步骤
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* 处理当前扫描行的边线数据。 */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* 执行当前图像处理步骤。 */
    // 记录当前处理步骤
}

//-------------------------------------------------------------------------------
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* 执行当前图像处理步骤。 */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // 记录当前处理步骤
    {
        for (i = H; i >= L; i--)
        {
            // 记录当前处理步骤
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // 记录当前处理步骤
                Q->type  = 'T';                 // 记录当前处理步骤
                break;
            }
            else if (i == L)                    // 记录当前处理步骤
            {
                if (*(p + (L + H) / 2) != 0)    // 记录当前处理步骤
                {
                    Q->point = (L + H) / 2;     // 记录当前处理步骤
                    Q->type  = 'W';             // 记录当前处理步骤
                }
                else                            // 记录当前处理步骤
                {
                    Q->point = (L + H) / 2;     // 记录当前处理步骤
                    Q->type  = 'H';             // 记录当前处理步骤
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // 记录当前处理步骤
    {
        for (i = L; i <= H; i++)
        {
            // 记录当前处理步骤
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // 记录当前处理步骤
                Q->type  = 'T';                 // 记录当前处理步骤
                break;
            }
            else if (i == H)                    // 记录当前处理步骤
            {
                if (*(p + (L + H) / 2) != 0)    // 记录当前处理步骤
                {
                    Q->point = (L + H) / 2;     // 记录当前处理步骤
                    Q->type  = 'W';             // 记录当前处理步骤
                }
                else                            // 记录当前处理步骤
                {
                    Q->point = (L + H) / 2;     // 记录当前处理步骤
                    Q->type  = 'H';             // 记录当前处理步骤
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
//  @parameter      void
//  @return         void
// 记录当前处理步骤
// 记录当前处理步骤
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // 记录当前处理步骤
    int   row;                                  // 记录当前处理步骤
    int   IntervalLow, IntervalHigh;            // 记录当前处理步骤
    int   i;                                    // 记录当前处理步骤

    /* 更新图像识别状态。 */
    ImageStatus.OFFLine          = 2;           // 更新图像识别状态
    ImageStatus.Miss_Left_lines  = 0;           // 更新图像识别状态
    ImageStatus.Miss_Right_lines = 0;           // 更新图像识别状态
    ImageStatus.WhiteLine        = 0;           // 更新图像识别状态
    ImageStatus.WhiteLine_L      = 0;           // 更新图像识别状态
    ImageStatus.WhiteLine_R      = 0;           // 更新图像识别状态
    ImageStatus.OFFLineBoundary  = 0;           // 更新图像识别状态
    ImageStatus.Det_True         = 0;           // 更新图像识别状态

    /* 更新图像识别状态。 */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // 记录当前处理步骤
        PicTemp = Pixle[row];

        /* 处理当前扫描行的边线数据。 */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // 记录当前处理步骤
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* 处理当前扫描行的边线数据。 */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* 处理当前扫描行的边线数据。 */
        if (JumpPoint[0].type == 'W')           // 记录当前处理步骤
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // 更新当前扫描行数据
            ImageStatus.Miss_Left_lines++;      // 更新图像识别状态
        }
        else                                    // 记录当前处理步骤
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // 更新图像识别状态
        }

        if (JumpPoint[1].type == 'W')           // 记录当前处理步骤
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // 更新当前扫描行数据
            ImageStatus.Miss_Right_lines++;     // 更新图像识别状态
        }
        else                                    // 记录当前处理步骤
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // 更新图像识别状态
        }

        /* 处理当前扫描行的边线数据。 */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* 更新图像识别状态。 */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // 更新图像识别状态
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* 更新图像识别状态。 */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* 处理当前扫描行的边线数据。 */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /* 处理当前扫描行的边线数据。 */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* 处理当前扫描行的边线数据。 */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)  // 记录当前处理步骤
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* 处理当前扫描行的边线数据。 */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)  // 记录当前处理步骤
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* 处理当前扫描行的边线数据。 */
            /* 处理当前扫描行的边线数据。 */
            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* 更新图像识别状态。 */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))  /* 更新图像识别状态。 */
        {
            ImageStatus.OFFLine = row;          // 更新图像识别状态
            break;
        }

        /*
         * 安财同源保护: 远景宽度过窄或边线贴边时停止继续向上追线。
         * TC264为94列, 由安财80列阈值(7/10/70)按比例映射为8/12/82。
         */
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



/* 执行当前图像处理步骤。 */
const uint8 Half_Road_Wide[60] = {           /* 执行当前图像处理步骤。 */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* 执行当前图像处理步骤。 */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* 执行当前图像处理步骤。 */
ImageFlagtypedef ImageFlag;                  /* 执行当前图像处理步骤。 */

/* 函数说明：Straight_Judge。 */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* 处理当前扫描行的边线数据。 */
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
    case 2: /* 处理当前扫描行的边线数据。 */
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

/* 函数说明：Straight_long_judge。 */
void Straight_long_judge(void)
{
    if (ImageFlag.Bend_Road || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1
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

/* 函数说明：Straight_xie_judge。 */
void Straight_xie_judge(void)
{
    float S, Sum, Err, midd_k;
    int i;

    if (ImageFlag.Zebra_Flag != 0 || ImageFlag.image_element_rings != 0
        || ImageFlag.Ramp == 1)
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

/* 函数说明：Element_Judgment_Bend。 */
void Element_Judgment_Bend(void)
{
    /* 执行当前图像处理步骤。 */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1)
        return;
    /* ponytail直道守卫: OFFLine<5时赛道完全可见，不可能是弯道
       防止噪声导致Miss计数累积引发误判 (安财原始OFFLine>=14, TC264适配60行->5) */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;  /* 更新图像识别状态。 */

    /* 更新图像识别状态。 */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;              /* 执行当前图像处理步骤。 */
    }

    /* 更新图像识别状态。 */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;              /* 执行当前图像处理步骤。 */
    }
}

/* 函数说明：Element_Handle_Bend。 */
void Element_Handle_Bend(void)
{
    int row;                                  /* 用int而非uchar以支持大范围循环 */

    /* ponytail兜底守卫: OFFLine<5时赛道完全可见，清除弯道标志并退出
       与Element_Judgment_Bend的OFFLine守卫呼应，双保险防止直道误判弯道 */
    if (ImageStatus.OFFLine < 5)
        { ImageFlag.Bend_Road = 0; return; }

    /* 双侧都追踪良好 -> 已恢复直道, 清除弯道标志 */
    if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4)
        { ImageFlag.Bend_Road = 0; return; }

    if (ImageFlag.Bend_Road == 1)             /* 左弯: 只靠右边界可见, center=右边界-半宽(向左偏移) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* 闄愬箙 >= 0 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* 右弯: 只靠左边界可见, center=左边界+半宽(向右偏移) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* 闄愬箙 <= 93 */
        }
    }
}

/* 执行当前图像处理步骤。 */
static void Ring_Set_State(uint8 state)
{
    ImageFlag.image_element_rings_flag = state;
    s_ring_state_frames = 0U;
    s_ring_feature_count = 0U;
    s_ring_stable_count = 0U;

    if (state == RING_STATE_CONFIRM)
    {
        s_ring_confirm_count = 0U;
        s_ring_exit_loss_seen = 0U;
        s_ring_entry_corner_row = -1;
        s_ring_entry_corner_col = -1;
    }
    else if (state == RING_STATE_INSIDE)
    {
        s_ring_exit_loss_seen = 0U;
    }
}

static void Ring_Clear_State(void)
{
    ImageFlag.image_element_rings = 0;
    ImageFlag.image_element_rings_flag = RING_STATE_IDLE;
    ImageFlag.ring_big_small = 0;
    s_ring_state_frames = 0U;
    s_ring_confirm_count = 0U;
    s_ring_feature_count = 0U;
    s_ring_stable_count = 0U;
    s_ring_exit_loss_seen = 0U;
    s_ring_entry_corner_row = -1;
    s_ring_entry_corner_col = -1;
}

static uint8 Ring_Is_Candidate(uint8 direction)
{
    if (ImageStatus.OFFLine > 2)
    {
        return 0U;
    }
    if (direction == 1U)
    {
        return (uint8)(ImageStatus.Miss_Left_lines >= 13
                    && ImageStatus.Miss_Right_lines <= 3);
    }
    if (direction == 2U)
    {
        return (uint8)(ImageStatus.Miss_Right_lines >= 15
                    && ImageStatus.Miss_Left_lines <= 3);
    }
    return 0U;
}

/* 从近端向远端寻找本侧边界突变，返回入口拐点所在行。 */
static int Ring_Find_Entry_Corner(uint8 direction, int *corner_col)
{
    int row;

    for (row = SCAN_BASE_START_ROW - 1; row > 25; row--)
    {
        if (direction == 1U
            && ImageDeal[row].IsLeftFind == 'T'
            && ImageDeal[row - 1].IsLeftFind == 'T'
            && abs(ImageDeal[row].LeftBorder - ImageDeal[row - 1].LeftBorder) > 4)
        {
            *corner_col = ImageDeal[row].LeftBorder;
            return row;
        }
        if (direction == 2U
            && ImageDeal[row].IsRightFind == 'T'
            && ImageDeal[row - 1].IsRightFind == 'T'
            && abs(ImageDeal[row].RightBorder - ImageDeal[row - 1].RightBorder) > 4)
        {
            *corner_col = ImageDeal[row].RightBorder;
            return row;
        }
    }

    *corner_col = -1;
    return -1;
}

static uint8 Ring_Is_Stable_Road(void)
{
    return (uint8)(ImageStatus.OFFLine <= 2
                && ImageStatus.Miss_Left_lines < 4
                && ImageStatus.Miss_Right_lines < 4
                && Straight_Judge(1, 5, SCAN_BASE_END_ROW) < 2.0f
                && Straight_Judge(2, 5, SCAN_BASE_END_ROW) < 2.0f);
}

/* 出口侧必须先丢失再恢复，避免环内短暂双边可见时提前出环。 */
static uint8 Ring_Has_Exit_Feature(uint8 direction)
{
    int row;

    if ((direction == 1U && ImageStatus.Miss_Right_lines > 4)
        || (direction == 2U && ImageStatus.Miss_Left_lines > 4))
    {
        return 0U;
    }

    for (row = SCAN_BASE_START_ROW - 1; row > 5; row--)
    {
        if (direction == 1U
            && ImageDeal[row].IsRightFind == 'T'
            && ImageDeal[row - 1].IsRightFind != 'T'
            && ImageDeal[row - 2].IsRightFind != 'T')
        {
            return 1U;
        }
        if (direction == 2U
            && ImageDeal[row].IsLeftFind == 'T'
            && ImageDeal[row - 1].IsLeftFind != 'T'
            && ImageDeal[row - 2].IsLeftFind != 'T')
        {
            return 1U;
        }
    }

    return Ring_Is_Stable_Road();
}

static int Ring_Get_Center_Offset(uint8 state)
{
    switch (state)
    {
    case RING_STATE_APPROACH: return RING_APPROACH_CENTER_OFFSET;
    case RING_STATE_ENTRY:    return RING_ENTRY_CENTER_OFFSET;
    case RING_STATE_INSIDE:   return RING_INSIDE_CENTER_OFFSET;
    case RING_STATE_EXIT:     return RING_EXIT_CENTER_OFFSET;
    case RING_STATE_RECOVERY: return RING_RECOVERY_CENTER_OFFSET;
    default:                  return 0;
    }
}

/* 左右圆环共用镜像补线，固定偏移确保舵机误差越过现有死区。 */
static void Ring_Rebuild_Center(uint8 direction)
{
    int row;
    int center_offset = Ring_Get_Center_Offset((uint8)ImageFlag.image_element_rings_flag);

    for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
    {
        if (direction == 1U)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder
                                  - Half_Bend_Wide[row] - center_offset;
            LimitL(ImageDeal[row].Center);
        }
        else
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                  + Half_Bend_Wide[row] + center_offset;
            LimitH(ImageDeal[row].Center);
        }
    }
}

static void Ring_State_Update(void)
{
    uint8 direction = (uint8)ImageFlag.image_element_rings;
    int corner_col = -1;
    int corner_row;

    if (direction != 1U && direction != 2U)
    {
        Ring_Clear_State();
        return;
    }
    if (s_ring_state_frames < 65535U)
    {
        s_ring_state_frames++;
    }

    switch (ImageFlag.image_element_rings_flag)
    {
    case RING_STATE_CONFIRM:
        if (Ring_Is_Candidate(direction))
        {
            if (s_ring_confirm_count < RING_CONFIRM_FRAMES)
            {
                s_ring_confirm_count++;
            }
        }
        else
        {
            s_ring_confirm_count = 0U;
        }
        if (s_ring_confirm_count >= RING_CONFIRM_FRAMES)
        {
            Ring_Set_State(RING_STATE_APPROACH);
        }
        else if (s_ring_state_frames >= RING_CONFIRM_MAX_FRAMES)
        {
            Ring_Clear_State();
        }
        break;

    case RING_STATE_APPROACH:
        corner_row = Ring_Find_Entry_Corner(direction, &corner_col);
        if (corner_row >= 0)
        {
            s_ring_entry_corner_row = corner_row;
            s_ring_entry_corner_col = corner_col;
        }
        if (corner_row >= RING_ENTRY_CORNER_ROW)
        {
            if (s_ring_feature_count < 2U) s_ring_feature_count++;
        }
        else
        {
            s_ring_feature_count = 0U;
        }
        if (s_ring_feature_count >= 2U
            || s_ring_state_frames >= RING_APPROACH_MAX_FRAMES)
        {
            Ring_Set_State(RING_STATE_ENTRY);
        }
        break;

    case RING_STATE_ENTRY:
        corner_row = Ring_Find_Entry_Corner(direction, &corner_col);
        if (corner_row >= 0)
        {
            s_ring_entry_corner_row = corner_row;
            s_ring_entry_corner_col = corner_col;
        }
        if (corner_row >= RING_INSIDE_CORNER_ROW
            || (corner_row < 0 && s_ring_state_frames >= 6U))
        {
            if (s_ring_feature_count < 3U) s_ring_feature_count++;
        }
        else
        {
            s_ring_feature_count = 0U;
        }
        if (s_ring_feature_count >= 3U
            || s_ring_state_frames >= RING_ENTRY_MAX_FRAMES)
        {
            Ring_Set_State(RING_STATE_INSIDE);
        }
        break;

    case RING_STATE_INSIDE:
        if ((direction == 1U && ImageStatus.Miss_Right_lines >= RING_EXIT_MISS_MIN)
            || (direction == 2U && ImageStatus.Miss_Left_lines >= RING_EXIT_MISS_MIN)
            || ImageStatus.OFFLine >= RING_EXIT_MISS_MIN)
        {
            s_ring_exit_loss_seen = 1U;
        }
        if (s_ring_exit_loss_seen && Ring_Has_Exit_Feature(direction))
        {
            if (s_ring_feature_count < RING_EXIT_CONFIRM_FRAMES)
            {
                s_ring_feature_count++;
            }
        }
        else
        {
            s_ring_feature_count = 0U;
        }
        if (s_ring_feature_count >= RING_EXIT_CONFIRM_FRAMES
            || s_ring_state_frames >= RING_INSIDE_MAX_FRAMES)
        {
            Ring_Set_State(RING_STATE_EXIT);
        }
        break;

    case RING_STATE_EXIT:
        if (Ring_Is_Stable_Road())
        {
            if (s_ring_stable_count < RING_EXIT_STABLE_FRAMES)
            {
                s_ring_stable_count++;
            }
        }
        else
        {
            s_ring_stable_count = 0U;
        }
        if (s_ring_stable_count >= RING_EXIT_STABLE_FRAMES
            || s_ring_state_frames >= RING_EXIT_MAX_FRAMES)
        {
            Ring_Set_State(RING_STATE_RECOVERY);
        }
        break;

    case RING_STATE_RECOVERY:
        if (Ring_Is_Stable_Road())
        {
            if (s_ring_stable_count < 4U) s_ring_stable_count++;
        }
        else
        {
            s_ring_stable_count = 0U;
        }
        if ((s_ring_state_frames >= RING_RECOVERY_FRAMES
             && s_ring_stable_count >= 4U)
            || s_ring_state_frames >= RING_RECOVERY_MAX_FRAMES)
        {
            Ring_Clear_State();
        }
        break;

    default:
        Ring_Clear_State();
        break;
    }
}

void Element_Judgment_Left_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Left_Less_Num = 0;

    /* 安财同源门槛：左圆环必须先出现左侧连续丢线，直道噪声不得触发。 */
    if (ImageStatus.Miss_Right_lines > 3
        || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 2 || Straight_Judge(2, 5, SCAN_BASE_END_ROW) > 3.0f   /* 执行当前图像处理步骤。 */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* 条件不足时禁止进入圆环补线，防止覆盖直道中心。 */

    /* 执行当前图像处理步骤。 */
    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* 处理当前扫描行的边线数据。 */
        {
            if (ImageDeal[r].IsLeftFind == 'W') return;
        }
    }

    /* 处理当前扫描行的边线数据。 */
    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 1].LeftBorder) > 4  /* 执行当前图像处理步骤。 */)
        {
            Left_Less_Num++;
            /* 执行当前图像处理步骤。 */
            if (Left_Less_Num == 1) {
                /* 执行当前图像处理步骤。 */
            }
        }
    }

    if (Left_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 1;    /* 执行当前图像处理步骤。 */
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* 函数说明：Element_Judgment_Right_Rings。 */
void Element_Judgment_Right_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Right_Less_Num = 0;

    /* 安财同源门槛：右圆环必须先出现右侧连续丢线，直道噪声不得触发。 */
    if (ImageStatus.Miss_Left_lines > 3
        || ImageStatus.Miss_Right_lines < 15
        || ImageStatus.OFFLine > 2 || Straight_Judge(1, 5, SCAN_BASE_END_ROW) > 3.0f   /* 执行当前图像处理步骤。 */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* 条件不足时禁止进入圆环补线，防止覆盖直道中心。 */

    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* 处理当前扫描行的边线数据。 */
        {
            if (ImageDeal[r].IsRightFind == 'W') return;
        }
    }

    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].RightBorder - ImageDeal[Ysite - 1].RightBorder) > 4  /* 执行当前图像处理步骤。 */)
        {
            Right_Less_Num++;
        }
    }

    if (Right_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 2;    /* 执行当前图像处理步骤。 */
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* 函数说明：Element_Handle_Left_Rings。 */
void Element_Handle_Left_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 1)
    {
        Ring_Rebuild_Center(1U);
    }
}

/* 函数说明：Element_Handle_Right_Rings。 */
void Element_Handle_Right_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 2)
    {
        Ring_Rebuild_Center(2U);
    }
}

/* 函数说明：Element_Judgment_Zebra。 */
/* 函数说明：Element_Judgment_Zebra，基于边线宽度差值之和判断斑马线。 */
/* 函数说明：Element_Judgment_Zebra，多重防误判的斑马线检测。 */
/* 函数说明：Element_Judgment_Zebra，近处宽+远处窄=斑马线。 */
/* 函数说明：Element_Judgment_Zebra，车身近处宽+远处窄=斑马线。 */
/* 函数说明：Element_Judgment_Zebra，近处宽+远处异常(窄或丢线)+非弯道=斑马线。 */
/* 函数说明：Element_Judgment_Zebra，基于原始灰度相邻像素差分检测斑马线。
   斑马线特征：赛道区内相邻像素灰度剧烈交替（黑白条纹），差分绝对值>50。
   完全不依赖二值化阈值，直接使用Image_Use原始灰度数组。 */
/* 函数说明：Element_Judgment_Zebra，二值图黑白跳变计数检测斑马线。 */
/* 函数说明：Element_Judgment_Zebra，二值图双向跳变+局部低阈值兜底。 */
/* 函数说明：Element_Judgment_Zebra，边线丢失时用固定宽度兜底扫描。 */
void Element_Judgment_Zebra(void)
{
    int Ysite, Xsite, prev, curr, trans, NUM = 0;
    int L, R, scanned = 0;

    if (ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    for (Ysite = 20; Ysite < 33; Ysite++)
    {
        L = ImageDeal[Ysite].LeftBorder;
        R = ImageDeal[Ysite].RightBorder;

        /* 边线有效时用边线范围，无效时用固定宽范围兜底(列5~89) */
        if (L >= 0 && R >= 0 && R - L >= 6)
        {
            L += 2;
            R -= 3;
        }
        else
        {
            L = 5;
            R = LCDW - 5;
        }

        trans = 0;
        /* 先用二值图Pixle检测双向跳变 */
        for (Xsite = L; Xsite < R; Xsite++)
        {
            if (Pixle[Ysite][Xsite] != Pixle[Ysite][Xsite + 1])
                trans++;
        }
        /* 如果二值图跳变不够，再用灰度图局部低阈值(120)补检 */
        if (trans < 3)
        {
            trans = 0;
            prev = (*Image_Use[Ysite][L] > 120) ? 1 : 0;
            for (Xsite = L + 1; Xsite <= R; Xsite++)
            {
                curr = (*Image_Use[Ysite][Xsite] > 120) ? 1 : 0;
                if (prev != curr) trans++;
                prev = curr;
            }
        }
        if (trans >= 3) NUM++;
        scanned++;
    }

    g_ZebraSum = (scanned > 0) ? NUM : -1;

    if (NUM > 6)
    {
        ImageFlag.Zebra_Flag = 1;
    }
}

/* 函数说明：Element_Handle_Zebra。 */
void Element_Handle_Zebra(void)
{
    int row;

    if (ImageFlag.Zebra_Flag == 1)            /* 更新图像识别状态。 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Road_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }
    else if (ImageFlag.Zebra_Flag == 2)       /* 更新图像识别状态。 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Road_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }
}

/* 函数说明：Element_Judgment_Ramp。 */
void Element_Judgment_Ramp(void)
{
        return;                              /* 执行当前图像处理步骤。 */
    int Ysite;
    int i = 0;                           /* 更新图像识别状态。 */

    if (ImageStatus.WhiteLine >= 3) return;

    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if (ImageDeal[Ysite].Wide > 18
             && ImageDeal[Ysite].IsRightFind == 'T'
             && ImageDeal[Ysite].IsLeftFind == 'T'
             && ImageDeal[Ysite].LeftBorder < 40
             && ImageDeal[Ysite].RightBorder > 55   /* TC264: >55(原>40) */
             && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center - 2] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center + 2] == 1
             && ImageStatus.Miss_Left_lines < 7
             && ImageStatus.Miss_Right_lines < 7)
            {
                i++;
            }
        }

        if (i >= 3)                           /* 执行当前图像处理步骤。 */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* 函数说明：Element_Handle_Ramp。 */
void Element_Handle_Ramp(void)
{
    /* 执行当前图像处理步骤。 */

}

/* 函数说明：Element_Judgment_OutRoad。 */
void Element_Judgment_OutRoad(void)
{
    int Right_Num = 0, Left_Num = 0;
    int Ysite;

    if (ImageFlag.Out_Road) return;

    if (ImageStatus.OFFLine > 20)
    {
        for (Ysite = ImageStatus.OFFLine + 1;
             Ysite < ImageStatus.OFFLine + 11; Ysite++)
        {
            if (ImageDeal[Ysite].IsLeftFind == 'T')  Left_Num++;
            if (ImageDeal[Ysite].IsRightFind == 'T') Right_Num++;
        }
    }

    if (Left_Num > 7 && Right_Num > 7)
    {
        ImageFlag.Out_Road = 1;
    }
}

/* 函数说明：Element_Handle_OutRoad。 */
void Element_Handle_OutRoad(void)
{
    int Ysite, Xsite;
    int gray_sum = 0;

    /* 执行当前图像处理步骤。 */
    for (Ysite = 35; Ysite < 55; Ysite++)
    {
        for (Xsite = 30; Xsite < 64; Xsite++) /* 处理当前扫描行的边线数据。 */
        {
            gray_sum += Pixle[Ysite][Xsite];
        }
    }

    /* 更新图像识别状态。 */
    if (gray_sum > 400 && ImageStatus.OFFLine < 20)
    {
        ImageFlag.Out_Road = 0;
    }
}

/* 执行当前图像处理步骤。 */
#define CROSS_WHITE_LINE_MIN 8
#define CROSS_VALID_LINE_COUNT 3

/* 分别修复十字区域的一侧边线，避免单侧识别异常影响另一侧。 */
static void Repair_Cross_Border(uint8 is_left)
{
    int row;
    int near_row = -1;
    int far_row = -1;
    int near_border;
    int far_border = 0;
    int border;

    /* 第一段白行的下一近场行作为补线起点。 */
    for (row = SCAN_BASE_END_ROW - 1;
         row >= ImageStatus.OFFLine + CROSS_VALID_LINE_COUNT - 1;
         row--)
    {
        if ((is_left && ImageDeal[row].IsLeftFind == 'W')
         || (!is_left && ImageDeal[row].IsRightFind == 'W'))
        {
            near_row = row + 1;
            break;
        }
    }
    if (near_row < 0 || near_row >= LCDH) return;

    near_border = is_left ? ImageDeal[near_row].LeftBorder
                          : ImageDeal[near_row].RightBorder;
    if (near_border < 1 || near_border > LCDW - 2) return;

    /* 十字远端必须连续三行边线有效，防止噪点被当作补线锚点。 */
    for (row = near_row - 2;
         row >= ImageStatus.OFFLine + CROSS_VALID_LINE_COUNT - 1;
         row--)
    {
        if (is_left
         && ImageDeal[row].IsLeftFind == 'T'
         && ImageDeal[row - 1].IsLeftFind == 'T'
         && ImageDeal[row - 2].IsLeftFind == 'T')
        {
            far_row = row - 2;
            break;
        }
        if (!is_left
         && ImageDeal[row].IsRightFind == 'T'
         && ImageDeal[row - 1].IsRightFind == 'T'
         && ImageDeal[row - 2].IsRightFind == 'T')
        {
            far_row = row - 2;
            break;
        }
    }

    if (far_row >= 0)
    {
        far_border = is_left ? ImageDeal[far_row].LeftBorder
                             : ImageDeal[far_row].RightBorder;
        if (far_border < 1 || far_border > LCDW - 2) far_row = -1;
    }

    if (far_row < 0)
    {
        /* 未找到可靠远锚点时保持入口边界，车辆继续按入口方向直行。 */
        for (row = near_row - 1; row > ImageStatus.OFFLine; row--)
        {
            if (is_left && ImageDeal[row].IsLeftFind == 'W')
                ImageDeal[row].LeftBorder = near_border;
            else if (!is_left && ImageDeal[row].IsRightFind == 'W')
                ImageDeal[row].RightBorder = near_border;
        }
        return;
    }

    for (row = near_row - 1; row >= far_row; row--)
    {
        border = near_border
                   + (far_border - near_border) * (near_row - row)
                   / (near_row - far_row);
        if (is_left)
            ImageDeal[row].LeftBorder = border;
        else
            ImageDeal[row].RightBorder = border;
    }
}

void Get_ExtensionLine(void)
{
    int row;

    if (ImageStatus.WhiteLine < CROSS_WHITE_LINE_MIN) return;

    Repair_Cross_Border(1);
    Repair_Cross_Border(0);

    /* 补线后统一限幅并重建中线，供 CPU0 计算 Err。 */
    for (row = SCAN_BASE_END_ROW; row > ImageStatus.OFFLine; row--)
    {
        LimitL(ImageDeal[row].LeftBorder);
        LimitH(ImageDeal[row].LeftBorder);
        LimitL(ImageDeal[row].RightBorder);
        LimitH(ImageDeal[row].RightBorder);

        if (ImageDeal[row].LeftBorder >= ImageDeal[row].RightBorder)
        {
            if (row < SCAN_BASE_END_ROW)
            {
                ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;
                ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder;
            }
            else
            {
                ImageDeal[row].LeftBorder = ImageSensorMid - Half_Road_Wide[row];
                ImageDeal[row].RightBorder = ImageSensorMid + Half_Road_Wide[row];
            }
        }

        ImageDeal[row].Wide = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
    }
}
/* 函数说明：Scan_Element。 */
void Scan_Element(void)
{
    /* 执行当前图像处理步骤。 */
    if (ImageFlag.Out_Road == 0 && ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0)  /* 更新圆环识别状态机。 */
    {
        Element_Judgment_OutRoad();           /* 更新圆环识别状态机。 */
        Element_Judgment_Left_Rings();        /* 更新圆环识别状态机。 */
        Element_Judgment_Right_Rings();       /* 执行当前图像处理步骤。 */
        Element_Judgment_Zebra();             /* 执行当前图像处理步骤。 */
        Element_Judgment_Bend();              /* 执行当前图像处理步骤。 */
        Element_Judgment_Ramp();              /* 执行当前图像处理步骤。 */
        Straight_long_judge();                /* 执行当前图像处理步骤。 */
    }

    /* 执行当前图像处理步骤。 */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();
        if (ImageFlag.Out_Road) ImageFlag.Bend_Road = 0;
    }

    /* 执行当前图像处理步骤。 */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* 函数说明：Element_Handle。 */
void Element_Handle(void)
{
    if (ImageFlag.Out_Road != 0)
        Element_Handle_OutRoad();
    else if (ImageFlag.image_element_rings == 1)
        Element_Handle_Left_Rings();
    else if (ImageFlag.image_element_rings == 2)
        Element_Handle_Right_Rings();
    else if (ImageFlag.Zebra_Flag != 0)
        Element_Handle_Zebra();
    else if (ImageFlag.Ramp != 0)
        Element_Handle_Ramp();
    else if (ImageStatus.WhiteLine >= CROSS_WHITE_LINE_MIN)
        Get_ExtensionLine();                  /* 十字优先于普通直道和弯道补线。 */
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
}
/* 函数说明：Flag_init。 */
void Flag_init(void)
{
    ImageFlag.Bend_Road              = 0;
    ImageFlag.Zebra_Flag             = 0;
    ImageFlag.Ramp                   = 0;
    ImageFlag.straight_xie           = 0;
    ImageFlag.straight_long          = 0;
    ImageFlag.Out_Road               = 0;
}


//-------------------------------------------------------------------------------
// 记录当前处理步骤
// 记录当前处理步骤
// 记录当前处理步骤
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* 执行当前图像处理步骤。 */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /* 执行当前图像处理步骤。 */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* 执行当前图像处理步骤。 */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* 更新图像识别状态。 */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* 执行当前图像处理步骤。 */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* 执行当前图像处理步骤。 */
    }

    /* 显示圆环阶段标志位：方向+阶段缩写，便于调试状态机切换 */
    {
        static const char *rst_name[] = {"IDLE","CNFM","APRC","ENTR","INSD","EXIT","RECV"};
        uint8 rst = (uint8)ImageFlag.image_element_rings_flag;
        if (ImageFlag.image_element_rings == 1 && rst < 7)
        {
            ips200_show_string(2, 210, "Ring:L-");
            ips200_show_string(58, 210, rst_name[rst]);
        }
        else if (ImageFlag.image_element_rings == 2 && rst < 7)
        {
            ips200_show_string(2, 210, "Ring:R-");
            ips200_show_string(58, 210, rst_name[rst]);
        }
        else
        {
            ips200_show_string(2, 210, "Ring:---   ");
        }
    }

    /* 执行当前图像处理步骤。 */
    /* 底栏右侧显示当前图像偏差，与元素状态同帧刷新。 */
    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


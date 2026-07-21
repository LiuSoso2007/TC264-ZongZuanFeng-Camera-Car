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


/*
 *******************************************************************************************
 ** 黑洞检测法 —— 圆环识别核心函数组
 ** 基于 hao-yue-1/SmartCar (广东工业大学霹雳火队) 纯视觉方案
 ** 适配 TC264 + MT9V03X + 94x60压缩图
 ** 无IMU、无电磁 —— 纯二值图像素分析
 *******************************************************************************************
 */

/* ---- 黑洞底部检测：检查图像底部角落是否存在黑色区域 ---- */
static uint8 BlackHole_Check_Bottom(uint8 direction)
{
    int row, col;
    int state;      /* 0=??, 1=??, 2=?? */
    int black_cnt;  /* ?????? */
    int start_col, end_col, step;

    /* ????3??LCDH-1(59), LCDH-2(58), LCDH-3(57) */
    for (row = LCDH - 1; row >= LCDH - 3; row--)
    {
        state = 0;
        black_cnt = 0;

        /* ????????????????? */
        if (direction == 1U) { start_col = 0; end_col = LCDW - 1; step = 1; }
        else                 { start_col = LCDW - 1; end_col = 0; step = -1; }

        for (col = start_col; col != end_col; col += step)
        {
            if (Pixle[row][col] == IMG_WHITE)
            {
                if (state == 2 && black_cnt >= 5)
                    return 1;   /* ???(>=5?)???????? */
                state = 1;
                black_cnt = 0;
            }
            else /* IMG_BLACK */
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


/* ---- 验证拐点上方是否存在黑洞 —— 区分普通弯道与圆环 ---- */
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

/* ---- 谷底追踪：从白黑跳变点向右下/左下追踪到谷底 ---- */
static int BlackHole_Track_Valley(uint8 direction, int *valley_row, int *valley_col)
{
    int row, col;
    int moved;
    int scan_col;
    
    scan_col = (direction == 1U) ? VALLEY_SCAN_COL_LEFT : VALLEY_SCAN_COL_RIGHT;
    
    for (row = VALLEY_SCAN_START_ROW; row > VALLEY_MIN_ROW; row--)
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
            
            if (row > VALLEY_MIN_ROW && row < VALLEY_MAX_ROW
                && col > 0 && col < LCDW - 1)
            {
                *valley_row = row;
                *valley_col = col;
                return 1;
            }
            return 0;
        }
    }
    return 0;
}

/* ---- 稳定直道检测：双边不丢线 + 斜率正常 ---- */
static uint8 Ring_Is_Stable_Road(void)
{
    return (uint8)(ImageStatus.OFFLine <= 2
                && ImageStatus.Miss_Left_lines < 4
                && ImageStatus.Miss_Right_lines < 4
                && Straight_Judge(1, 5, SCAN_BASE_END_ROW) < 2.0f
                && Straight_Judge(2, 5, SCAN_BASE_END_ROW) < 2.0f);
}

/* ---- 候选检测：黑洞底部 + 丢线特征 ---- */
static uint8 Ring_Is_Candidate(uint8 direction)
{
    if (ImageStatus.OFFLine > 2)
        return 0U;
    if (!BlackHole_Check_Bottom(direction))
        return 0U;
    if (direction == 1U)
        return (uint8)(ImageStatus.Miss_Left_lines >= 10);
    if (direction == 2U)
        return (uint8)(ImageStatus.Miss_Right_lines >= 10);
    return 0U;
}

/* ---- 寻找入口谷底点：用谷底追踪替代边线跳变检测 ---- */
static int Ring_Find_Valley_Point(uint8 direction, int *valley_col)
{
    int valley_row = -1;
    int vcol = -1;
    
    if (BlackHole_Track_Valley(direction, &valley_row, &vcol) == 0)
    {
        *valley_col = -1;
        return -1;
    }
    *valley_col = vcol;
    return valley_row;
}

/* ---- 出环特征检测：出口侧边线恢复 + 黑洞验证 ---- */
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

/* ---- ????????????? + ???????ImageDeal ---- */
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
            Pixle[row][col] = IMG_WHITE;     /* ??????????????? */

        /* ???????ImageDeal??Err?????? */
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

/* ---- 补线偏移量 ---- */
static int Ring_Get_Fill_Offset(uint8 ring_state)
{
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
    case RING_STATE_APPROACH: return FILL_ENTRY_OFFSET;
    case RING_STATE_ENTRY:    return FILL_ENTRY_OFFSET;
    case RING_STATE_INSIDE:   return FILL_INSIDE_OFFSET;
    case RING_STATE_EXIT:     return FILL_EXIT_OFFSET;
    case RING_STATE_RECOVERY: return FILL_RECOVERY_OFFSET;
    default:                  return 0;
    }
}

static void Ring_Rebuild_Fill(uint8 direction)
{
    int row;
    int valley_row, valley_col;
    uint8 ring_state = (uint8)ImageFlag.image_element_rings_flag;
    int fill_offset = Ring_Get_Fill_Offset(ring_state);
    int has_valley = BlackHole_Track_Valley(direction, &valley_row, &valley_col);
    
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
        /* ??????????????????? */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = ImageDeal[row].RightBorder
                                      - Half_Bend_Wide[row] - fill_offset;
            else
                ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                      + Half_Bend_Wide[row] + fill_offset;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_APPROACH:
        /* ????????????????????(??)?????? */
        if (has_valley)
        {
            if (direction == 1U) /* ?????? ? ??????? */
                Ring_DrawAndUpdate(direction, LCDH - 1, 2,
                                   valley_row, valley_col, 'L');
            else                 /* ?????? ? ??????? */
                Ring_DrawAndUpdate(direction, LCDH - 1, LCDW - 3,
                                   valley_row, valley_col, 'R');
        }
        else
        {
            /* ??????????? */
            for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        break;
    case RING_STATE_ENTRY:
        /* ???????????????????????? */
        if (has_valley)
        {
            if (direction == 1U) /* ?????? ? ???????? */
                Ring_DrawAndUpdate(direction, LCDH - 1, LCDW - 3,
                                   valley_row, valley_col, 'R');
            else                 /* ?????? ? ???????? */
                Ring_DrawAndUpdate(direction, LCDH - 1, 2,
                                   valley_row, valley_col, 'L');
        }
        else
        {
            /* ??????????? */
            for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
            {
                if (direction == 1U)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + fill_offset;
                LimitL(ImageDeal[row].Center);
                LimitH(ImageDeal[row].Center);
            }
        }
        break;
    case RING_STATE_INSIDE:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = ImageDeal[row].RightBorder
                                      - Half_Bend_Wide[row] - fill_offset;
            else
                ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                      + Half_Bend_Wide[row] + fill_offset;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_EXIT:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - FILL_INSIDE_OFFSET;
            }
            else
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + FILL_INSIDE_OFFSET;
            }
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_RECOVERY:
    default:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = (ImageDeal[row].RightBorder > 0)
                    ? ImageDeal[row].RightBorder - Half_Road_Wide[row] - fill_offset
                    : ImageSensorMid;
            else
                ImageDeal[row].Center = (ImageDeal[row].LeftBorder < LCDW - 1)
                    ? ImageDeal[row].LeftBorder + Half_Road_Wide[row] + fill_offset
                    : ImageSensorMid;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    }
}

/* ---- 状态机更新 ---- */
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
        if (Ring_Is_Candidate(direction))
        { if (s_ring_confirm_count < RING_CONFIRM_FRAMES) s_ring_confirm_count++; }
        else
        { s_ring_confirm_count = 0U; }
        if (s_ring_confirm_count >= RING_CONFIRM_FRAMES)
            Ring_Set_State(RING_STATE_APPROACH);
        else if (s_ring_state_frames >= RING_CONFIRM_MAX_FRAMES)
            Ring_Set_State(RING_STATE_APPROACH);
        break;

    case RING_STATE_APPROACH:
        valley_row = Ring_Find_Valley_Point(direction, &valley_col);
        if (valley_row >= 0)
        { s_ring_entry_corner_row = valley_row; s_ring_entry_corner_col = valley_col; }
        if (valley_row >= 0 && valley_row < VALLEY_MAX_ROW)
        { if (s_ring_feature_count < 3U) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= 3U || s_ring_state_frames >= RING_APPROACH_MAX_FRAMES)
            Ring_Set_State(RING_STATE_ENTRY);
        break;

    case RING_STATE_ENTRY:
        valley_row = Ring_Find_Valley_Point(direction, &valley_col);
        if (valley_row >= 0)
        { s_ring_entry_corner_row = valley_row; s_ring_entry_corner_col = valley_col; }
        if (valley_row < 0 || valley_row < VALLEY_MIN_ROW + 5)
        { if (s_ring_feature_count < 3U) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= 3U || s_ring_state_frames >= RING_ENTRY_MAX_FRAMES)
            Ring_Set_State(RING_STATE_INSIDE);
        break;

    case RING_STATE_INSIDE:
        if ((direction == 1U && ImageStatus.Miss_Right_lines >= EXIT_LOST_MIN)
            || (direction == 2U && ImageStatus.Miss_Left_lines >= EXIT_LOST_MIN)
            || ImageStatus.OFFLine >= EXIT_LOST_MIN)
            s_ring_exit_loss_seen = 1U;
        if (s_ring_exit_loss_seen && Ring_Has_Exit_Feature(direction))
        { if (s_ring_feature_count < RING_EXIT_CONFIRM_FRAMES) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= RING_EXIT_CONFIRM_FRAMES
            || s_ring_state_frames >= RING_INSIDE_MAX_FRAMES)
            Ring_Set_State(RING_STATE_EXIT);
        break;

    case RING_STATE_EXIT:
        if (Ring_Is_Stable_Road())
        { if (s_ring_stable_count < RING_EXIT_STABLE_FRAMES) s_ring_stable_count++; }
        else
        { s_ring_stable_count = 0U; }
        if (s_ring_stable_count >= RING_EXIT_STABLE_FRAMES
            || s_ring_state_frames >= RING_EXIT_MAX_FRAMES)
            Ring_Set_State(RING_STATE_RECOVERY);
        break;

    case RING_STATE_RECOVERY:
        if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4
            && ImageStatus.OFFLine <= 2)
        { if (s_ring_stable_count < 4U) s_ring_stable_count++; }
        else
        { s_ring_stable_count = 0U; }
        if ((s_ring_state_frames >= RING_RECOVERY_FRAMES && s_ring_stable_count >= 4U)
            || s_ring_state_frames >= RING_RECOVERY_MAX_FRAMES)
            Ring_Clear_State();
        break;

    default:
        Ring_Clear_State();
        break;
    }
}

/* ---- 左圆环判断：黑洞检测触发 ---- */
void Element_Judgment_Left_Rings(void)
{
    if (ImageStatus.Miss_Right_lines > 5
        || ImageStatus.Miss_Left_lines < 6
        || ImageStatus.OFFLine > 2
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    { int r; for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)
    { if (ImageDeal[r].IsLeftFind == 'W') return; } }

    if (BlackHole_Check_Bottom(1U))
    { ImageFlag.image_element_rings = 1; Ring_Set_State(RING_STATE_CONFIRM); }
}

/* ---- 右圆环判断：黑洞检测触发 ---- */
void Element_Judgment_Right_Rings(void)
{
    if (ImageStatus.Miss_Left_lines > 5
        || ImageStatus.Miss_Right_lines < 6
        || ImageStatus.OFFLine > 2
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    { int r; for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)
    { if (ImageDeal[r].IsRightFind == 'W') return; } }

    if (BlackHole_Check_Bottom(2U))
    { ImageFlag.image_element_rings = 2; Ring_Set_State(RING_STATE_CONFIRM); }
}
/* Element_Handle_Left_Rings */
void Element_Handle_Left_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 1)
    {
        Ring_Rebuild_Fill(1U);
    }
}

/* Element_Handle_Right_Rings */
void Element_Handle_Right_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 2)
    {
        Ring_Rebuild_Fill(2U);
    }
}


void Element_Judgment_Zebra(void)
{
    int Ysite, Xsite;
    int trans_count;        /* 当前行黑->白跳变次数 */
    int valid_rows = 0;     /* 有效行数(跳变>=5的行) */
    static int confirm_cnt = 0;     /* 连续确认帧计数(用于防抖) */

    /* 环岛/出库等元素激活或已处于斑马线状态时不检测 */
    if (ImageFlag.image_element_rings || ImageFlag.Out_Road == 1
     || ImageFlag.Zebra_Flag != 0)
        return;

    /* 固定中央窗口扫描行44~57，仅统计黑->白跳变(0->1)。
     * 斑马线在直道正中，路面宽度30~52px落在60px窗口内。
     * 不依赖边界检测，天然免疫十字路口和弯道背景噪声。 */
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

    /* 有效行>=5时疑似斑马线，需连续2帧确认防误判 */
    if (valid_rows >= 5)
    {
        confirm_cnt++;
        if (confirm_cnt >= 2)
        {
            ImageFlag.Zebra_Flag = 1;       /* 确认斑马线 */
        }
    }
    else
    {
        confirm_cnt = 0;                     /* 未达阈值，清零确认计数 */
    }
}











/* 函数说明：Element_Handle_Zebra */
/* 函数说明：Element_Handle_Zebra：斑马线处理——直道斑马线，锁定中线为图像中点直行，并检测退出条件。 */
/* 函数说明：Element_Handle_Zebra */
/* 函数说明：Element_Handle_Zebra：斑马线处理——直道斑马线，锁定中线为图像中点直行，并检测退出条件。 */
void Element_Handle_Zebra(void)
{
    int row, Ysite, Xsite;
    int trans_count;
    int exit_rows = 0;
    static int lost_cnt = 0;        /* 连续未检测到斑马线帧计数(用于退出) */

    /* 第一步：固定中央窗口重扫跳变，检测斑马线是否已消失 */
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

    /* 斑马线特征消失时累计帧数，连续3帧确认退出 */
    if (exit_rows < 4)
    {
        lost_cnt++;
        if (lost_cnt >= 3)
        {
            ImageFlag.Zebra_Flag = 0;       /* 退出斑马线状态 */
            lost_cnt = 0;
            return;
        }
    }
    else
    {
        lost_cnt = 0;                       /* 仍在斑马线内，重置 */
    }

    /* 第二步：直道斑马线——锁定中线到图像中点，防止条纹干扰边界检测 */
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


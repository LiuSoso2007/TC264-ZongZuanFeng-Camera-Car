/*
 * Camera.c --- MT9V03X 摄像头驱动 + 图像压缩 + OTSU二值化 + IPS200调试显示
 *
 * OTSU修复: 移除提前退出, 完整遍历256级 + 阈值限幅30~220 (防直道全黑)
 * 帧率优化: 新增 Camera_ShowBinaryFast() 仅显示二值图, SPI传输量最小
 */
#include "Camera.h"
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
ImageDealDatatypedef ImageDeal[LCDH];        // ??????
ImageStatustypedef ImageStatus;              // ????(OFFLine/????)
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/*
 * Camera_CompressInit - 建立 Image_Use 到 mt9v03x_image 的指针映射
 */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/*
 * Camera_OTSU_GetThreshold - 大津法求阈值
 * 已移除提前退出优化 (直道场景下类间方差曲线可能有局部峰, 提前break会截在错误位置)
 * 完整遍历0~255, 输出限幅OTSU_MIN~OTSU_MAX
 */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row) {
    #define GRAY_SCALE  256
    #define OTSU_MIN   30                       /* ???? */
    #define OTSU_MAX   220                      /* ???? */
    uint16 width  = col;
    uint16 height = row;
    uint32 pixelSum;
    static uint32 pixelCount[GRAY_SCALE];        /* static: ?.bss, ?????(~1KB) */
    static float  pixelPro[GRAY_SCALE];          /* static: ?.bss, ?????(~1KB) */
    uint32 gray_sum = 0;
    uint8  threshold = 128;
    uint16 i, j;

    pixelSum = (uint32)width * height;
    if (pixelSum == 0) return threshold;

    for (i = 0; i < GRAY_SCALE; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i]  = 0.0f;
    }

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            uint8 gray_val = *image[i][j];
            pixelCount[gray_val]++;
            gray_sum += gray_val;
        }
    }

    for (i = 0; i < GRAY_SCALE; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / (float)pixelSum;
    }

    {
        float w0 = 0.0f, gray_avg = (float)gray_sum / (float)pixelSum;
        float u0tmp = 0.0f, deltaMax = 0.0f;
        uint8  jj;

        for (jj = 0; jj < GRAY_SCALE; jj++)
        {
            w0    += pixelPro[jj];
            u0tmp += (float)jj * pixelPro[jj];

            if (w0 < 1e-6f || (1.0f - w0) < 1e-6f) continue;

            float w1    = 1.0f - w0;
            float u1tmp = gray_avg - u0tmp;
            float u0    = u0tmp / w0;
            float u1    = u1tmp / w1;
            float deltaTmp = w0 * (u0 - gray_avg) * (u0 - gray_avg)
                           + w1 * (u1 - gray_avg) * (u1 - gray_avg);

            if (deltaTmp > deltaMax)
            {
                deltaMax  = deltaTmp;
                threshold = jj;
            }
        }
    }

    if (threshold < OTSU_MIN) threshold = OTSU_MIN;
    if (threshold > OTSU_MAX) threshold = OTSU_MAX;
    return threshold;
}

/*
 * Camera_GetBinaryImage - 灰度图二值化
 */
void Camera_GetBinaryImage(void) {
    uint8 thr = Camera_OTSU_GetThreshold(Image_Use, (uint16)LCDW, (uint16)LCDH);
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

/*
 * Camera_ShowDebug - 全量调试显示 (慢, 建议每3~5帧调用一次)
 * 上: 原始灰度 188x120 / 中: OTSU阈值 / 下: 二值图 94x60
 */
void Camera_ShowDebug(void) {
    uint16 xo;
    /* 原始灰度图 */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* OTSU 阈值 */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* 二值化图像 */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* 图例 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(2, 215, "[0=黑 1=白]");
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
//  @brief          Get_BaseLine - ??????
//  ??: ???59~57, ??56????????5?(56->52)
//  ?????(ImageSensorMid=47)????, ???????(0,0)????
//  5???????, ??????
//  ?? Pixle[][] ????? (0=?/??, 1=?/??)
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // ???????
    int   Xsite;                                // ?????
    int   row;                                  // ?????

    /* ---- ?1?: ???56? (???) ---- */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // ???56?

    // ????, ????
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

    // ????, ????
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

    // ???56???????
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* ????????'T', ??????'F' */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ---- ?2?: ?????55->52? ---- */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // ????
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
                ImageDeal[row].IsRightFind = 'F';   // ?????????
                break;
            }
        }

        // ????
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
                ImageDeal[row].IsLeftFind = 'F';    // ?????????
                break;
            }
        }

        // ??????????
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* ????????'T', ??????'F' */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* ---- ?3?: 5??????? (?????) ---- */
    // TODO: ????????????????????????
}

//-------------------------------------------------------------------------------
//  @brief          Get_Border_And_SideType - ?????????
//  @brief          ???[L, H]???, ??->?????????
//  @brief          ??: 'T'=????, 'W'=??(??), 'H'=???
//  @parameter      p    ???????
//  @parameter      type ????: 'L'=???, 'R'=???
//  @parameter      L, H ???????
//  @parameter      Q    ?????
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* ---- ????: ??L/H???????[0, LCDW-1] ---- */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // ?????: ?????
    {
        for (i = H; i >= L; i--)
        {
            // ?(1)->?(0)??: ???, ???
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // ?????????
                Q->type  = 'T';                 // ????
                break;
            }
            else if (i == (L + 1))              // ????????
            {
                if (*(p + (L + H) / 2) != 0)    // ??????
                {
                    Q->point = (L + H) / 2;     // ?????
                    Q->type  = 'W';             // ???(??)
                }
                else                            // ??????
                {
                    Q->point = H;               // ?????
                    Q->type  = 'H';             // ????
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // ?????: ?????
    {
        for (i = L; i <= H; i++)
        {
            // ?(1)->?(0)??: ???, ???
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // ?????????
                Q->type  = 'T';                 // ????
                break;
            }
            else if (i == (H - 1))              // ????????
            {
                if (*(p + (L + H) / 2) != 0)    // ??????
                {
                    Q->point = (L + H) / 2;     // ?????
                    Q->type  = 'W';             // ???(??)
                }
                else                            // ??????
                {
                    Q->point = L;               // ?????
                    Q->type  = 'H';             // ????
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
//  @brief          Get_AllLine - ????????
//  @brief          Get_BaseLine(56->52)?, ??51????52?????????????0
//  @brief          ??????: ???????????+/-ImageScanInterval????
//  @brief          ????: ????????????????; ????????OFFLine??
//  @parameter      void
//  @return         void
//  @note           ?? ImageDeal[52] (??????) ? Pixle[][] (?????)
//  @note           OFFLine: ?????????????, ?????????????
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // ???????
    int   row;                                  // ?????
    int   IntervalLow, IntervalHigh;            // ???????
    int   i;                                    // ????

    /* ---- ??????? ---- */
    ImageStatus.OFFLine          = 2;           // ?????(???2?)
    ImageStatus.Miss_Left_lines  = 0;           // ?????
    ImageStatus.Miss_Right_lines = 0;           // ?????
    ImageStatus.WhiteLine        = 0;           // ??????
    ImageStatus.WhiteLine_L      = 0;           // ?????
    ImageStatus.WhiteLine_R      = 0;           // ?????
    ImageStatus.OFFLineBoundary  = 0;           // ?????
    ImageStatus.Det_True         = 0;           // ??????

    /*
     * ??51??, ??52(??????)??????
     * ?????????0?OFFLine??
     */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // [0]=?, [1]=?
        PicTemp = Pixle[row];

        /* ============================================================
         * ?????: ??????? +/- ImageScanInterval ????
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // ???[0, 93]
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ============================================================
         * ?????: ??????? +/- ImageScanInterval ????
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ============================================================
         * ????????????
         * 'T'=??: ??????
         * 'W'=??: ??????? (??+1)
         * 'H'=???: ?????????, ?????
         * ============================================================ */
        if (JumpPoint[0].type == 'W')           // ??????
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // ????
            ImageStatus.Miss_Left_lines++;      // ?????
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // ????, ??????
        }

        if (JumpPoint[1].type == 'W')           // ??????
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // ????
            ImageStatus.Miss_Right_lines++;     // ?????
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // ????, ??????
        }

        /* ---- ???????? ---- */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ---- ??????(?????) ---- */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // ??????
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* ?????? */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ---- ?????????? ---- */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /*
         * H?????: ????????????
         * ????????H?, ???????????
         */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ---- ?H?: ????+1??????????????? ---- */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 0)    // ????
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ---- ?H?: ????-1??????????????? ---- */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 0)    // ????
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ---- ????????? ---- */
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ============================================================
         * OFFLine????: ???????????????
         * ????????????, ??????????
         * ============================================================ */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3)
        {
            ImageStatus.OFFLine = row;          // ????
            break;
        }
    }
}



/* ================================================================
 * ????? (TC264: 94?, AnCai?x1.175??)
 * ================================================================ */
const uint8 Half_Road_Wide[60] = {           /* ????(???->???) */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* ???? */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* ================================================================
 * ????
 * ================================================================ */
ImageFlagtypedef ImageFlag;                  /* ???? */

/* ================================================================
 * Helper: Straight_Judge - ?????
 * dir=1: ??????, dir=2: ??????
 * ????S, S<1 ????
 * ================================================================ */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* ??? */
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
    case 2: /* ??? */
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

/* ================================================================
 * ?????/??
 * ================================================================ */
void Straight_long_judge(void)
{
    if (ImageFlag.Bend_Road || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1
        || ImageFlag.image_element_rings)
        return;

    if ((Straight_Judge(1, 10, 50) < 1.0f)
     && (Straight_Judge(2, 10, 50) < 1.0f)
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

    if ((Straight_Judge(1, 10, 50) > 1.0f)
     || (Straight_Judge(2, 10, 50) > 1.0f)
     || ImageStatus.OFFLine >= 3
     || ImageStatus.Miss_Left_lines >= 2
     || ImageStatus.Miss_Right_lines >= 2)
    {
        ImageFlag.straight_long = 0;
    }
}

/* ================================================================
 * ?????? (????? + ?????)
 * ================================================================ */
void Straight_xie_judge(void)
{
    float S, Sum, Err, midd_k;
    int i;

    if (ImageFlag.Zebra_Flag != 0 || ImageFlag.image_element_rings != 0
        || ImageFlag.Ramp == 1)
        return;

    ImageFlag.straight_xie = 0;

    if (ImageStatus.OFFLine >= 10) return;

    midd_k = (float)(ImageDeal[55].Center - ImageDeal[ImageStatus.OFFLine + 1].Center)
           / (float)(55 - ImageStatus.OFFLine - 1);
    Sum = 0.0f;
    for (i = 0; i < 55 - ImageStatus.OFFLine - 1; i++)
    {
        Err = (ImageDeal[ImageStatus.OFFLine + 1].Center + midd_k * i
             - ImageDeal[i + ImageStatus.OFFLine + 1].Center);
        Sum += Err * Err;
    }
    S = Sum / (float)(55 - ImageStatus.OFFLine - 1);

    if (S < 1.0f && ImageStatus.OFFLine < 10
     && (ImageStatus.Miss_Left_lines > 30 || ImageStatus.Miss_Right_lines > 30))
    {
        ImageFlag.straight_xie = 1;
    }
}

/* ================================================================
 * ???? (??????+???)
 * ================================================================ */
void Element_Judgment_Bend(void)
{
    if (ImageFlag.image_element_rings != 0 || ImageStatus.OFFLine < 14
        || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1)
        return;

    /* ??: ?????(>30), ????, ?????? */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 30
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 8
     && Straight_Judge(1, ImageStatus.OFFLine + 2, 58) > 1.0f)
    {
        ImageFlag.Bend_Road = 1;              /* ?? */
    }

    /* ??: ?????(<50), ????, ?????? */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 50
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 8
     && Straight_Judge(2, ImageStatus.OFFLine + 2, 58) > 1.0f)
    {
        ImageFlag.Bend_Road = 2;              /* ?? */
    }
}

/* ================================================================
 * ????: ??????????
 * ================================================================ */
void Element_Handle_Bend(void)
{
    int row;                                  /* ??int???? */
    if (ImageStatus.OFFLine < 10) { ImageFlag.Bend_Road = 0; return; }

    if (ImageFlag.Bend_Road == 1)             /* ??: center=???+???? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* ?? <= 93 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* ??: center=???-???? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* ?? >= 0 */
        }
    }
}

/* ================================================================
 * ?????
 * ================================================================ */
void Element_Judgment_Left_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Left_Less_Num = 0;

    if (ImageStatus.Miss_Right_lines > 3 || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 5 || Straight_Judge(2, 5, 55) > 1.0f
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    /* ????????????'W'?(??) */
    {
        int r;
        for (r = 56; r >= 52; r--)           /* TC264: ???56->52 */
        {
            if (ImageDeal[r].IsLeftFind == 'W') return;
        }
    }

    /* ?????????? */
    for (Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 1].LeftBorder > 4)
        {
            Left_Less_Num++;
            /* ???????? */
            if (Left_Less_Num == 1) {
                /* ???????, ??????: ?????? */
            }
        }
    }

    if (Left_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 1;    /* ??? */
        ImageFlag.image_element_rings_flag = 1;
    }
}

/* ================================================================
 * ????? (????)
 * ================================================================ */
void Element_Judgment_Right_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Right_Less_Num = 0;

    if (ImageStatus.Miss_Left_lines > 3 || ImageStatus.Miss_Right_lines < 13
        || ImageStatus.OFFLine > 5 || Straight_Judge(1, 5, 55) > 1.0f
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    {
        int r;
        for (r = 56; r >= 52; r--)
        {
            if (ImageDeal[r].IsRightFind == 'W') return;
        }
    }

    for (Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBorder - ImageDeal[Ysite].RightBorder > 4)
        {
            Right_Less_Num++;
        }
    }

    if (Right_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 2;    /* ??? */
        ImageFlag.image_element_rings_flag = 2;
    }
}

/* ================================================================
 * ?????: ????????
 * ================================================================ */
void Element_Handle_Left_Rings(void)
{
    int row;

    if (ImageFlag.image_element_rings_flag == 1)
    {
        /* ???: ???? (LeftBorder + ????) */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }

    /* ????: OFFLine?? (????) */
    if (ImageStatus.OFFLine >= 15)
    {
        ImageFlag.image_element_rings = 0;
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.ring_big_small = 0;
    }
}

/* ================================================================
 * ?????
 * ================================================================ */
void Element_Handle_Right_Rings(void)
{
    int row;

    if (ImageFlag.image_element_rings_flag == 2)
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }

    if (ImageStatus.OFFLine >= 15)
    {
        ImageFlag.image_element_rings = 0;
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.ring_big_small = 0;
    }
}

/* ================================================================
 * ?????: ????20~32????????
 * ================================================================ */
void Element_Judgment_Zebra(void)
{
    int Ysite, Xsite, net, NUM = 0;

    if (ImageFlag.Zebra_Flag || ImageFlag.image_element_rings
        || ImageFlag.Out_Road == 1)
        return;

    if (ImageStatus.OFFLineBoundary >= 20) return;

    for (Ysite = 20; Ysite < 33; Ysite++)
    {
        net = 0;
        for (Xsite = ImageDeal[Ysite].LeftBorder + 2;
             Xsite < ImageDeal[Ysite].RightBorder - 2; Xsite++)
        {
            if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
            {
                net++;
                if (net > 4) NUM++;
            }
        }
    }

    if (NUM > 8)                              /* ?????: ?????? */
    {
        if (ImageDeal[SCAN_BASE_START_ROW].Center > 47)  /* TC264: ??????56???59 */        /* ???? -> ??? */
            ImageFlag.Zebra_Flag = 1;
        else                                  /* ???? -> ??? */
            ImageFlag.Zebra_Flag = 2;
    }
}

/* ================================================================
 * ?????: ???????????
 * ================================================================ */
void Element_Handle_Zebra(void)
{
    int row;

    if (ImageFlag.Zebra_Flag == 1)            /* ???: ???? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Road_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }
    else if (ImageFlag.Zebra_Flag == 2)       /* ???: ???? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Road_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }
}

/* ================================================================
 * ????: OFFLine?? + ??? + ??????
 * ================================================================ */
void Element_Judgment_Ramp(void)
{
    int Ysite;
    int i = 0;                           /* ??????? */

    if (ImageStatus.WhiteLine >= 3) return;

    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if (ImageDeal[Ysite].Wide > 18
             && ImageDeal[Ysite].IsRightFind == 'T'
             && ImageDeal[Ysite].IsLeftFind == 'T'
             && ImageDeal[Ysite].LeftBorder < 40
             && ImageDeal[Ysite].RightBorder > 55   /* TC264: >55(?>40) */
             && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center - 2] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center + 2] == 1
             && ImageStatus.Miss_Left_lines < 7
             && ImageStatus.Miss_Right_lines < 7)
            {
                i++;
            }
        }

        if (i >= 3)                           /* ??3??? */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ================================================================
 * ????
 * ================================================================ */
void Element_Handle_Ramp(void)
{
    /* ??????????, ??????????? */
    /* ????: ????imu?????? */
}

/* ================================================================
 * ????: OFFLine?? + ???????????
 * ================================================================ */
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

/* ================================================================
 * ????: ?????????
 * ================================================================ */
void Element_Handle_OutRoad(void)
{
    int Ysite, Xsite;
    int gray_sum = 0;

    /* ???????? (???) */
    for (Ysite = 35; Ysite < 55; Ysite++)
    {
        for (Xsite = 30; Xsite < 64; Xsite++) /* TC264???? */
        {
            gray_sum += Pixle[Ysite][Xsite];
        }
    }

    /* ????? -> ???? */
    if (gray_sum > 400 && ImageStatus.OFFLine < 20)
    {
        ImageFlag.Out_Road = 0;
    }
}

/* ================================================================
 * ?????: ????????, ????????
 * ================================================================ */
void Get_ExtensionLine(void)
{
    int Ysite, TFSite = 55;
    int left_FTSite = 0, right_FTSite = 0;

    if (ImageStatus.WhiteLine < 8) return;

    /* ????? */
    for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
    {
        if (ImageDeal[Ysite].IsLeftFind == 'W')
        {
            if (ImageDeal[Ysite + 1].LeftBorder >= 70)
            {
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
            /* ?????? */
            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].LeftBorder;
        }
    }

    /* ????? */
    for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
    {
        if (ImageDeal[Ysite].IsRightFind == 'W')
        {
            if (ImageDeal[Ysite + 1].RightBorder <= 23)   /* TC264: ????? */
            {
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].RightBorder;
        }
    }

    /* ?????? */
    for (Ysite = TFSite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder
                                 + ImageDeal[Ysite].RightBorder) / 2;
    }
}

/* ================================================================
 * ??????: ???????????
 * ??: ????????????????
 * ================================================================ */
void Scan_Element(void)
{
    /* ???????????????? */
    if (ImageFlag.Out_Road == 0 && ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0 && ImageFlag.Bend_Road == 0
     && ImageFlag.straight_long == 0)
    {
        Element_Judgment_OutRoad();           /* ?? */
        Element_Judgment_Left_Rings();        /* ??? */
        Element_Judgment_Right_Rings();       /* ??? */
        Element_Judgment_Zebra();             /* ??? */
        Element_Judgment_Bend();              /* ?? */
        Element_Judgment_Ramp();              /* ?? */
        Straight_long_judge();                /* ??? */
    }

    /* ????????? */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();
        if (ImageFlag.Out_Road) ImageFlag.Bend_Road = 0;
    }

    /* ???????? */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ================================================================
 * ??????: ?????????????
 * ================================================================ */
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
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
    else if (ImageStatus.WhiteLine >= 8)
        Get_ExtensionLine();                  /* ????? */
}

/* ================================================================
 * ?????
 * ================================================================ */
void Flag_init(void)
{
    ImageFlag.Bend_Road              = 0;
    ImageFlag.Zebra_Flag             = 0;
    ImageFlag.Ramp                   = 0;
    ImageFlag.image_element_rings    = 0;
    ImageFlag.image_element_rings_flag = 0;
    ImageFlag.straight_xie           = 0;
    ImageFlag.straight_long          = 0;
    ImageFlag.ring_big_small         = 0;
    ImageFlag.Out_Road               = 0;
}


//-------------------------------------------------------------------------------
//  @brief          Camera_ShowElementStatus - ????????
//  @brief          ?IPS200???????????(????????)
//  @brief          zhi=?? wan_L/R=?? shi=?? huan_L/R=?? banma=??? po=?? duan=??
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* ???: ????, ?? */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /*
     * ????????????, ???????
     * ??: y=225 (??240??, ??15px??)
     */
    if (ImageFlag.Out_Road != 0)
    {
        ips200_show_string(2, 225, "ELEM: duan   ");     /* ?? */
    }
    else if (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: huan_L ");     /* ??? */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: huan_R ");     /* ??? */
    }
    else if (ImageFlag.Zebra_Flag == 1)
    {
        ips200_show_string(2, 225, "ELEM: banma_L");     /* ???-??? */
    }
    else if (ImageFlag.Zebra_Flag == 2)
    {
        ips200_show_string(2, 225, "ELEM: banma_R");     /* ???-??? */
    }
    else if (ImageFlag.Ramp != 0)
    {
        ips200_show_string(2, 225, "ELEM: po     ");     /* ?? */
    }
    else if (ImageFlag.Bend_Road == 1)
    {
        ips200_show_string(2, 225, "ELEM: wan_L  ");     /* ?? */
    }
    else if (ImageFlag.Bend_Road == 2)
    {
        ips200_show_string(2, 225, "ELEM: wan_R  ");     /* ?? */
    }
    else if (ImageFlag.straight_long)
    {
        ips200_show_string(2, 225, "ELEM: zhi    ");     /* ??? */
    }
    else if (ImageFlag.straight_xie)
    {
        ips200_show_string(2, 225, "ELEM: xie    ");     /* ???? */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* ?? */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* ??? */
    }

    /* ?????? */
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


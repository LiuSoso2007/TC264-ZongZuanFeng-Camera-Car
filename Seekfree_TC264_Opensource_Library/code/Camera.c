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
    #define GRAY_SCALE 256
    #define OTSU_MIN   30    /* 阈值下限: 低于此值赛道全白, 无意义 */
    #define OTSU_MAX   220   /* 阈值上限: 高于此值赛道全黑, 无意义 */
    uint16 w = col, h = row;
    uint32 ps = (uint32)w * h, pc[GRAY_SCALE], gs = 0;
    float  pp[GRAY_SCALE];
    uint8  thr = 128;
    uint16 i, j;

    if (ps == 0) return thr;

    /* 初始化直方图 */
    for (i = 0; i < GRAY_SCALE; i++) { pc[i] = 0; pp[i] = 0.0f; }

    /* 统计直方图 + 灰度总和 */
    for (i = 0; i < h; i++)
        for (j = 0; j < w; j++) {
            uint8 g = *image[i][j]; pc[g]++; gs += g; }

    /* 灰度比例 */
    for (i = 0; i < GRAY_SCALE; i++)
        pp[i] = (float)pc[i] / (float)ps;

    /* OTSU 完整遍历 0~255 (不做提前退出) */
    {
        float w0 = 0.0f, ut = 0.0f, ga = (float)gs / (float)ps, dm = 0.0f;
        uint8 jj;
        for (jj = 0; jj < GRAY_SCALE; jj++) {
            w0 += pp[jj];
            ut += (float)jj * pp[jj];
            if (w0 < 1e-6f || (1.0f - w0) < 1e-6f) continue;
            float w1 = 1.0f - w0, u1t = ga - ut;
            float u0 = ut / w0, u1 = u1t / w1;
            float dt = w0 * (u0 - ga) * (u0 - ga) + w1 * (u1 - ga) * (u1 - ga);
            if (dt > dm) { dm = dt; thr = jj; }
        }
    }

    /* 输出限幅 */
    if (thr < OTSU_MIN) thr = OTSU_MIN;
    if (thr > OTSU_MAX) thr = OTSU_MAX;
    return thr;
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

/*
 * Camera_ShowBinaryFast - 仅显示二值化图像 (快, SPI传输量最小)
 * 不传原始灰度图, 不显示阈值文字, 适合帧率优先场景
 */
void Camera_ShowBinaryFast(void) {
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 0, Pixle[0], LCDW, LCDH, LCDW, LCDH, 0);
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
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 0);
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
    ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
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
                break;
            }
        }

        // ??????????
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        ImageDeal[row].IsLeftFind  = 'T';
        ImageDeal[row].IsRightFind = 'T';
    }

    /* ---- ?3?: 5??????? (?????) ---- */
    // TODO: ????????????????????????
}

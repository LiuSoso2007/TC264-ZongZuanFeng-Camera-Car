/* ï¿½ï¿½ï¿½ï¿½Í·Í¼ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½Ëµï¿½ï¿½ */
#include "Camera.h"
#include "Shared.h"
static uint16 s_ring_state_frames = 0U;      /* ï¿½ï¿½Ç°ï¿½×¶ï¿½ï¿½Ñ¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ö¡ï¿½ï¿½ */
static uint8 s_ring_confirm_count = 0U;      /* ï¿½ï¿½ï¿½ï¿½Ê¶ï¿½ï¿½È·ï¿½ï¿½Ö¡ï¿½ï¿½ */
static uint8 s_ring_feature_count = 0U;      /* ï¿½ï¿½Ú»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È·ï¿½ï¿½Ö¡ï¿½ï¿½ */
static uint8 s_ring_stable_count = 0U;       /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È¶ï¿½Ö±ï¿½ï¿½Ö¡ï¿½ï¿½ */
static uint8 s_ring_exit_loss_seen = 0U;     /* ³ö»·Ê±ÊÇ·ñÒÑ¹Û²ìµ½¶Ô²à¶ªÏß */
static int s_ring_entry_corner_row = -1;     /* ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½Ú¹Õµï¿½ï¿½ï¿½ */
static int s_ring_entry_corner_col = -1;     /* ????????? */
static uint8 s_ring_edge_squeezed = 0U;      /* row50 edge squeezed */
static uint8 s_ring_edge_released = 0U;      /* row50 edge released */
static int s_ring_prev_valley_row = -1;     /* ÉÏÒ»Ö¡¹Èµ×ĞĞºÅ, APPROACH½×¶ÎÓÃ */
volatile int g_corner_black_max = 0;   /* ??????: ???????? */
volatile int g_bottom_black_width = 0; /* ??????: W-B???? */
volatile int g_ring_miss_cnt = 0;      /* ????????(Miss_Left?Miss_Right) */
volatile uint8 g_left_jump_count = 0;   /* left border jump count */
volatile uint8 g_right_jump_count = 0;  /* right border jump count */
volatile int g_approach_valley_row = -99; /* debug: approach valley row */
volatile uint8 g_edge_squeezed_dbg = 0;  /* debug: squeeze state */
volatile uint8 g_ring_phase_dbg = 0;     /* debug: valley phase 0/1/2 */
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
int16_t g_ZebraSum = 0;                 /* ï¿½ï¿½ï¿½ï¿½ï¿½ß¼ï¿½ï¿½ï¿½ÖµÖ®ï¿½ï¿½ */
ImageDealDatatypedef ImageDeal[LCDH];        // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ImageStatustypedef ImageStatus;              // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Camera_CompressInitï¿½ï¿½ */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/* »Ò¶ÈÖ±·½Í¼+OTSU´ó½ò·¨¼ÆËã×ÔÊÊÓ¦ãĞÖµ */
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

    /* µÚÒ»±éÉ¨ÃèÈ«Í¼£¬ÕÒµ½×îĞ¡ºÍ×î´ó»Ò¶È */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++) {
            uint8 v = *image[i][j];
            if (v < pmin) pmin = v;
            if (v > pmax) pmax = v;
        }

    range = pmax - pmin;

    /* Í³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½Í¼ï¿½ï¿½ï¿½Ô±È¶ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ìµ½[0,255]ï¿½ï¿½ï¿½ï¿½OTSUï¿½ï¿½ */
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

    /* OTSUËã·¨ºËĞÄ£º±éÀúãĞÖµ£¬×î´ó»¯Àà¼ä·½²î */
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

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÖµÓ³ï¿½ï¿½ï¿½Ô­Ê¼ï¿½Ò¶È·ï¿½Î§ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½Ê¹ï¿½Ã¡ï¿½ */
    if (range > 30) {
        bestThr = (uint8)(pmin + ((uint16)bestThr * range) / 255U);
    }

    /* Ç¯Î»ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½Î§ï¿½ï¿½ */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    bestThr += OTSU_BIAS;
    return bestThr;
}

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Camera_GetBinaryImageï¿½ï¿½ */
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

/* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Camera_DrawCenterLinesï¿½ï¿½ */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);  /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_draw_line(94, 0, 94, 119, RGB565_RED);
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    /* ï¿½ï¿½ï¿½ï¿½ï¿½Ëµã¶¼ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½OFFLineï¿½ï¿½ï¿½Ïµï¿½ï¿½ï¿½Ğ§ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

void Camera_ShowDebug(void) {
    uint16 xo;
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    /* legend removed */
    Camera_ShowElementStatus();
    
    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    int   Xsite;                                // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    int   row;                                  // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
                ImageDeal[row].IsRightFind = 'F';   // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                break;
            }
        }

        // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
                ImageDeal[row].IsLeftFind = 'F';    // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                break;
            }
        }

        // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
}

//-------------------------------------------------------------------------------
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    {
        for (i = H; i >= L; i--)
        {
            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                Q->type  = 'T';                 // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                break;
            }
            else if (i == L)                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            {
                if (*(p + (L + H) / 2) != 0)    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    Q->type  = 'W';             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                }
                else                            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    Q->type  = 'H';             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    {
        for (i = L; i <= H; i++)
        {
            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                Q->type  = 'T';                 // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                break;
            }
            else if (i == H)                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            {
                if (*(p + (L + H) / 2) != 0)    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    Q->type  = 'W';             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                }
                else                            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    Q->type  = 'H';             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  @parameter      void
//  @return         void
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    int   row;                                  // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    int   IntervalLow, IntervalHigh;            // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    int   i;                                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    ImageStatus.OFFLine          = 2;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.Miss_Left_lines  = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.Miss_Right_lines = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.WhiteLine        = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.WhiteLine_L      = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.WhiteLine_R      = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.OFFLineBoundary  = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
    ImageStatus.Det_True         = 0;           // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬

    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        PicTemp = Pixle[row];

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        if (JumpPoint[0].type == 'W')           // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            ImageStatus.Miss_Left_lines++;      // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
        }
        else                                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
        }

        if (JumpPoint[1].type == 'W')           // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // ï¿½ï¿½ï¿½Âµï¿½Ç°É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            ImageStatus.Miss_Right_lines++;     // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
        }
        else                                    // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
        }

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)  // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)  // ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
            /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))  /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
        {
            ImageStatus.OFFLine = row;          // ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬
            break;
        }

        /*
         * ï¿½ï¿½ï¿½ï¿½Í¬Ô´ï¿½ï¿½ï¿½ï¿½: Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½È¹ï¿½Õ­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±Í£Ö¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×·ï¿½ß¡ï¿
         * TC264Îª94ï¿½ï¿½, ï¿½É°ï¿½ï¿½ï¿½80ï¿½ï¿½ï¿½ï¿½Öµ(7/10/70)ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó³ï¿½ï¿½Îª8/12/82ï¿½ï¿½
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



/* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
const uint8 Half_Road_Wide[60] = {           /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
ImageFlagtypedef ImageFlag;                  /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Straight_Judgeï¿½ï¿½ */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
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
    case 2: /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½ĞµÄ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ¡ï¿½ */
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

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Straight_long_judgeï¿½ï¿½ */
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

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Straight_xie_judgeï¿½ï¿½ */
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

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Judgment_Bendï¿½ï¿½ */
void Element_Judgment_Bend(void)
{
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag)
        return;
    /* ponytailÖ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: OFFLine<5Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È«ï¿½É¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿
       ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Missï¿½ï¿½ï¿½ï¿½ï¿½Û»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ (ï¿½ï¿½ï¿½ï¿½Ô­Ê¼OFFLine>=14, TC264ï¿½ï¿½ï¿½ï¿½60ï¿½ï¿½->5) */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;  /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */

    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;              /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }

    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;              /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }
}

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handle_Bendï¿½ï¿½ */
void Element_Handle_Bend(void)
{
    int row;                                  /* ï¿½ï¿½intï¿½ï¿½ï¿½ï¿½ucharï¿½ï¿½Ö§ï¿½Ö´ï¿½Î§Ñ­ï¿½ï¿½ */

    /* ponytailï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: OFFLine<5Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È«ï¿½É¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾ï¿½ï¿½ï¿½Ë³ï¿½
       ï¿½ï¿½Element_Judgment_Bendï¿½ï¿½OFFLineï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½Ë«ï¿½ï¿½ï¿½Õ·ï¿½Ö¹Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿ */
    if (ImageStatus.OFFLine < 5)
        { ImageFlag.Bend_Road = 0; return; }

    /* Ë«ï¿½à¶¼×·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ -> ï¿½Ñ»Ö¸ï¿½Ö±ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾ */
    if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4)
        { ImageFlag.Bend_Road = 0; return; }

if (ImageFlag.Bend_Road == 1)             /* ×óÍäµÀ */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* é™å¹… >= 0 */
        }
    }
else if (ImageFlag.Bend_Road == 2)        /* ÓÒÍäµÀ */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* é™å¹… <= 93 */
        }
    }
}

/* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
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
    s_ring_edge_squeezed = 0U;
    s_ring_edge_released = 0U;
}


/*
 *******************************************************************************************
 ** ï¿½Ú¶ï¿½ï¿½ï¿½â· ï¿½ï¿½ï¿½ï¿½ Ô²ï¿½ï¿½Ê¶ï¿½ï¿½ï¿½ï¿½Äºï¿½ï¿½ï¿½ï¿½ï¿
 ** ï¿½ï¿½ï¿½ï¿½ hao-yue-1/SmartCar (ï¿½ã¶«ï¿½ï¿½Òµï¿½ï¿½Ñ§ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿) ï¿½ï¿½ï¿½Ó¾ï¿½ï¿½ï¿½ï¿½ï¿½
 ** ï¿½ï¿½ï¿½ï¿½ TC264 + MT9V03X + 94x60Ñ¹ï¿½ï¿½Í¼
 ** ï¿½ï¿½IMUï¿½ï¿½ï¿½Şµï¿½ï¿ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ÖµÍ¼ï¿½ï¿½ï¿½Ø·ï¿½ï¿½ï¿½
 *******************************************************************************************
 */

/* ---- ï¿½Ú¶ï¿½ï¿½×²ï¿½ï¿½ï¿½â£ºï¿½ï¿½ï¿½Í¼ï¿½ï¿½×²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½Úºï¿½É«ï¿½ï¿½ï¿½ï¿½ ---- */
/* ---- ?????????????????????????? ---- */
static uint8 BlackHole_Check_Corner(uint8 direction)
{
    int row, col, black_cnt;
    int start_col, end_col;

    /* ponytail: ?????????????????????
       ?? BlackHole_Check_Bottom ????????? */
    if (direction == 1U) { start_col = 0; end_col = 9; }      /* ???: ??? */
    else                 { start_col = LCDW - 10; end_col = LCDW - 1; } /* ???: ??? */

    for (row = LCDH - 1; row >= LCDH - 6; row--)
    {
        black_cnt = 0;
        for (col = start_col; col <= end_col; col++)
        {
            if (Pixle[row][col] == IMG_BLACK)
                black_cnt++;
        }
        /* ????>=4???????????? */
        if (black_cnt > g_corner_black_max) g_corner_black_max = black_cnt;
        if (black_cnt >= 3)
            return 1;
    }
    return 0;
}
static uint8 BlackHole_Check_Bottom(uint8 direction)
{
    int row, col;
    int state;      /* 0=??, 1=??, 2=?? */
    int black_cnt;  /* ?????? */
    int start_col, end_col, step;

    /* ????3??LCDH-1(59), LCDH-2(58), LCDH-3(57) */
    for (row = LCDH - 1; row >= LCDH - 6; row--)
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
                                    g_bottom_black_width = black_cnt;
                if (state == 2 && black_cnt >= 3)
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


    /* ¹Èµ×ĞĞÓĞĞ§ÇÒÎ»ÓÚÌ½²âÇø¼äÄÚ */
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

    /* ¼ì²éÓÒ²àÊÇ·ñ´æÔÚºáÏòÈüµÀ(Ê®×ÖÌØÕ÷) */
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

    /* ºÚ¶´¼ì²â: Í³¼Æµ×²¿ºÚÉ«ÏñËØÊıÁ¿ */
static uint8 Ring_Is_Stable_Road(void)
{
    return (uint8)(ImageStatus.OFFLine <= 2
                && ImageStatus.Miss_Left_lines < 4
                && ImageStatus.Miss_Right_lines < 4
                && Straight_Judge(1, 5, SCAN_BASE_END_ROW) < 2.0f
                && Straight_Judge(2, 5, SCAN_BASE_END_ROW) < 2.0f);
}

    /* ¹Õ½ÇºÚ¶´¼ì²â: ¼ì²é²à±ßºÚÉ«ÇøÓò */
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

/* ---- Ñ°ï¿½ï¿½ï¿½ï¿½Ú¹Èµ×µã£ºï¿½Ã¹Èµï¿½×·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
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

/* ---- ENTRYé˜¶æµæ–°æ–¹æ: ä»å¹ä¾§è¾¹ç¼˜æ¨å‘æ‰«æ‰¾å…¥å£æ‹ç‚ ----
 * ä»ä¸‹å¾ä¸Šæ‰«æ¯è, ä»å¹ä¾§è¾¹ç¼˜å‡ºå‘å‘ç¯å²›æ–¹å‘æ‰«,
 * æ‰¾é»‘è‰²åŒºåŸŸçš„è¿œä¾§è¾¹è·³å˜ç‚¹ä½œä¸ºæ‹ç‚¹, è®°å½•è·³å˜ç‚¹ä¸èµ·ç‚¹çš„æ¨ªå‘è·ç¦.
 * ç›¸é‚»ä¸¤èŒè·ç¦»å·®ç»å¹å>10æ—, å–é ä¸(è¡Œæ•°å°)é‚£èŒçš„è·³å˜ç‚¹ä½œä¸ºå…¥å£æ‹ç‚.
 * direction=1(å·¦åœ†ç): ä»å³è¾¹ç¼˜å‘å·¦æ‰, æ‰¾é»‘è‰²åŒºåŸŸå·¦è¾¹ç¼˜(é»->ç™), å·¦è¾¹çš„è·³å˜ç‚¹ä½œä¸ºæ‹ç‚¹
 * direction=2(å³åœ†ç): ä»å·¦è¾¹ç¼˜å‘å³æ‰, æ‰¾é»‘è‰²åŒºåŸŸå³è¾¹ç¼˜(é»->ç™), å³è¾¹çš„è·³å˜ç‚¹ä½œä¸ºæ‹ç‚¹
 * è¿”å›: 1=æ‰¾åˆ°æ‹ç‚¹, 0=æœæ‰¾åˆ°; æ‹ç‚¹åæ ‡é€šè¿‡ corner_row/corner_col è¾“å‡º
 */
static uint8 Ring_Find_Entry_Corner(uint8 direction, int *corner_row, int *corner_col)
{
    int row;
    int prev_dist = -1, curr_dist;
    int jump_col;
    int col;
    uint8 in_black;

    for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
    {
        if (direction == 2U)
        {
    /* Ìø±äµãÎ»ÖÃ(Èë»·¹Õ½ÇÁĞ) */
            jump_col = LCDW - 1;
            in_black = 0U;
            for (col = 0; col < LCDW - 1; col++)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col + 1] == IMG_WHITE)
                    {
        jump_col = col;       /* ¼ÇÂ¼×ó±ß½çÌø±äÁĞ */
                        break;
                    }
                }
                else
                {
                    if (Pixle[row][col] == IMG_WHITE && Pixle[row][col + 1] == IMG_BLACK)
                    {
                        in_black = 1U;       /* è¿›å…¥é»‘è‰²åŒºåŸŸ */
                    }
                }
            }
        curr_dist = jump_col;         /* µ±Ç°Ìø±ä¾àÀë = Ìø±äÁĞ */
        }
        else
        {
    /* ×ó±ß½ç: ´ÓÉÏ¹ÕµãÑØÇĞÏß·½ÏòÑÓÉì±ß½ç */
            jump_col = 0;
            in_black = 0U;
            for (col = LCDW - 1; col > 0; col--)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col - 1] == IMG_WHITE)
                    {
        jump_col = col;       /* ¼ÇÂ¼ÓÒ±ß½çÌø±äÁĞ */
                        break;
                    }
                }
                else
                {
                    if (Pixle[row][col] == IMG_WHITE && Pixle[row][col - 1] == IMG_BLACK)
                    {
                        in_black = 1U;       /* è¿›å…¥é»‘è‰²åŒºåŸŸ */
                    }
                }
            }
        curr_dist = (LCDW - 1) - jump_col;  /* µ±Ç°Ìø±ä¾àÀë(´ÓÓÒ±ßËã) */
        }

    /* ÓÒ±ß½ç: ´ÓÉÏ¹ÕµãÑØÇĞÏß·½ÏòÑÓÉì±ß½ç */
        if (prev_dist >= 0
            && (curr_dist - prev_dist > 10 || prev_dist - curr_dist > 10))
        {
            *corner_row = row;
            *corner_col = jump_col;
            return 1U;
        }
        prev_dist = curr_dist;
    }
    return 0U;
}

/* ---- count border jumps between adjacent rows ---- */
static int Ring_Check_Border_Jump(uint8 direction, int threshold, int min_row, int max_row)
{
    int row;
    int prev_col = -1, curr_col;
    int count = 0;

    for (row = max_row; row > min_row; row--)
    {
        if (direction == 1U)
        {
            if (ImageDeal[row].IsLeftFind != 'T')
            {
                if (prev_col >= 0 && ImageDeal[row + 1].IsLeftFind == 'T')
                {
                    curr_col = ImageDeal[row + 1].LeftBorder;
                    if (curr_col - prev_col > threshold || curr_col - prev_col < -threshold)
                        count++;
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
                        count++;
                }
                prev_col = -1;
                continue;
            }
            curr_col = ImageDeal[row].RightBorder;
        }

        if (prev_col >= 0)
        {
            if (curr_col - prev_col > threshold || curr_col - prev_col < -threshold)
                count++;
        }
        prev_col = curr_col;
    }
    return count;
}

/* ---- APPROACH phase valley: two-scheme detection ---- */
static int Ring_Find_Approach_Valley(uint8 direction, int *valley_col)
{
    int row;
    int prev_col = 0, curr_col;
    uint8 moved_away = 0U;
    uint8 is_lost;

    /* Check row 50 state */
    int row50_at_edge;
    if (direction == 1U)
    {
        row50_at_edge = (ImageDeal[50].IsLeftFind != 'T')
                      || (ImageDeal[50].LeftBorder <= 10);
    }
    else
    {
        row50_at_edge = (ImageDeal[50].IsRightFind != 'T')
                      || (ImageDeal[50].RightBorder >= LCDW - 11);
    }

    /* ENTRY: Èë»·½×¶Î - È·ÈÏ¹Õ½ÇĞĞÓĞĞ§ºó½øÈë»·ÖĞ */
    if (row50_at_edge)
    {
        uint8 phase = 1U;  /* already squeezed, look for release */
        s_ring_edge_squeezed = 1U;
        g_ring_phase_dbg = 1U;
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

            int at_edge = (direction == 1U) ? (curr_col <= 10) : (curr_col >= LCDW - 11);

            if (phase == 1U)
            {
                if (!at_edge) { phase = 2U; prev_col = curr_col; }
                continue;
            }

            /* phase 2: find bounce */
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
    /* APPROACH: ½Ó½ü½×¶Î - ¼ì²âµ½×ã¹»¹Èµ×ºÚÉ«½øÈëÈë»· */
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
                            moved_away = 0U;
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

    /* CONFIRM: È·ÈÏ½×¶Î - ³ÖĞø¼ì²âÌØÕ÷Ö¡Êıºó½øÈë½Ó½ü */
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
    /* INSIDE: »·ÖĞ½×¶Î - µÈ´ı±ß½ç»Ö¸´ÎÈ¶¨ºó³ö»· */
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
                            moved_away = 0U;
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

    /* row 50 never been at edge */
    g_ring_phase_dbg = 0U;
    return -1;
}

/* ---- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½â£ºï¿½ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ß»Ö¸ï¿½ + ï¿½Ú¶ï¿½ï¿½ï¿½Ö¤ ---- */
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

/* ---- ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ï¿½ï¿½ ---- */
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
    int scan_arg, min_arg;
    if (ring_state == RING_STATE_ENTRY) { scan_arg = VALLEY_SCAN_START_ROW - 26; min_arg = VALLEY_MIN_ROW - 15; }
    else                                          { scan_arg = VALLEY_SCAN_START_ROW;      min_arg = VALLEY_MIN_ROW;      }
    int has_valley = BlackHole_Track_Valley(direction, &valley_row, &valley_col, scan_arg, min_arg);
    
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
        /* ??????????????????? */
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
    case RING_STATE_APPROACH:
        if (s_ring_entry_corner_row >= 0)
        {
            /* found valley: draw line from bottom corner to valley */
            if (direction == 1U)
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'L');
            else
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'R');
        }
        else if (s_ring_edge_squeezed)
        {
            /* squeezed but no valley yet: draw line from bottom corner to edge at row 50 */
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
    /* ¼ì²é¹Èµ×ÇøÓòÊÇ·ñÓĞÌø±ä(Ìø±äÉÙµÄÒ»²àÌØÕ÷Ã÷ÏÔ) */
        if (s_ring_entry_corner_row > 0)
        {
        if (direction == 1U) /* ÓÒ±ß½ç: ¼ì²é×ó²àÊÇ·ñ¿¿±ßÌ«¶à */
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'R');
        else                 /* ×ó±ß½ç: ¼ì²éÓÒ²àÊÇ·ñ¿¿±ßÌ«¶à */
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'L');
        }
        else
        {
    /* ¿¿±ßĞĞÊı > 4 Ôò·µ»Ø1 */
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
    case RING_STATE_EXIT:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] * 2 / 3 - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] * 2 / 3 - FILL_INSIDE_OFFSET;
            }
            else
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] * 2 / 3 + FILL_INSIDE_OFFSET;
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

/* ---- ×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
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
    /* Ô²»·´¦Àí: Èô·ÇÔ²»·Ôò¼ì²éÆäËûÔªËØ */
            if (valley_row > 40
                || (s_ring_prev_valley_row >= 0
                    && (valley_row - s_ring_prev_valley_row > 20
                        || s_ring_prev_valley_row - valley_row > 20)))
                Ring_Set_State(RING_STATE_ENTRY);
        }
        s_ring_prev_valley_row = valley_row;
        break;

    case RING_STATE_ENTRY:
        valley_row = -1;
    /* °ßÂíÏß/Ô²»·ÓÅÏÈ¼¶×î¸ß, ÒÑ´¥·¢Ê±²»ÖØ¸´¼ì²â */
        if (Ring_Find_Entry_Corner(direction, &valley_row, &valley_col))
        {
            s_ring_entry_corner_row = valley_row;
            s_ring_entry_corner_col = valley_col;
    /* ÆÂµÀ: ½öÔÚÖ±µÀ+ÎŞ»·+ÎŞ°ßÂíÏßÏÂ¼ì²â */
            if (valley_row > 48
             || (direction == 1U && valley_col >= LCDW - 15)
             || (direction == 2U && valley_col <= 15))
            {
                Ring_Set_State(RING_STATE_INSIDE);
                break;
            }
        }
        else
        {
    /* ³¤Ö±µÀÅĞ¶¨ */
            Ring_Set_State(RING_STATE_INSIDE);
            break;
        }
        /* è¶…æ—¶ä¿æŠ¤ */
        if (s_ring_state_frames >= RING_ENTRY_MAX_FRAMES)
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

    /* ÍäµÀ: ½öÔÚÎŞÔ²»·Ê±ÅĞ¶¨ */
/* ---- è·³å˜å°‘çš„é‚£è¾¹é è¾¹è¡Œæ•°æ£æŸ ----
 * æ£æŸ¥èŒèŒƒå› 15~45, è¾¹ç¼˜margin=8åƒç´ , é è¾¹è¡>5è¡Œåˆ™è¿”å›1
 * direction=1: æ£æŸ¥å³è¾¹ç•Œæ˜å¦å¤ªå¤šèŒæŒ¤åˆ°å³è¾¹ç¼˜
 * direction=2: æ£æŸ¥å·¦è¾¹ç•Œæ˜å¦å¤ªå¤šèŒæŒ¤åˆ°å·¦è¾¹ç¼˜
 */
static uint8 Ring_OtherSide_Too_Much_Edge(uint8 direction)
{
    int row;
    int edge_rows = 0;
    const int margin = 8;

    for (row = 45; row >= 15; row--)
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
    /* ÔªËØ´¦Àí: °´ÓÅÏÈ¼¶ÒÀ´Îµ÷ÓÃ */
    return (uint8)(edge_rows > 4);
}

void Element_Judgment_Left_Rings(void)
{
    if (ImageStatus.Miss_Right_lines > 15
        || ImageStatus.OFFLine > 16
        || ImageFlag.image_element_rings)
        return;

    if (g_left_jump_count >= 5
        && g_right_jump_count <= RING_JUMP_OTHER_MAX
        && !Ring_OtherSide_Too_Much_Edge(1U))
    {
        g_ring_miss_cnt = ImageStatus.Miss_Left_lines;
        ImageFlag.image_element_rings = 1;
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

    /* Ê®×Ö²¹Ïß: ĞŞ¸´Ê®×ÖÂ·¿Ú±ß½ç */
void Element_Judgment_Right_Rings(void)
{
    if (ImageStatus.Miss_Left_lines > 15
        || ImageStatus.OFFLine > 16
        || ImageFlag.image_element_rings)
        return;

    if (g_right_jump_count >= 5
        && g_left_jump_count <= RING_JUMP_OTHER_MAX
        && !Ring_OtherSide_Too_Much_Edge(2U))
    {
        g_ring_miss_cnt = ImageStatus.Miss_Right_lines;
        ImageFlag.image_element_rings = 2;
        Ring_Set_State(RING_STATE_CONFIRM);
    }
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
    int trans_count;        /* µ±Ç°ĞĞÌø±ä¼ÆÊı */
    int valid_rows = 0;     /* ï¿½ï¿½Ğ§ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½>=5ï¿½ï¿½ï¿½ï¿½) */
    static int confirm_cnt = 0;     /* ï¿½ï¿½ï¿½ï¿½È·ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½Ú·ï¿½ï¿½ï¿½) */

    /* ³¤Ö±µÀ´¦Àí */
    if (ImageFlag.image_element_rings
     || ImageFlag.Zebra_Flag != 0)
        return;

    /* ï¿½Ì¶ï¿½ï¿½ï¿½ï¿½ë´°ï¿½ï¿½É¨ï¿½ï¿½ï¿½ï¿½44~57ï¿½ï¿½ï¿½ï¿½Í³ï¿½Æºï¿½->ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(0->1)ï¿½ï¿½
     * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½Ğ£ï¿½Â·ï¿½ï¿½ï¿½ï¿½ï¿30~52pxï¿½ï¿½ï¿½ï¿½60pxï¿½ï¿½ï¿½ï¿½ï¿½Ú¡ï¿½
     * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß½ï¿½ï¿½â£¬ï¿½ï¿½È»ï¿½ï¿½ï¿½ï¿½Ê®ï¿½ï¿½Â·ï¿½Úºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿ */
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

    /* ï¿½ï¿½Ğ§ï¿½ï¿½>=5Ê±ï¿½ï¿½ï¿½Æ°ï¿½ï¿½ï¿½ï¿½ß£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½2Ö¡È·ï¿½Ï·ï¿½ï¿½ï¿½ï¿½ï¿½ */
    if (valid_rows >= 5)
    {
        confirm_cnt++;
        if (confirm_cnt >= 2)
        {
            ImageFlag.Zebra_Flag = 1;       /* È·ï¿½Ï°ï¿½ï¿½ï¿½ï¿½ï¿½ */
        }
    }
    else
    {
        confirm_cnt = 0;                     /* Î´ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È·ï¿½Ï¼ï¿½ï¿½ï¿½ */
    }
}











/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handle_Zebra */
/* °ßÂíÏß´¦Àí: ¼ì²â×´Ì¬+Ç¿ÖÆÖ±µÀÑ²Ïß */
/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handle_Zebra */
/* °ßÂíÏß´¦Àí: ¼ì²â×´Ì¬+Ç¿ÖÆÖ±µÀÑ²Ïß */
void Element_Handle_Zebra(void)
{
    int row, Ysite, Xsite;
    int trans_count;
    int exit_rows = 0;
    static int lost_cnt = 0;        /* °ßÂíÏß¶ªÊ§¼ÆÊıÆ÷ */

    /* ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½Ì¶ï¿½ï¿½ï¿½ï¿½ë´°ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½ï¿½ä£¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½Ê§ */
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

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê§Ê±ï¿½Û¼ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½3Ö¡È·ï¿½ï¿½ï¿½Ë³ï¿½ */
    if (exit_rows < 4)
    {
        lost_cnt++;
        if (lost_cnt >= 3)
        {
            ImageFlag.Zebra_Flag = 0;       /* ï¿½Ë³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ */
            lost_cnt = 0;
            return;
        }
    }
    else
    {
        lost_cnt = 0;                       /* ï¿½ï¿½ï¿½Ú°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú£ï¿½ï¿½ï¿½ï¿½ï¿½ */
    }

    /* ï¿½Ú¶ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ßµï¿½Í¼ï¿½ï¿½ï¿½Ğµã£¬ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½Æ¸ï¿½ï¿½Å±ß½ï¿½ï¿½ï¿½ */
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
        return;                              /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    int Ysite;
    int i = 0;                           /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */

    if (ImageStatus.WhiteLine >= 3) return;

    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if (ImageDeal[Ysite].Wide > 18
             && ImageDeal[Ysite].IsRightFind == 'T'
             && ImageDeal[Ysite].IsLeftFind == 'T'
             && ImageDeal[Ysite].LeftBorder < 40
             && ImageDeal[Ysite].RightBorder > 55   /* TC264: >55(Ô­>40) */
             && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center - 2] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center + 2] == 1
             && ImageStatus.Miss_Left_lines < 7
             && ImageStatus.Miss_Right_lines < 7)
            {
                i++;
            }
        }

        if (i >= 3)                           /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handle_Rampï¿½ï¿½ */
void Element_Handle_Ramp(void)
{
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */

}

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Judgment_OutRoadï¿½ï¿½ */























/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handle_OutRoadï¿½ï¿½ */














    /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */






/* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
#define CROSS_WHITE_LINE_MIN 8
#define CROSS_VALID_LINE_COUNT 3

/* ï¿½Ö±ï¿½ï¿½Ş¸ï¿½Ê®ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ß£ï¿½ï¿½ï¿½ï¿½âµ¥ï¿½ï¿½Ê¶ï¿½ï¿½ï¿½ì³£Ó°ï¿½ï¿½ï¿½ï¿½Ò»ï¿½à¡£ */
static void Repair_Cross_Border(uint8 is_left)
{
    int row;
    int near_row = -1;
    int far_row = -1;
    int near_border;
    int far_border = 0;
    int border;

    /* ¶ÏÂ·ÅĞ¶Ï(ÔİÎ´ÊµÏÖ) */
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

    /* ¶ÏÂ·´¦Àí(ÔİÎ´ÊµÏÖ) */
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
    /* Ê®×Ö°×ÏßĞŞ¸´: ÏßĞÔ²åÖµÌî³äÈ«°×¶ªÊ§ÇøÓò */
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

    /* ï¿½ï¿½ï¿½ßºï¿½Í³Ò»ï¿½Ş·ï¿½ï¿½ï¿½ï¿½Ø½ï¿½ï¿½ï¿½ï¿½ß£ï¿½ï¿½ï¿½ CPU0 ï¿½ï¿½ï¿½ï¿½ Errï¿½ï¿½ */
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
/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Scan_Elementï¿½ï¿½ */
void Scan_Element(void)
{
    /* update jump counts every frame */
    g_left_jump_count  = (uint8)Ring_Check_Border_Jump(1U, RING_JUMP_THRESHOLD, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW);
    g_right_jump_count = (uint8)Ring_Check_Border_Jump(2U, RING_JUMP_THRESHOLD, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW);

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    if (ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0)  /* ï¿½ï¿½ï¿½ï¿½Ô²ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½ */
    {

        Element_Judgment_Left_Rings();        /* ï¿½ï¿½ï¿½ï¿½Ô²ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½ */
        Element_Judgment_Right_Rings();       /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        Element_Judgment_Zebra();             /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        Element_Judgment_Bend();              /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        Element_Judgment_Ramp();              /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        Straight_long_judge();                /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    if (ImageFlag.Bend_Road)
    {


    }

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Element_Handleï¿½ï¿½ */
void Element_Handle(void)
{
    if (ImageFlag.image_element_rings == 1)

    if (ImageFlag.image_element_rings == 1)
        Element_Handle_Left_Rings();
    if (ImageFlag.image_element_rings == 2)
        Element_Handle_Right_Rings();
    else if (ImageFlag.Zebra_Flag != 0)
        Element_Handle_Zebra();
    else if (ImageFlag.Ramp != 0)
        Element_Handle_Ramp();
    else if (ImageStatus.WhiteLine >= CROSS_WHITE_LINE_MIN)
        Get_ExtensionLine();                  /* Ê®×ÖÂ·¿Ú: ÑÓÉìÏß²¹È«¶ªÊ§±ß½ç */
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
}
/* ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ï¿½ï¿½Flag_initï¿½ï¿½ */
void Flag_init(void)
{
    ImageFlag.Bend_Road              = 0;
    ImageFlag.Zebra_Flag             = 0;
    ImageFlag.Ramp                   = 0;
    ImageFlag.straight_xie           = 0;
    ImageFlag.straight_long          = 0;

}


//-------------------------------------------------------------------------------
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
// ï¿½ï¿½Â¼ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½Ê¶ï¿½ï¿½×´Ì¬ï¿½ï¿½ */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    }

    /* ï¿½ï¿½Ê¾Ô²ï¿½ï¿½ï¿½×¶Î±ï¿½Ö¾Î»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½+ï¿½×¶ï¿½ï¿½ï¿½Ğ´ï¿½ï¿½ï¿½ï¿½ï¿½Úµï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½Ğ»ï¿½ */
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

    /* Ö´ï¿½Ğµï¿½Ç°Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¡£ */
    /* ï¿½ï¿½ï¿½ï¿½ï¿½Ò²ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½Ç°Í¼ï¿½ï¿½Æ«ï¿½î£¬ï¿½ï¿½Ôªï¿½ï¿½×´Ì¬Í¬Ö¡Ë¢ï¿½Â¡ï¿½ */
    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


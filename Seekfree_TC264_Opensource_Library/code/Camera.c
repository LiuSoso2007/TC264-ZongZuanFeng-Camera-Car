/*
 * Camera.c --- MT9V03X ï¿½ï¿½ï¿½ï¿½Í·ï¿½ï¿½ï¿½ï¿½ + Í¼ï¿½ï¿½Ñ¹ï¿½ï¿½ + OTSUï¿½ï¿½Öµï¿½ï¿½ + IPS200ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾
 *
 * OTSUï¿½Ş¸ï¿½: ï¿½Æ³ï¿½ï¿½ï¿½Ç°ï¿½Ë³ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½256ï¿½ï¿½ + ï¿½ï¿½Öµï¿½Ş·ï¿½30~220 (ï¿½ï¿½Ö±ï¿½ï¿½È«ï¿½ï¿½)
 * Ö¡ï¿½ï¿½ï¿½Å»ï¿½: ï¿½ï¿½ï¿½ï¿½ Camera_ShowBinaryFast() ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ÖµÍ¼, SPIï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¡
 */
#include "Camera.h"
#include "Shared.h"
static uint16 s_ring_state_frames = 0U;      /* µ±Ç°½×¶ÎÒÑ¾­³ÖĞøµÄÍ¼ÏñÖ¡Êı */
static uint8 s_ring_confirm_count = 0U;      /* Á¬ĞøÊ¶±ğÈ·ÈÏÖ¡Êı */
static uint8 s_ring_feature_count = 0U;      /* Èë¿Ú»ò³ö¿ÚÌØÕ÷È·ÈÏÖ¡Êı */
static uint8 s_ring_stable_count = 0U;       /* ³ö»·ºóÎÈ¶¨Ö±µÀÖ¡Êı */
static uint8 s_ring_exit_loss_seen = 0U;     /* »·ÄÚÊÇ·ñ¼û¹ı³ö¿Ú²à¶ªÏß */
static int s_ring_entry_corner_row = -1;     /* ×î½üÒ»´ÎÈë¿Ú¹ÕµãĞĞ */
static int s_ring_entry_corner_col = -1;     /* ×î½üÒ»´ÎÈë¿Ú¹ÕµãÁĞ */
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
ImageDealDatatypedef ImageDeal[LCDH];        // Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ½á¹¹ (Ã¿ï¿½ï¿½Ò»ï¿½ï¿½)
ImageStatustypedef ImageStatus;              // Í¼ï¿½ï¿½×´Ì¬(OFFLine/ï¿½ï¿½ï¿½ßµï¿½)
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/*
 * Camera_CompressInit - ï¿½ï¿½ï¿½ï¿½ Image_Use ï¿½ï¿½ mt9v03x_image ï¿½ï¿½Ö¸ï¿½ï¿½Ó³ï¿½ï¿½
 */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/*
 * Camera_OTSU_GetThreshold - ????????
 * ?????????????? (??????????????????????§à????, ???break????????¦Ë??)
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½0~255, ï¿½ï¿½ï¿½ï¿½Ş·ï¿½OTSU_MIN~OTSU_MAX
 */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row) {
    uint32 hist[256] = {0};
    uint16 i, j;
    uint16 t;
    uint32 total = (uint32)col * row;
    uint64 totalSum = 0;
    uint32 w0 = 0;
    uint64 sum0 = 0;
    float maxVar = 0.0f;
    uint8 bestThr = 128;

    /* ï¿½ï¿½Ò»ï¿½ï¿½: Í³ï¿½Æ»Ò¶ï¿½Ö±ï¿½ï¿½Í¼ */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++)
            hist[*image[i][j]]++;

    /* ?????: ????????? */
    for (t = 0; t < 256; t++)
        totalSum += (uint64)t * hist[t];

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµ, Ñ°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ä·½ï¿½ï¿½ (ï¿½ï¿½ï¿½ï¿½Ç°ï¿½Ë³ï¿½, È«ï¿½ï¿½ï¿½ï¿½) */
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

    /* ï¿½ï¿½ï¿½Ä²ï¿½: ï¿½ï¿½Öµï¿½Ş·ï¿½, ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½/ï¿½ï¿½ï¿½Øµï¿½ï¿½ï¿½ï¿½ì³£ */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    return bestThr;
}

/*
 * Camera_GetBinaryImage - ï¿½Ò¶ï¿½Í¼ï¿½ï¿½Öµï¿½ï¿½
 */
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

/*
 * Camera_ShowDebug - È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ (ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½Ã¿3~5Ö¡ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½)
 * ï¿½ï¿½: Ô­Ê¼ï¿½Ò¶ï¿½ 188x120 / ï¿½ï¿½: OTSUï¿½ï¿½Öµ / ï¿½ï¿½: ï¿½ï¿½ÖµÍ¼ 94x60
 */
/*
 * Camera_DrawCenterLines - ï¿½ï¿½Ô­Ê¼Í¼ï¿½Í¶ï¿½ÖµÍ¼ï¿½ï¿½ÊµÊ±ï¿½ï¿½ï¿½ï¿½Ë«ï¿½ï¿½ï¿½ï¿½
 *   ï¿½ï¿½É«ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ (ImageSensorMid)  -- ï¿½Ì¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä²Î¿ï¿½ï¿½ï¿½
 *   ï¿½ï¿½É«ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ (ImageDeal[].Center) -- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ä»¯
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½Ï²ï¿½Ô­Ê¼ï¿½Ò¶ï¿½Í¼(188x120) + ï¿½Â²ï¿½ï¿½ï¿½ÖµÍ¼(94x60)
 */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);  /* ï¿½ï¿½ÖµÍ¼XÆ«ï¿½ï¿½ */

    /* ---- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½É«ï¿½ï¿½ï¿½ï¿½): Í¼ï¿½ñ´«¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
    /* Ô­Ê¼ï¿½Ò¶ï¿½Í¼ï¿½ï¿½ï¿½ï¿½: y=0~119, x=ImageSensorMid*2=94 */
    ips200_draw_line(94, 0, 94, 119, RGB565_RED);
    /* ï¿½ï¿½ÖµÍ¼ï¿½ï¿½ï¿½ï¿½: y=150~209, x=xo+ImageSensorMid */
    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    /* ---- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½É«ï¿½ï¿½ï¿½ï¿½): ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ImageDeal[row].Center ---- */
    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½OFFLineï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ§ï¿½ï¿½ï¿½İµï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½2ï¿½ï¿½ï¿½Ô¼ï¿½ï¿½ï¿½SPIï¿½ï¿½ï¿½ï¿½ */
    /* Á½¸ö¶Ëµã¶¼±ØĞëÎ»ÓÚOFFLineÒÔÉÏµÄÓĞĞ§ËÑÏßÇøÓò¡£ */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        /* Ô­Ê¼ï¿½Ò¶ï¿½Í¼ (188x120): CenterÖµ*2Ó³ï¿½ï¿½, ï¿½Ğºï¿½*2Ó³ï¿½ï¿½ */
        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        /* ï¿½ï¿½ÖµÍ¼ (94x60, Æ«ï¿½ï¿½xo,150): Ê¹ï¿½ï¿½Ô­Ê¼94x60ï¿½ï¿½ï¿½ï¿½ */
        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

void Camera_ShowDebug(void) {
    uint16 xo;
    /* Ô­Ê¼ï¿½Ò¶ï¿½Í¼ */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* OTSU ï¿½ï¿½Öµ */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* ï¿½ï¿½Öµï¿½ï¿½Í¼ï¿½ï¿½ */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* Í¼ï¿½ï¿½ */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    /* legend removed */
    Camera_ShowElementStatus();
    
    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
//  @brief          Get_BaseLine - ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×¼ï¿½ï¿½
//  Ëµï¿½ï¿½: ï¿½Óµï¿½59-57ï¿½ï¿½Ô¤É¨, ï¿½Óµï¿½56ï¿½Ğ¿ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½5ï¿½ï¿½(56->52)
//  ï¿½ï¿½Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(ImageSensorMid=47)ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  5ï¿½ï¿½È«É¨Ò»ï¿½ï¿½, È·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  ï¿½ï¿½ï¿½ï¿½ Pixle[][] ï¿½ï¿½ÖµÍ¼ï¿½ï¿½ (0=ï¿½ï¿½/ï¿½ï¿½ï¿½ï¿½, 1=ï¿½ï¿½/ï¿½ï¿½ï¿½ï¿½)
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
    int   Xsite;                                // ï¿½ï¿½É¨ï¿½ï¿½Î»ï¿½ï¿½
    int   row;                                  // ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½Ğºï¿½

    /* ---- ï¿½ï¿½1ï¿½ï¿½: ï¿½Óµï¿½56ï¿½Ğ¿ï¿½Ê¼É¨ (ï¿½ï¿½×¼ï¿½ï¿½) ---- */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // ï¿½Óµï¿½56ï¿½Ğ¿ï¿½Ê¼

    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½Ò°×µï¿½ï¿½Úµï¿½ï¿½ï¿½ï¿½ï¿½
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

    // ???????????????, ???????????
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

    // ??56???????????????????
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* ????¦Ä????'F', ?????'T' */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ---- ï¿½ï¿½2ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½55->52ï¿½ï¿½ ---- */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò±ß½ï¿½
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
                ImageDeal[row].IsRightFind = 'F';   // ï¿½Ò²ï¿½Î´ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½
                break;
            }
        }

        // ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß½ï¿½
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
                ImageDeal[row].IsLeftFind = 'F';    // ???¦Ä???????
                break;
            }
        }

        // ï¿½ï¿½ï¿½Â±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* ????¦Ä????'F', ?????'T' */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* ---- ??3??: 5???????? (????????) ---- */
    // TODO: ?????????????????§¹??§µ?????
}

//-------------------------------------------------------------------------------
//  @brief          Get_Border_And_SideType - ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  ??[L, H]??¦¶??, ????????????????????????
//  ï¿½ï¿½ï¿½ï¿½: 'T'=ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½, 'W'=ï¿½ï¿½ï¿½Ğ°ï¿½(ï¿½ï¿½ï¿½ï¿½), 'H'=ï¿½ï¿½ï¿½Ğºï¿½(ï¿½ï¿½Â·)
//  @parameter      p    ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
//  @parameter      type ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: 'L'=ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½, 'R'=ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  @parameter      L, H ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î§ï¿½ß½ï¿½
//  @parameter      Q    ???????????
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* ---- ï¿½ï¿½È«Ğ£ï¿½ï¿½: ï¿½ï¿½L/Hï¿½ï¿½ï¿½ï¿½ï¿½ÚºÏ·ï¿½ï¿½ï¿½Î§[0, LCDW-1] ---- */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½
    {
        for (i = H; i >= L; i--)
        {
            // ï¿½ï¿½(1)->ï¿½ï¿½(0)ï¿½ï¿½ï¿½ï¿½: ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // ??????????????
                Q->type  = 'T';                 // ???????????
                break;
            }
            else if (i == L)                    // É¨ï¿½èµ½ï¿½ï¿½ï¿½ï¿½Î´ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½
            {
                if (*(p + (L + H) / 2) != 0)    // ï¿½ï¿½ï¿½ï¿½ï¿½Ğµï¿½ï¿½ï¿½ï¿½Ç°ï¿½É«
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½É¨ï¿½ï¿½Î»ï¿½ï¿½
                    Q->type  = 'W';             // ï¿½ï¿½ï¿½Ğ°ï¿½(ï¿½ï¿½Ê§ï¿½ï¿½ï¿½ï¿½)
                }
                else                            // Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ½á¹¹ (Ã¿ï¿½ï¿½Ò»ï¿½ï¿½)
                {
                    Q->point = (L + H) / 2;     // Hï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğµï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½LeftBorder>RightBorder
                    Q->type  = 'H';             // ï¿½ï¿½ï¿½Ğºï¿½
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½
    {
        for (i = L; i <= H; i++)
        {
            // ï¿½ï¿½(1)->ï¿½ï¿½(0)ï¿½ï¿½ï¿½ï¿½: ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // ??????????????
                Q->type  = 'T';                 // ???????????
                break;
            }
            else if (i == H)                    // É¨ï¿½èµ½ï¿½ï¿½ï¿½ï¿½Î´ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½
            {
                if (*(p + (L + H) / 2) != 0)    // ï¿½ï¿½ï¿½ï¿½ï¿½Ğµï¿½ï¿½ï¿½ï¿½Ç°ï¿½É«
                {
                    Q->point = (L + H) / 2;     // ï¿½ï¿½É¨ï¿½ï¿½Î»ï¿½ï¿½
                    Q->type  = 'W';             // ï¿½ï¿½ï¿½Ğ°ï¿½(ï¿½ï¿½Ê§ï¿½ï¿½ï¿½ï¿½)
                }
                else                            // Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ½á¹¹ (Ã¿ï¿½ï¿½Ò»ï¿½ï¿½)
                {
                    Q->point = (L + H) / 2;     // Hï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğµï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½LeftBorder>RightBorder
                    Q->type  = 'H';             // ï¿½ï¿½ï¿½Ğºï¿½
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
//  @brief          Get_AllLine - ï¿½Ó»ï¿½×¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  ??Get_BaseLine(56->52)???, ??51?§á??????52?§ß???????????Úb0??
//  ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½Úµï¿½Ç°ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Ğ±ï¿½ï¿½ï¿½Î»ï¿½ï¿½+/-ImageScanIntervalï¿½ï¿½Î§ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  ï¿½ì³£ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ò´¥·ï¿½ï¿½ï¿½ï¿½ï¿½; ï¿½ï¿½ï¿½Ğ°×´ï¿½ï¿½ï¿½OFFLine
//  @parameter      void
//  @return         void
//  @note           ï¿½ï¿½ï¿½ï¿½ ImageDeal[52] (ï¿½ï¿½×¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½) ï¿½ï¿½ Pixle[][] (ï¿½ï¿½ÖµÍ¼ï¿½ï¿½)
//  @note           OFFLine: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í¬Ê±ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¶ï¿½Îªï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
    int   row;                                  // ï¿½ï¿½Ç°É¨ï¿½ï¿½ï¿½Ğºï¿½
    int   IntervalLow, IntervalHigh;            // ??????????????
    int   i;                                    // ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

    /* ---- ï¿½ï¿½Ê¼ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½ ---- */
    ImageStatus.OFFLine          = 2;           // ï¿½ï¿½ï¿½ï¿½ï¿½Ğºï¿½(ï¿½ï¿½Ê¼Îª2)
    ImageStatus.Miss_Left_lines  = 0;           // ??????????????
    ImageStatus.Miss_Right_lines = 0;           // ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê§ï¿½ï¿½ï¿½ï¿½
    ImageStatus.WhiteLine        = 0;           // ï¿½ï¿½É«ï¿½Ğ¼ï¿½ï¿½ï¿½(Ê®ï¿½ï¿½Ê¶ï¿½ï¿½)
    ImageStatus.WhiteLine_L      = 0;           // ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¼ï¿½ï¿½ï¿½
    ImageStatus.WhiteLine_R      = 0;           // ?????§Ş???
    ImageStatus.OFFLineBoundary  = 0;           // ï¿½ï¿½ï¿½ß±ß½ï¿½ï¿½Ğºï¿½
    ImageStatus.Det_True         = 0;           // ï¿½ï¿½Ğ§ï¿½ï¿½ï¿½ï¿½Ö¾

    /*
     * ï¿½ï¿½51ï¿½Ğ¿ï¿½Ê¼, ï¿½ï¿½52ï¿½ï¿½(ï¿½ï¿½×¼ï¿½ï¿½)Îªï¿½Î¿ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½
     * ï¿½ï¿½ï¿½Ğµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½0ï¿½Ğ»ò´¥·ï¿½OFFLineï¿½ï¿½ï¿½ï¿½
     */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // [0]=????????, [1]=????????
        PicTemp = Pixle[row];

        /* ============================================================
         * ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ò±ß½ï¿½ +/- ImageScanInterval ï¿½ï¿½Î§ï¿½ï¿½É¨ï¿½ï¿½
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // ï¿½Ş·ï¿½ï¿½ï¿½[0, 93]
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ============================================================
         * ???????: ??????????? +/- ImageScanInterval ??¦¶?????
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ============================================================
         * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í½ï¿½ï¿½Ğ±ï¿½ï¿½ß´ï¿½ï¿½ï¿½:
         * 'T'=ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½Îªï¿½Òµï¿½ï¿½Ä±ï¿½ï¿½ï¿½Î»ï¿½ï¿½
         * 'W'=È«ï¿½ï¿½: Ê¹ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Ğ±ï¿½ï¿½ï¿½Öµ (ï¿½ï¿½ï¿½ï¿½+1)
         * 'H'=È«ï¿½ï¿½: É¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã»ï¿½Ğ°×µï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
         * ============================================================ */
        if (JumpPoint[0].type == 'W')           // ??????§Ñ?(????)
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // ?????????????
            ImageStatus.Miss_Left_lines++;      // ????????+1
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ã¶ªÊ§ï¿½ï¿½ï¿½ï¿½
        }

        if (JumpPoint[1].type == 'W')           // ï¿½Ò²ï¿½ï¿½ï¿½ï¿½Ğ°ï¿½(ï¿½ï¿½ï¿½ï¿½)
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ò±ß½ï¿½
            ImageStatus.Miss_Right_lines++;     // ï¿½Ò²à¶ªÊ§ï¿½ï¿½ï¿½ï¿½+1
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ã¶ªÊ§ï¿½ï¿½ï¿½ï¿½
        }

        /* ---- ï¿½ï¿½Â¼ï¿½Òµï¿½×´Ì¬ ---- */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ---- ï¿½ï¿½ï¿½Ğ¼ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½Í¬Ê±Îªï¿½ï¿½) ---- */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // ï¿½Û¼Æ°ï¿½É«ï¿½Ğ¼ï¿½ï¿½ï¿½
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* ??????§Ş??? */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ---- ï¿½ï¿½ï¿½ã±¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /*
         * Hï¿½ï¿½ï¿½ï¿½ï¿½Ş¸ï¿½: È«ï¿½ï¿½ï¿½Ğ³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò±ï¿½
         * ?????????H?, ??????????????????
         */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ---- ??H??: ????????????????, ???????(???)??????? ---- */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)  // ??->??????: ???????????
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ---- ï¿½ï¿½Hï¿½ï¿½: ï¿½Óµï¿½Ç°ï¿½Ò±ß½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½, Ñ°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(ï¿½×µï¿½)ï¿½Ø¶ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)  // ï¿½ï¿½->ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½Òµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò±ß½ï¿½
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ---- ï¿½Ş¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---- */
            /* Hï¿½ï¿½ï¿½Ş¸ï¿½Ê§ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Ğ±ß½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä»ï¿½ï¿½Ôµï¿½Î³ï¿½Ö±ï¿½ï¿½ */
            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ============================================================
         * OFFLineï¿½ï¿½ï¿½ï¿½ï¿½Ğ¶ï¿½: ï¿½ï¿½ï¿½ï¿½Í¬Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê§ï¿½ï¿½ï¿½ï¿½3ï¿½Ğ¡ï¿½
         * ï¿½ï¿½ï¿½ï¿½: Ê®ï¿½ï¿½Ôªï¿½ï¿½Ë«ï¿½ï¿½È«ï¿½ï¿½(W)ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½OFFLine, ï¿½ï¿½Ö¤WhiteLineï¿½ï¿½ï¿½Û»ï¿½ï¿½ï¿½8ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¡ï¿½
         * ============================================================ */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))  /* Ë«ï¿½ï¿½È«ï¿½ï¿½=Ê®ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
        {
            ImageStatus.OFFLine = row;          // ï¿½ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½Ğºï¿½
            break;
        }

        /*
         * °²²ÆÍ¬Ô´±£»¤: Ô¶¾°¿í¶È¹ıÕ­»ò±ßÏßÌù±ßÊ±Í£Ö¹¼ÌĞøÏòÉÏ×·Ïß¡£
         * TC264Îª94ÁĞ, ÓÉ°²²Æ80ÁĞãĞÖµ(7/10/70)°´±ÈÀıÓ³ÉäÎª8/12/82¡£
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



/* ================================================================
 * Ôªï¿½ï¿½Ê¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ (TC264: 94ï¿½Ğ¿ï¿½, AnCaiÔ­ï¿½ï¿½x1.175ï¿½ï¿½Ó³ï¿½ï¿½)
 * ================================================================ */
const uint8 Half_Road_Wide[60] = {           /* ï¿½ï¿½ï¿½Â·ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½->Ô¶ï¿½ï¿½ï¿½İ¼ï¿½) */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* ????????? */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* ================================================================
 * Í¼ï¿½ï¿½Ôªï¿½Ø±ï¿½Ö¾
 * ================================================================ */
ImageFlagtypedef ImageFlag;                  /* Í¼ï¿½ï¿½Ôªï¿½Ø±ï¿½Ö¾ */

/* ================================================================
 * Helper: Straight_Judge - Ö±ï¿½ï¿½ï¿½Ğ±ï¿½
 * dir=1: ?????????, dir=2: ?????????
 * ï¿½ï¿½ï¿½Ø¾ï¿½ï¿½ï¿½ï¿½ï¿½S, S<1 ï¿½ï¿½ÎªÖ±ï¿½ï¿½
 * ================================================================ */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* ???? */
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
    case 2: /* ï¿½Ò±ß½ï¿½ */
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
 * ï¿½ï¿½Ö±ï¿½ï¿½ï¿½Ğ¶ï¿½ï¿½ë´¦ï¿½ï¿½
 * ================================================================ */
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

/* ================================================================
 * §Ò??????§Ø? (???????????)
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

/* ================================================================
 * ?????? (???????+????)
 * ================================================================ */
void Element_Judgment_Bend(void)
{
    /*
     * ??????????: ??????????????????????
     * Ô­ OFFLine < 3 ï¿½ß¼ï¿½ï¿½ï¿½ï¿½ï¿½: OFFLine ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í¬Ê±ï¿½ï¿½ï¿½ß²Å´ï¿½ï¿½ï¿½,
     * ????????????, OFFLine ????2, ????????????????
     * ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½Ë«ï¿½à¶ªï¿½ß¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¶ï¿½ ï¿½ï¿½ Ë«ï¿½à¶¼×·ï¿½ï¿½ï¿½ï¿½ï¿½Ã²ï¿½ï¿½Ç´ï¿½Ö±ï¿½ï¿½ï¿½ï¿½
     */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1)
        return;
    /* ponytailÖ±µÀÊØÎÀ: OFFLine<5Ê±ÈüµÀÍêÈ«¿É¼û£¬²»¿ÉÄÜÊÇÍäµÀ
       ·ÀÖ¹ÔëÉùµ¼ÖÂMiss¼ÆÊıÀÛ»ıÒı·¢ÎóÅĞ (°²²ÆÔ­Ê¼OFFLine>=14, TC264ÊÊÅä60ĞĞ->5) */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;  /* ?????????? = ?????, ????????? */

    /* ????: ???ÂR??(<59), ??????????, ???????? (???????????) */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;              /* ï¿½ï¿½ï¿½ï¿½ */
    }

    /* ????: ???ÂR??(>35), ??????????, ???????? (???????????) */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;              /* ï¿½ï¿½ï¿½ï¿½ */
    }
}

/* ================================================================
 * ???????: ???¡¤???????????
 * ================================================================ */
void Element_Handle_Bend(void)
{
    int row;                                  /* ÓÃint¶ø·ÇucharÒÔÖ§³Ö´ó·¶Î§Ñ­»· */

    /* ponytail¶µµ×ÊØÎÀ: OFFLine<5Ê±ÈüµÀÍêÈ«¿É¼û£¬Çå³ıÍäµÀ±êÖ¾²¢ÍË³ö
       ÓëElement_Judgment_BendµÄOFFLineÊØÎÀºôÓ¦£¬Ë«±£ÏÕ·ÀÖ¹Ö±µÀÎóÅĞÍäµÀ */
    if (ImageStatus.OFFLine < 5)
        { ImageFlag.Bend_Road = 0; return; }

    /* Ë«²à¶¼×·×ÙÁ¼ºÃ -> ÒÑ»Ö¸´Ö±µÀ, Çå³ıÍäµÀ±êÖ¾ */
    if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4)
        { ImageFlag.Bend_Road = 0; return; }

    if (ImageFlag.Bend_Road == 1)             /* ×óÍä: Ö»¿¿ÓÒ±ß½ç¿É¼û, center=ÓÒ±ß½ç-°ë¿í(Ïò×óÆ«ÒÆ) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* é™å¹… >= 0 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* ÓÒÍä: Ö»¿¿×ó±ß½ç¿É¼û, center=×ó±ß½ç+°ë¿í(ÏòÓÒÆ«ÒÆ) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* é™å¹… <= 93 */
        }
    }
}

/* ================================================================
 * ï¿½ï¿½Ô²ï¿½ï¿½Ê¶ï¿½ï¿½
 * ================================================================ */
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

/* ´Ó½ü¶ËÏòÔ¶¶ËÑ°ÕÒ±¾²à±ß½çÍ»±ä£¬·µ»ØÈë¿Ú¹ÕµãËùÔÚĞĞ¡£ */
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

/* ³ö¿Ú²à±ØĞëÏÈ¶ªÊ§ÔÙ»Ö¸´£¬±ÜÃâ»·ÄÚ¶ÌÔİË«±ß¿É¼ûÊ±ÌáÇ°³ö»·¡£ */
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

/* ×óÓÒÔ²»·¹²ÓÃ¾µÏñ²¹Ïß£¬¹Ì¶¨Æ«ÒÆÈ·±£¶æ»úÎó²îÔ½¹ıÏÖÓĞËÀÇø¡£ */
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

    /* °²²ÆÍ¬Ô´ÃÅ¼÷£º×óÔ²»·±ØĞëÏÈ³öÏÖ×ó²àÁ¬Ğø¶ªÏß£¬Ö±µÀÔëÉù²»µÃ´¥·¢¡£ */
    if (ImageStatus.Miss_Right_lines > 3
        || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 2 || Straight_Judge(2, 5, SCAN_BASE_END_ROW) > 3.0f   /* ???????????: ????????????? */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* Ìõ¼ş²»×ãÊ±½ûÖ¹½øÈëÔ²»·²¹Ïß£¬·ÀÖ¹¸²¸ÇÖ±µÀÖĞĞÄ¡£ */

    /* ï¿½ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğµï¿½ï¿½ï¿½ï¿½ï¿½ß¶ï¿½ï¿½ï¿½'W'(È«ï¿½ï¿½) */
    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: ï¿½ï¿½ï¿½ï¿½TC264ï¿½Ğ·ï¿½Î§48->44 */
        {
            if (ImageDeal[r].IsLeftFind == 'W') return;
        }
    }

    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 1].LeftBorder) > 4  /* abs: ??????????????? */)
        {
            Left_Less_Num++;
            /* ???????????????? */
            if (Left_Less_Num == 1) {
                /* ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±, ï¿½ï¿½ï¿½Ú´Ë¼ï¿½Â¼ï¿½ï¿½Ê¼ï¿½Ğºï¿½ */
            }
        }
    }

    if (Left_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 1;    /* ï¿½ï¿½Ô²ï¿½ï¿½ */
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* ================================================================
 * ï¿½ï¿½Ô²ï¿½ï¿½Ê¶ï¿½ï¿½ (ï¿½ß¼ï¿½ï¿½Ô³ï¿½)
 * ================================================================ */
void Element_Judgment_Right_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Right_Less_Num = 0;

    /* °²²ÆÍ¬Ô´ÃÅ¼÷£ºÓÒÔ²»·±ØĞëÏÈ³öÏÖÓÒ²àÁ¬Ğø¶ªÏß£¬Ö±µÀÔëÉù²»µÃ´¥·¢¡£ */
    if (ImageStatus.Miss_Left_lines > 3
        || ImageStatus.Miss_Right_lines < 15
        || ImageStatus.OFFLine > 2 || Straight_Judge(1, 5, SCAN_BASE_END_ROW) > 3.0f   /* ???????????: ????????????? */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* Ìõ¼ş²»×ãÊ±½ûÖ¹½øÈëÔ²»·²¹Ïß£¬·ÀÖ¹¸²¸ÇÖ±µÀÖĞĞÄ¡£ */

    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: ï¿½ï¿½ï¿½ï¿½TC264ï¿½Ğ·ï¿½Î§ */
        {
            if (ImageDeal[r].IsRightFind == 'W') return;
        }
    }

    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].RightBorder - ImageDeal[Ysite - 1].RightBorder) > 4  /* abs: ??????????????? */)
        {
            Right_Less_Num++;
        }
    }

    if (Right_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 2;    /* ï¿½ï¿½Ô²ï¿½ï¿½ */
        Ring_Set_State(RING_STATE_CONFIRM);
    }
}

/* ================================================================
 * ï¿½ï¿½Ô²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½Ãµï¿½Â·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * ================================================================ */
void Element_Handle_Left_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 1)
    {
        Ring_Rebuild_Center(1U);
    }
}

/* ================================================================
 * ????????
 * ================================================================ */
void Element_Handle_Right_Rings(void)
{
    Ring_State_Update();
    if (ImageFlag.image_element_rings == 2)
    {
        Ring_Rebuild_Center(2U);
    }
}

/* ================================================================
 * ?????????: ????20~32??¦¶???????????
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

    if (NUM > 8)                              /* ?????????: ?§Ø???????? */
    {
        if (ImageDeal[SCAN_BASE_START_ROW].Center > 47)  /* TC264: ????94????¦Ë47, ???????=?????? */
            ImageFlag.Zebra_Flag = 2;           /* ?????, ?????? */
        else                                  /* ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½=ï¿½Ò²à±»ï¿½Úµï¿½ */
            ImageFlag.Zebra_Flag = 1;           /* ?????, ?????? */
    }
}

/* ================================================================
 * ï¿½ï¿½ï¿½ï¿½ï¿½ß´ï¿½ï¿½ï¿½: ï¿½Ãµï¿½Â·ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * ================================================================ */
void Element_Handle_Zebra(void)
{
    int row;

    if (ImageFlag.Zebra_Flag == 1)            /* ?????: ??????, ??????????????? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Road_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }
    else if (ImageFlag.Zebra_Flag == 2)       /* ?????: ??????, ??????????????? */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Road_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }
}

/* ================================================================
 * ??????: OFFLine??? + ??? + ??????
 * ================================================================ */
void Element_Judgment_Ramp(void)
{
        return;                              /* ï¿½İ²ï¿½ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å»ï¿½ */
    int Ysite;
    int i = 0;                           /* ï¿½ï¿½Ğ§ï¿½Ğ¼ï¿½ï¿½ï¿½ï¿½ï¿½ */

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

        if (i >= 3)                           /* ï¿½Û¼ï¿½3ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ================================================================
 * Í¼ï¿½ï¿½Ôªï¿½Ø±ï¿½Ö¾
 * ================================================================ */
void Element_Handle_Ramp(void)
{
    /* Rampï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î´Êµï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */

}

/* ================================================================
 * ??¡¤???: OFFLine??? + ????????????????
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
 * ??¡¤????: ??????????????
 * ================================================================ */
void Element_Handle_OutRoad(void)
{
    int Ysite, Xsite;
    int gray_sum = 0;

    /* ???????????(????)????? */
    for (Ysite = 35; Ysite < 55; Ysite++)
    {
        for (Xsite = 30; Xsite < 64; Xsite++) /* TC264Í¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
        {
            gray_sum += Pixle[Ysite][Xsite];
        }
    }

    /* ï¿½×µï¿½ï¿½ã¹»ï¿½ï¿½ -> ï¿½Ë³ï¿½ï¿½ï¿½Â·×´Ì¬ */
    if (gray_sum > 400 && ImageStatus.OFFLine < 20)
    {
        ImageFlag.Out_Road = 0;
    }
}

/* ================================================================
 * ??????: ????????, ????????§¹?????????
 * ================================================================ */
#define CROSS_WHITE_LINE_MIN 8
#define CROSS_VALID_LINE_COUNT 3

/* ·Ö±ğĞŞ¸´Ê®×ÖÇøÓòµÄÒ»²à±ßÏß£¬±ÜÃâµ¥²àÊ¶±ğÒì³£Ó°ÏìÁíÒ»²à¡£ */
static void Repair_Cross_Border(uint8 is_left)
{
    int row;
    int near_row = -1;
    int far_row = -1;
    int near_border;
    int far_border = 0;
    int border;

    /* µÚÒ»¶Î°×ĞĞµÄÏÂÒ»½ü³¡ĞĞ×÷Îª²¹ÏßÆğµã¡£ */
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

    /* Ê®×ÖÔ¶¶Ë±ØĞëÁ¬ĞøÈıĞĞ±ßÏßÓĞĞ§£¬·ÀÖ¹Ôëµã±»µ±×÷²¹ÏßÃªµã¡£ */
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
        /* Î´ÕÒµ½¿É¿¿Ô¶ÃªµãÊ±±£³ÖÈë¿Ú±ß½ç£¬³µÁ¾¼ÌĞø°´Èë¿Ú·½ÏòÖ±ĞĞ¡£ */
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

    /* ²¹ÏßºóÍ³Ò»ÏŞ·ù²¢ÖØ½¨ÖĞÏß£¬¹© CPU0 ¼ÆËã Err¡£ */
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
/* ================================================================
 * ?????????: ?????????????
 * ???: ????????????????????
 * ================================================================ */
void Scan_Element(void)
{
    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôªï¿½ï¿½×´Ì¬ï¿½Â½ï¿½ï¿½ï¿½Ôªï¿½ï¿½Ê¶ï¿½ï¿½ */
    if (ImageFlag.Out_Road == 0 && ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0)  /* ???/???????????????? */
    {
        Element_Judgment_OutRoad();           /* ï¿½ï¿½Â· */
        Element_Judgment_Left_Rings();        /* ï¿½ï¿½Ô²ï¿½ï¿½ */
        Element_Judgment_Right_Rings();       /* ï¿½ï¿½Ô²ï¿½ï¿½ */
        Element_Judgment_Zebra();             /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
        Element_Judgment_Bend();              /* ??? */
        Element_Judgment_Ramp();              /* ï¿½Âµï¿½ */
        Straight_long_judge();                /* ï¿½ï¿½Ö±ï¿½ï¿½ */
    }

    /* ????????????¡¤ */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();
        if (ImageFlag.Out_Road) ImageFlag.Bend_Road = 0;
    }

    /* ???????????????? */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ================================================================
 * ?????????: ???????????????????????
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
    else if (ImageStatus.WhiteLine >= CROSS_WHITE_LINE_MIN)
        Get_ExtensionLine();                  /* Ê®×ÖÓÅÏÈÓÚÆÕÍ¨Ö±µÀºÍÍäµÀ²¹Ïß¡£ */
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
}
/* ================================================================
 * ï¿½ï¿½Ô²ï¿½ï¿½Ê¶ï¿½ï¿½
 * ================================================================ */
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
//  @brief          Camera_ShowElementStatus - ï¿½ï¿½Ê¾ï¿½ï¿½Ç°Ôªï¿½ï¿½×´Ì¬
//  ï¿½ï¿½IPS200ï¿½×²ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½Ç°Ê¶ï¿½ğµ½µï¿½ï¿½ï¿½ï¿½ï¿½Ôªï¿½ï¿½(ï¿½ï¿½Ğ´ï¿½ï¿½Ê¶)
//  ??§Õ: zhi=??? wan_L/R=??? shi=??? huan_L/R=??? banma=???? po=??? duan=??¡¤
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* ï¿½×²ï¿½×´Ì¬ï¿½ï¿½: ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /*
     * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½Ç°Ôªï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½È¼ï¿½ï¿½ï¿½ï¿½Ïµï¿½ï¿½ï¿½
     * Î»ï¿½ï¿½: y=225 (ï¿½ï¿½Ä»240ï¿½ï¿½, ï¿½×²ï¿½15pxï¿½Ğ¸ï¿½)
     */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* ï¿½ï¿½Ô²ï¿½ï¿½ */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* ï¿½ï¿½Ô²ï¿½ï¿½ */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* Ê®ï¿½ï¿½ */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* ï¿½ï¿½Ôªï¿½ï¿½ */
    }

    /* ??????§Ş??? */
    /* µ×À¸ÓÒ²àÏÔÊ¾µ±Ç°Í¼ÏñÆ«²î£¬ÓëÔªËØ×´Ì¬Í¬Ö¡Ë¢ĞÂ¡£ */
    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


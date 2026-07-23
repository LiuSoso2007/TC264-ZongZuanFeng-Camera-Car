/* ����ͷͼ����ģ��˵�� */
#include "Camera.h"
#include "Shared.h"
static uint16 s_ring_state_frames = 0U;      /* ��ǰ�׶��Ѿ�������ͼ��֡�� */
static uint8 s_ring_confirm_count = 0U;      /* ����ʶ��ȷ��֡�� */
static uint8 s_ring_feature_count = 0U;      /* ��ڻ��������ȷ��֡�� */
static uint8 s_ring_stable_count = 0U;       /* �������ȶ�ֱ��֡�� */
static uint8 s_ring_exit_loss_seen = 0U;     /* �����Ƿ�������ڲඪ�� */
static int s_ring_entry_corner_row = -1;     /* ���һ����ڹյ��� */
static int s_ring_entry_corner_col = -1;     /* ????????? */
static uint8 s_ring_edge_squeezed = 0U;      /* row50 edge squeezed */
static uint8 s_ring_edge_released = 0U;      /* row50 edge released */
static int s_ring_prev_valley_row = -1;     /* 上一帧谷底行号(APPROACH用) */
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
int16_t g_ZebraSum = 0;                 /* �����߼���ֵ֮�� */
ImageDealDatatypedef ImageDeal[LCDH];        // ���µ�ǰɨ��������
ImageStatustypedef ImageStatus;              // ����ͼ��ʶ��״̬
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/* ����˵����Camera_CompressInit�� */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/* ����˵����Camera_OTSU_GetThreshold����ֱ��ͼ�����OTSU��򷨡� */
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

    /* ��һ��ɨ��ȫͼ�һҶ���С���ֵ�� */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++) {
            uint8 v = *image[i][j];
            if (v < pmin) pmin = v;
            if (v > pmax) pmax = v;
        }

    range = pmax - pmin;

    /* ͳ�������ֱ��ͼ���Աȶ���ʱ���쵽[0,255]����OTSU�� */
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

    /* OTSU��䷽���������ѷָ���ֵ�� */
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

    /* �������ֵӳ���ԭʼ�Ҷȷ�Χ����ֵ��ʹ�á� */
    if (range > 30) {
        bestThr = (uint8)(pmin + ((uint16)bestThr * range) / 255U);
    }

    /* ǯλ����ȫ��Χ�� */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    bestThr += OTSU_BIAS;
    return bestThr;
}

/* ����˵����Camera_GetBinaryImage�� */
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

/* ִ�е�ǰͼ�������衣 */
/* ����˵����Camera_DrawCenterLines�� */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);  /* ִ�е�ǰͼ�������衣 */

    /* ִ�е�ǰͼ�������衣 */
    /* ִ�е�ǰͼ�������衣 */
    ips200_draw_line(94, 0, 94, 119, RGB565_RED);
    /* ִ�е�ǰͼ�������衣 */
    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    /* ����ͼ��ʶ��״̬�� */
    /* ����ͼ��ʶ��״̬�� */
    /* �����˵㶼����λ��OFFLine���ϵ���Ч�������� */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        /* ������ǰɨ���еı������ݡ� */
        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        /* ������ǰɨ���еı������ݡ� */
        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

void Camera_ShowDebug(void) {
    uint16 xo;
    /* ִ�е�ǰͼ�������衣 */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* ִ�е�ǰͼ�������衣 */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* ������ǰɨ���еı������ݡ� */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* ִ�е�ǰͼ�������衣 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    /* legend removed */
    Camera_ShowElementStatus();
    
    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // ��¼��ǰ��������
    int   Xsite;                                // ��¼��ǰ��������
    int   row;                                  // ��¼��ǰ��������

    /* ������ǰɨ���еı������ݡ� */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // ���µ�ǰɨ��������

    // ��¼��ǰ��������
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

    // ��¼��ǰ��������
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

    // ��¼��ǰ��������
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* ������ǰɨ���еı������ݡ� */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ������ǰɨ���еı������ݡ� */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // ��¼��ǰ��������
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
                ImageDeal[row].IsRightFind = 'F';   // ���µ�ǰɨ��������
                break;
            }
        }

        // ��¼��ǰ��������
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
                ImageDeal[row].IsLeftFind = 'F';    // ���µ�ǰɨ��������
                break;
            }
        }

        // ��¼��ǰ��������
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* ������ǰɨ���еı������ݡ� */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* ִ�е�ǰͼ�������衣 */
    // ��¼��ǰ��������
}

//-------------------------------------------------------------------------------
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* ִ�е�ǰͼ�������衣 */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // ��¼��ǰ��������
    {
        for (i = H; i >= L; i--)
        {
            // ��¼��ǰ��������
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // ��¼��ǰ��������
                Q->type  = 'T';                 // ��¼��ǰ��������
                break;
            }
            else if (i == L)                    // ��¼��ǰ��������
            {
                if (*(p + (L + H) / 2) != 0)    // ��¼��ǰ��������
                {
                    Q->point = (L + H) / 2;     // ��¼��ǰ��������
                    Q->type  = 'W';             // ��¼��ǰ��������
                }
                else                            // ��¼��ǰ��������
                {
                    Q->point = (L + H) / 2;     // ��¼��ǰ��������
                    Q->type  = 'H';             // ��¼��ǰ��������
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // ��¼��ǰ��������
    {
        for (i = L; i <= H; i++)
        {
            // ��¼��ǰ��������
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // ��¼��ǰ��������
                Q->type  = 'T';                 // ��¼��ǰ��������
                break;
            }
            else if (i == H)                    // ��¼��ǰ��������
            {
                if (*(p + (L + H) / 2) != 0)    // ��¼��ǰ��������
                {
                    Q->point = (L + H) / 2;     // ��¼��ǰ��������
                    Q->type  = 'W';             // ��¼��ǰ��������
                }
                else                            // ��¼��ǰ��������
                {
                    Q->point = (L + H) / 2;     // ��¼��ǰ��������
                    Q->type  = 'H';             // ��¼��ǰ��������
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
//  @parameter      void
//  @return         void
// ��¼��ǰ��������
// ��¼��ǰ��������
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // ��¼��ǰ��������
    int   row;                                  // ��¼��ǰ��������
    int   IntervalLow, IntervalHigh;            // ��¼��ǰ��������
    int   i;                                    // ��¼��ǰ��������

    /* ����ͼ��ʶ��״̬�� */
    ImageStatus.OFFLine          = 2;           // ����ͼ��ʶ��״̬
    ImageStatus.Miss_Left_lines  = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.Miss_Right_lines = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.WhiteLine        = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.WhiteLine_L      = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.WhiteLine_R      = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.OFFLineBoundary  = 0;           // ����ͼ��ʶ��״̬
    ImageStatus.Det_True         = 0;           // ����ͼ��ʶ��״̬

    /* ����ͼ��ʶ��״̬�� */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // ��¼��ǰ��������
        PicTemp = Pixle[row];

        /* ������ǰɨ���еı������ݡ� */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // ��¼��ǰ��������
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ������ǰɨ���еı������ݡ� */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ������ǰɨ���еı������ݡ� */
        if (JumpPoint[0].type == 'W')           // ��¼��ǰ��������
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // ���µ�ǰɨ��������
            ImageStatus.Miss_Left_lines++;      // ����ͼ��ʶ��״̬
        }
        else                                    // ��¼��ǰ��������
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // ����ͼ��ʶ��״̬
        }

        if (JumpPoint[1].type == 'W')           // ��¼��ǰ��������
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // ���µ�ǰɨ��������
            ImageStatus.Miss_Right_lines++;     // ����ͼ��ʶ��״̬
        }
        else                                    // ��¼��ǰ��������
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // ����ͼ��ʶ��״̬
        }

        /* ������ǰɨ���еı������ݡ� */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ����ͼ��ʶ��״̬�� */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // ����ͼ��ʶ��״̬
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* ����ͼ��ʶ��״̬�� */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ������ǰɨ���еı������ݡ� */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /* ������ǰɨ���еı������ݡ� */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ������ǰɨ���еı������ݡ� */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)  // ��¼��ǰ��������
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ������ǰɨ���еı������ݡ� */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)  // ��¼��ǰ��������
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ������ǰɨ���еı������ݡ� */
            /* ������ǰɨ���еı������ݡ� */
            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ����ͼ��ʶ��״̬�� */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))  /* ����ͼ��ʶ��״̬�� */
        {
            ImageStatus.OFFLine = row;          // ����ͼ��ʶ��״̬
            break;
        }

        /*
         * ����ͬԴ����: Զ�����ȹ�խ���������ʱֹͣ��������׷�ߡ�
         * TC264Ϊ94��, �ɰ���80����ֵ(7/10/70)������ӳ��Ϊ8/12/82��
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



/* ִ�е�ǰͼ�������衣 */
const uint8 Half_Road_Wide[60] = {           /* ִ�е�ǰͼ�������衣 */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* ִ�е�ǰͼ�������衣 */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* ִ�е�ǰͼ�������衣 */
ImageFlagtypedef ImageFlag;                  /* ִ�е�ǰͼ�������衣 */

/* ����˵����Straight_Judge�� */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* ������ǰɨ���еı������ݡ� */
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
    case 2: /* ������ǰɨ���еı������ݡ� */
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

/* ����˵����Straight_long_judge�� */
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

/* ����˵����Straight_xie_judge�� */
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

/* ����˵����Element_Judgment_Bend�� */
void Element_Judgment_Bend(void)
{
    /* ִ�е�ǰͼ�������衣 */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag)
        return;
    /* ponytailֱ������: OFFLine<5ʱ������ȫ�ɼ��������������
       ��ֹ��������Miss�����ۻ��������� (����ԭʼOFFLine>=14, TC264����60��->5) */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;  /* ����ͼ��ʶ��״̬�� */

    /* ����ͼ��ʶ��״̬�� */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;              /* ִ�е�ǰͼ�������衣 */
    }

    /* ����ͼ��ʶ��״̬�� */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;              /* ִ�е�ǰͼ�������衣 */
    }
}

/* ����˵����Element_Handle_Bend�� */
void Element_Handle_Bend(void)
{
    int row;                                  /* ��int����uchar��֧�ִ�Χѭ�� */

    /* ponytail��������: OFFLine<5ʱ������ȫ�ɼ�����������־���˳�
       ��Element_Judgment_Bend��OFFLine������Ӧ��˫���շ�ֱֹ��������� */
    if (ImageStatus.OFFLine < 5)
        { ImageFlag.Bend_Road = 0; return; }

    /* ˫�඼׷������ -> �ѻָ�ֱ��, ��������־ */
    if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4)
        { ImageFlag.Bend_Road = 0; return; }

    if (ImageFlag.Bend_Road == 1)             /* ����: ֻ���ұ߽�ɼ�, center=�ұ߽�-���(����ƫ��) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* 限幅 >= 0 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* ����: ֻ����߽�ɼ�, center=��߽�+���(����ƫ��) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* 限幅 <= 93 */
        }
    }
}

/* ִ�е�ǰͼ�������衣 */
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
 ** �ڶ���ⷨ ���� Բ��ʶ����ĺ�����
 ** ���� hao-yue-1/SmartCar (�㶫��ҵ��ѧ�������) ���Ӿ�����
 ** ���� TC264 + MT9V03X + 94x60ѹ��ͼ
 ** ��IMU���޵�� ���� ����ֵͼ���ط���
 *******************************************************************************************
 */

/* ---- �ڶ��ײ���⣺���ͼ��ײ������Ƿ���ں�ɫ���� ---- */
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


/* ---- ��֤�յ��Ϸ��Ƿ���ںڶ� ���� ������ͨ�����Բ�� ---- */
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

/* ---- �ȵ�׷�٣��Ӱ׺������������/����׷�ٵ��ȵ� ---- */
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

/* ---- �ȶ�ֱ����⣺˫�߲����� + б������ ---- */
static uint8 Ring_Is_Stable_Road(void)
{
    return (uint8)(ImageStatus.OFFLine <= 2
                && ImageStatus.Miss_Left_lines < 4
                && ImageStatus.Miss_Right_lines < 4
                && Straight_Judge(1, 5, SCAN_BASE_END_ROW) < 2.0f
                && Straight_Judge(2, 5, SCAN_BASE_END_ROW) < 2.0f);
}

/* ---- ��ѡ��⣺�ڶ��ײ� + �������� ---- */
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

/* ---- Ѱ����ڹȵ׵㣺�ùȵ�׷��������������� ---- */
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

/* ---- ENTRY阶段新方案: 从对侧边缘横向扫找入口拐点 ----
 * 从下往上扫每行, 从对侧边缘出发向环岛方向扫,
 * 找黑色区域的远侧边跳变点作为拐点, 记录跳变点与起点的横向距离.
 * 相邻两行距离差绝对值>10时, 取靠上(行数小)那行的跳变点作为入口拐点.
 * direction=1(左圆环): 从右边缘向左扫, 找黑色区域左边缘(黑->白), 左边的跳变点作为拐点
 * direction=2(右圆环): 从左边缘向右扫, 找黑色区域右边缘(黑->白), 右边的跳变点作为拐点
 * 返回: 1=找到拐点, 0=未找到; 拐点坐标通过 corner_row/corner_col 输出
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
            /* 右圆环: 从左边缘(col=0)向右扫, 先遇白->黑(环岛左缘), 再遇黑->白(环岛右缘=右边跳变点) */
            jump_col = LCDW - 1;
            in_black = 0U;
            for (col = 0; col < LCDW - 1; col++)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col + 1] == IMG_WHITE)
                    {
                        jump_col = col;       /* 黑色区域右边缘: 黑->白 的黑点位置 */
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
            curr_dist = jump_col;         /* 从左边缘到跳变点的距离 = 列数 */
        }
        else
        {
            /* 左圆环: 从右边缘(col=LCDW-1)向左扫, 先遇白->黑(环岛右缘), 再遇黑->白(环岛左缘=左边跳变点) */
            jump_col = 0;
            in_black = 0U;
            for (col = LCDW - 1; col > 0; col--)
            {
                if (in_black)
                {
                    if (Pixle[row][col] == IMG_BLACK && Pixle[row][col - 1] == IMG_WHITE)
                    {
                        jump_col = col;       /* 黑色区域左边缘: 黑->白 的黑点位置 */
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
            curr_dist = (LCDW - 1) - jump_col;  /* 从右边缘到跳变点的距离 */
        }

        /* 与下一行(更靠下, row+1)的距离比较, 差值>10则当前行(靠上)为拐点 */
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

    /* Scheme 1: row 50 at edge → squeeze→release→bounce in one scan */
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
                    /* 谷点向上5行有3行是丢线则跳过, 继续往上找 */
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

    /* Scheme 2: row 50 released but was squeezed before → just find bounce */
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
                    /* 谷点向上5行有3行是丢线则跳过, 继续往上找 */
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

/* ---- ����������⣺���ڲ���߻ָ� + �ڶ���֤ ---- */
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

/* ---- ����ƫ���� ---- */
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
        /* 补线: 出发点与APPROACH相反, 从对侧下角补到入圈拐点 */
        if (s_ring_entry_corner_row > 0)
        {
            if (direction == 1U) /* 左圆环: 右下角 -> 拐点, 作为右边线 */
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, LCDW - 1,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'R');
            else                 /* 右圆环: 左下角 -> 拐点, 作为左边线 */
                Ring_DrawAndUpdate(direction, SCAN_BASE_START_ROW, 0,
                                   s_ring_entry_corner_row, s_ring_entry_corner_col, 'L');
        }
        else
        {
            /* 无拐点兜底: 贴对侧边 */
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

/* ---- ״̬������ ---- */
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
            /* 进入ENTRY条件: 谷点行>40 或 连续两帧谷底行数差绝对值>20 */
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
        /* 调用新方案: 从对侧边缘横向扫找入圈拐点 */
        if (Ring_Find_Entry_Corner(direction, &valley_row, &valley_col))
        {
            s_ring_entry_corner_row = valley_row;
            s_ring_entry_corner_col = valley_col;
            /* 行数>48 或 列数太靠边(右圆环col<15 / 左圆环col>LCDW-15): 进入INSIDE */
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
            /* 找不到拐点: 进入INSIDE */
            Ring_Set_State(RING_STATE_INSIDE);
            break;
        }
        /* 超时保护 */
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

/* ---- ��Բ���жϣ��ڶ���ⴥ�� ---- */
/* ---- 跳变少的那边靠边行数检查 ----
 * 检查行范围 15~45, 边缘margin=8像素, 靠边行>5行则返回1
 * direction=1: 检查右边界是否太多行挤到右边缘
 * direction=2: 检查左边界是否太多行挤到左边缘
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
    /* 靠边行数 > 4 则返回1 */
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

/* ---- ��Բ���жϣ��ڶ���ⴥ�� ---- */
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
    int trans_count;        /* ��ǰ�к�->��������� */
    int valid_rows = 0;     /* ��Ч����(����>=5����) */
    static int confirm_cnt = 0;     /* ����ȷ��֡����(���ڷ���) */

    /* ����/�����Ԫ�ؼ�����Ѵ��ڰ�����״̬ʱ����� */
    if (ImageFlag.image_element_rings
     || ImageFlag.Zebra_Flag != 0)
        return;

    /* �̶����봰��ɨ����44~57����ͳ�ƺ�->������(0->1)��
     * ��������ֱ�����У�·�����30~52px����60px�����ڡ�
     * �������߽��⣬��Ȼ����ʮ��·�ں�������������� */
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

    /* ��Ч��>=5ʱ���ư����ߣ�������2֡ȷ�Ϸ����� */
    if (valid_rows >= 5)
    {
        confirm_cnt++;
        if (confirm_cnt >= 2)
        {
            ImageFlag.Zebra_Flag = 1;       /* ȷ�ϰ����� */
        }
    }
    else
    {
        confirm_cnt = 0;                     /* δ����ֵ������ȷ�ϼ��� */
    }
}











/* ����˵����Element_Handle_Zebra */
/* ����˵����Element_Handle_Zebra�������ߴ�������ֱ�������ߣ���������Ϊͼ���е�ֱ�У�������˳������� */
/* ����˵����Element_Handle_Zebra */
/* ����˵����Element_Handle_Zebra�������ߴ�������ֱ�������ߣ���������Ϊͼ���е�ֱ�У�������˳������� */
void Element_Handle_Zebra(void)
{
    int row, Ysite, Xsite;
    int trans_count;
    int exit_rows = 0;
    static int lost_cnt = 0;        /* ����δ��⵽������֡����(�����˳�) */

    /* ��һ�����̶����봰����ɨ���䣬���������Ƿ�����ʧ */
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

    /* ������������ʧʱ�ۼ�֡��������3֡ȷ���˳� */
    if (exit_rows < 4)
    {
        lost_cnt++;
        if (lost_cnt >= 3)
        {
            ImageFlag.Zebra_Flag = 0;       /* �˳�������״̬ */
            lost_cnt = 0;
            return;
        }
    }
    else
    {
        lost_cnt = 0;                       /* ���ڰ������ڣ����� */
    }

    /* �ڶ�����ֱ�������ߡ����������ߵ�ͼ���е㣬��ֹ���Ƹ��ű߽��� */
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
        return;                              /* ִ�е�ǰͼ�������衣 */
    int Ysite;
    int i = 0;                           /* ����ͼ��ʶ��״̬�� */

    if (ImageStatus.WhiteLine >= 3) return;

    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if (ImageDeal[Ysite].Wide > 18
             && ImageDeal[Ysite].IsRightFind == 'T'
             && ImageDeal[Ysite].IsLeftFind == 'T'
             && ImageDeal[Ysite].LeftBorder < 40
             && ImageDeal[Ysite].RightBorder > 55   /* TC264: >55(ԭ>40) */
             && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center - 2] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center + 2] == 1
             && ImageStatus.Miss_Left_lines < 7
             && ImageStatus.Miss_Right_lines < 7)
            {
                i++;
            }
        }

        if (i >= 3)                           /* ִ�е�ǰͼ�������衣 */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ����˵����Element_Handle_Ramp�� */
void Element_Handle_Ramp(void)
{
    /* ִ�е�ǰͼ�������衣 */

}

/* ����˵����Element_Judgment_OutRoad�� */























/* ����˵����Element_Handle_OutRoad�� */














    /* ����ͼ��ʶ��״̬�� */






/* ִ�е�ǰͼ�������衣 */
#define CROSS_WHITE_LINE_MIN 8
#define CROSS_VALID_LINE_COUNT 3

/* �ֱ��޸�ʮ�������һ����ߣ����ⵥ��ʶ���쳣Ӱ����һ�ࡣ */
static void Repair_Cross_Border(uint8 is_left)
{
    int row;
    int near_row = -1;
    int far_row = -1;
    int near_border;
    int far_border = 0;
    int border;

    /* ��һ�ΰ��е���һ��������Ϊ������㡣 */
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

    /* ʮ��Զ�˱����������б�����Ч����ֹ��㱻��������ê�㡣 */
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
        /* δ�ҵ��ɿ�Զê��ʱ������ڱ߽磬������������ڷ���ֱ�С� */
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

    /* ���ߺ�ͳһ�޷����ؽ����ߣ��� CPU0 ���� Err�� */
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
/* ����˵����Scan_Element�� */
void Scan_Element(void)
{
    /* update jump counts every frame */
    g_left_jump_count  = (uint8)Ring_Check_Border_Jump(1U, RING_JUMP_THRESHOLD, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW);
    g_right_jump_count = (uint8)Ring_Check_Border_Jump(2U, RING_JUMP_THRESHOLD, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW);

    /* ִ�е�ǰͼ�������衣 */
    if (ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0)  /* ����Բ��ʶ��״̬���� */
    {

        Element_Judgment_Left_Rings();        /* ����Բ��ʶ��״̬���� */
        Element_Judgment_Right_Rings();       /* ִ�е�ǰͼ�������衣 */
        Element_Judgment_Zebra();             /* ִ�е�ǰͼ�������衣 */
        Element_Judgment_Bend();              /* ִ�е�ǰͼ�������衣 */
        Element_Judgment_Ramp();              /* ִ�е�ǰͼ�������衣 */
        Straight_long_judge();                /* ִ�е�ǰͼ�������衣 */
    }

    /* ִ�е�ǰͼ�������衣 */
    if (ImageFlag.Bend_Road)
    {


    }

    /* ִ�е�ǰͼ�������衣 */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ����˵����Element_Handle�� */
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
        Get_ExtensionLine();                  /* ʮ����������ֱͨ����������ߡ� */
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
}
/* ����˵����Flag_init�� */
void Flag_init(void)
{
    ImageFlag.Bend_Road              = 0;
    ImageFlag.Zebra_Flag             = 0;
    ImageFlag.Ramp                   = 0;
    ImageFlag.straight_xie           = 0;
    ImageFlag.straight_long          = 0;

}


//-------------------------------------------------------------------------------
// ��¼��ǰ��������
// ��¼��ǰ��������
// ��¼��ǰ��������
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* ִ�е�ǰͼ�������衣 */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /* ִ�е�ǰͼ�������衣 */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* ִ�е�ǰͼ�������衣 */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* ����ͼ��ʶ��״̬�� */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* ִ�е�ǰͼ�������衣 */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* ִ�е�ǰͼ�������衣 */
    }

    /* ��ʾԲ���׶α�־λ������+�׶���д�����ڵ���״̬���л� */
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

    /* ִ�е�ǰͼ�������衣 */
    /* �����Ҳ���ʾ��ǰͼ��ƫ���Ԫ��״̬ͬ֡ˢ�¡� */
    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


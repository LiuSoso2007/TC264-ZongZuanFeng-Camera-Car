/*
 * Camera.c --- MT9V03X ����ͷ���� + ͼ��ѹ�� + OTSU��ֵ�� + IPS200������ʾ
 *
 * OTSU�޸�: �Ƴ���ǰ�˳�, ��������256�� + ��ֵ�޷�30~220 (��ֱ��ȫ��)
 * ֡���Ż�: ���� Camera_ShowBinaryFast() ����ʾ��ֵͼ, SPI��������С
 */
#include "Camera.h"
#include "Shared.h"
static uint8 s_ring_exit_stable_count = 0U;  /* �����������ȶ�ֱ��֡�� */
uint8  Pixle[LCDH][LCDW];
uint8 *Image_Use[LCDH][LCDW];
uint8  Camera_Threshold = 128;
ImageDealDatatypedef ImageDeal[LCDH];        // ͼ�������ݽṹ (ÿ��һ��)
ImageStatustypedef ImageStatus;              // ͼ��״̬(OFFLine/���ߵ�)
#define COMPRESS_STEP_H (MT9V03X_H/LCDH)
#define COMPRESS_STEP_W (MT9V03X_W/LCDW)

void Camera_Init(void) { system_delay_ms(200); mt9v03x_init(); }

uint8 Camera_IsFrameReady(void) {
    uint8 f = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return f; }

uint8 (*Camera_GetImage(void))[CAMERA_W] { return mt9v03x_image; }

/*
 * Camera_CompressInit - ���� Image_Use �� mt9v03x_image ��ָ��ӳ��
 */
void Camera_CompressInit(void) {
    uint8 i, j; uint16 r, c;
    for (i = 0; i < LCDH; i++) { r = (uint16)i * COMPRESS_STEP_H;
        for (j = 0; j < LCDW; j++) { c = (uint16)j * COMPRESS_STEP_W;
            Image_Use[i][j] = &mt9v03x_image[r][c]; } } }

/*
 * Camera_OTSU_GetThreshold - �������ֵ
 * ���Ƴ���ǰ�˳��Ż� (ֱ����������䷽�����߿����оֲ���, ��ǰbreak����ڴ���λ��)
 * ��������0~255, ����޷�OTSU_MIN~OTSU_MAX
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

    /* ��һ��: ͳ�ƻҶ�ֱ��ͼ */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++)
            hist[*image[i][j]]++;

    /* �ڶ���: ����Ҷ��ܺ� */
    for (t = 0; t < 256; t++)
        totalSum += (uint64)t * hist[t];

    /* ������: ������ֵ, Ѱ�������䷽�� (����ǰ�˳�, ȫ����) */
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

    /* ���Ĳ�: ��ֵ�޷�, ��ֹ����/���ص����쳣 */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    return bestThr;
}

/*
 * Camera_GetBinaryImage - �Ҷ�ͼ��ֵ��
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
 * Camera_ShowDebug - ȫ��������ʾ (��, ����ÿ3~5֡����һ��)
 * ��: ԭʼ�Ҷ� 188x120 / ��: OTSU��ֵ / ��: ��ֵͼ 94x60
 */
/*
 * Camera_DrawCenterLines - ��ԭʼͼ�Ͷ�ֵͼ��ʵʱ����˫����
 *   ��ɫ����: �������� (ImageSensorMid)  -- �̶������Ĳο���
 *   ��ɫ����: �������� (ImageDeal[].Center) -- �����������仯
 * ��������: �ϲ�ԭʼ�Ҷ�ͼ(188x120) + �²���ֵͼ(94x60)
 */
void Camera_DrawCenterLines(void)
{
    int row;
    uint16 xo = (uint16)((MT9V03X_W - LCDW) / 2);  /* ��ֵͼXƫ�� */

    /* ---- ��������(��ɫ����): ͼ�񴫸����������� ---- */
    /* ԭʼ�Ҷ�ͼ����: y=0~119, x=ImageSensorMid*2=94 */
    ips200_draw_line(94, 0, 94, 119, RGB565_RED);
    /* ��ֵͼ����: y=150~209, x=xo+ImageSensorMid */
    ips200_draw_line(xo + ImageSensorMid, 150, xo + ImageSensorMid, 209, RGB565_RED);

    /* ---- ��������(��ɫ����): ��������ImageDeal[row].Center ---- */
    /* ������OFFLine��������Ч���ݵ���, ����2���Լ���SPI���� */
    /* �����˵㶼����λ��OFFLine���ϵ���Ч�������� */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
    {
        if (ImageDeal[row].Center < 0 || ImageDeal[row].Center >= LCDW) continue;
        if (ImageDeal[row-2].Center < 0 || ImageDeal[row-2].Center >= LCDW) continue;

        /* ԭʼ�Ҷ�ͼ (188x120): Centerֵ*2ӳ��, �к�*2ӳ�� */
        ips200_draw_line(
            (uint16)ImageDeal[row].Center * 2, (uint16)row * 2,
            (uint16)ImageDeal[row-2].Center * 2, (uint16)(row-2) * 2,
            RGB565_BLUE);

        /* ��ֵͼ (94x60, ƫ��xo,150): ʹ��ԭʼ94x60���� */
        ips200_draw_line(
            xo + (uint16)ImageDeal[row].Center, 150 + (uint16)row,
            xo + (uint16)ImageDeal[row-2].Center, 150 + (uint16)(row-2),
            RGB565_BLUE);
    }
}

void Camera_ShowDebug(void) {
    uint16 xo;
    /* ԭʼ�Ҷ�ͼ */
    ips200_show_gray_image(0, 0, mt9v03x_image[0],
        MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    /* OTSU ��ֵ */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(2, 125, "OTSU Thr:");
    ips200_show_uint(82, 125, Camera_Threshold, 3);
    /* ��ֵ��ͼ�� */
    xo = (uint16)((MT9V03X_W - LCDW) / 2);
    ips200_show_gray_image(xo, 150, Pixle[0], LCDW, LCDH, LCDW, LCDH, 1);
    /* ͼ�� */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    /* legend removed */
    Camera_ShowElementStatus();
    
    Camera_DrawCenterLines();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
//  @brief          Get_BaseLine - ��ȡ������׼��
//  ˵��: �ӵ�59-57��Ԥɨ, �ӵ�56�п�ʼ������5��(56->52)
//  ��ͼ������(ImageSensorMid=47)��������, �ҵ�������������������
//  5��ȫɨһ��, ȷ����������
//  ���� Pixle[][] ��ֵͼ�� (0=��/����, 1=��/����)
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // ��ǰ������ָ��
    int   Xsite;                                // ��ɨ��λ��
    int   row;                                  // ��ǰɨ���к�

    /* ---- ��1��: �ӵ�56�п�ʼɨ (��׼��) ---- */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // �ӵ�56�п�ʼ

    // ���������Ҳ�����, �Ұ׵��ڵ�����
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

    // ���������������, �Ұ׵��ڵ�����
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

    // ��56�����ұ߽������������
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* ��֮ǰδ���Ϊ'F', ����Ϊ'T' */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ---- ��2��: ��������ɨ��55->52�� ---- */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // ����һ���������Ҳ������ұ߽�
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
                ImageDeal[row].IsRightFind = 'F';   // �Ҳ�δ�ҵ�����
                break;
            }
        }

        // ����һ�����������������߽�
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
                ImageDeal[row].IsLeftFind = 'F';    // ���δ�ҵ�����
                break;
            }
        }

        // ���±���������λ��
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* ��֮ǰδ���Ϊ'F', ����Ϊ'T' */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* ---- ��3��: 5��ɨ����� (��������) ---- */
    // TODO: �˴�����ӻ���������Ч��У���߼�
}

//-------------------------------------------------------------------------------
//  @brief          Get_Border_And_SideType - ��ȡ��������������
//  ��[L, H]��Χ��, ��ָ�����������׵��ڵ������
//  ����: 'T'=�ҵ�����, 'W'=���а�(����), 'H'=���к�(��·)
//  @parameter      p    ��ǰ����������ָ��
//  @parameter      type ��������: 'L'=��������, 'R'=��������
//  @parameter      L, H ������Χ�߽�
//  @parameter      Q    ��������ṹ��
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* ---- ��ȫУ��: ��L/H�����ںϷ���Χ[0, LCDW-1] ---- */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // ��������: ��������ɨ��
    {
        for (i = H; i >= L; i--)
        {
            // ��(1)->��(0)����: �ҵ�����, ����λ��
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // ��¼�����������
                Q->type  = 'T';                 // ���Ϊ�ҵ�����
                break;
            }
            else if (i == L)                    // ɨ�赽����δ�ҵ�����
            {
                if (*(p + (L + H) / 2) != 0)    // �����е����ǰ�ɫ
                {
                    Q->point = (L + H) / 2;     // ��ɨ��λ��
                    Q->type  = 'W';             // ���а�(��ʧ����)
                }
                else                            // ͼ�������ݽṹ (ÿ��һ��)
                {
                    Q->point = (L + H) / 2;     // H��: ���������е�, �������LeftBorder>RightBorder
                    Q->type  = 'H';             // ���к�
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // ��������: ��������ɨ��
    {
        for (i = L; i <= H; i++)
        {
            // ��(1)->��(0)����: �ҵ�����, ����λ��
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // ��¼�����������
                Q->type  = 'T';                 // ���Ϊ�ҵ�����
                break;
            }
            else if (i == H)                    // ɨ�赽����δ�ҵ�����
            {
                if (*(p + (L + H) / 2) != 0)    // �����е����ǰ�ɫ
                {
                    Q->point = (L + H) / 2;     // ��ɨ��λ��
                    Q->type  = 'W';             // ���а�(��ʧ����)
                }
                else                            // ͼ�������ݽṹ (ÿ��һ��)
                {
                    Q->point = (L + H) / 2;     // H��: ���������е�, �������LeftBorder>RightBorder
                    Q->type  = 'H';             // ���к�
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
//  @brief          Get_AllLine - �ӻ�׼������ɨ��ȫ����������
//  ��Get_BaseLine(56->52)֮��, ��51�п�ʼ����52�н�����µ���ɨ�赽0��
//  ��������: �ڵ�ǰ����һ�б���λ��+/-ImageScanInterval��Χ����������
//  �쳣����: ���������Ҳ��������򴥷�����; ���а״���OFFLine
//  @parameter      void
//  @return         void
//  @note           ���� ImageDeal[52] (��׼������) �� Pixle[][] (��ֵͼ��)
//  @note           OFFLine: ������������ͬʱ�����ж�Ϊ����ƫ������
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // ��ǰ������ָ��
    int   row;                                  // ��ǰɨ���к�
    int   IntervalLow, IntervalHigh;            // ������������߽�
    int   i;                                    // ��ʱ��������

    /* ---- ��ʼ��״̬���� ---- */
    ImageStatus.OFFLine          = 2;           // �����к�(��ʼΪ2)
    ImageStatus.Miss_Left_lines  = 0;           // ���������ʧ����
    ImageStatus.Miss_Right_lines = 0;           // �Ҳ�������ʧ����
    ImageStatus.WhiteLine        = 0;           // ��ɫ�м���(ʮ��ʶ��)
    ImageStatus.WhiteLine_L      = 0;           // �����м���
    ImageStatus.WhiteLine_R      = 0;           // �Ҳ���м���
    ImageStatus.OFFLineBoundary  = 0;           // ���߽߱��к�
    ImageStatus.Det_True         = 0;           // ��Ч����־

    /*
     * ��51�п�ʼ, ��52��(��׼��)Ϊ�ο�����ɨ��
     * ���е�������ֱ��0�л򴥷�OFFLine����
     */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // [0]=��������, [1]=�Ҳ������
        PicTemp = Pixle[row];

        /* ============================================================
         * �Ҳ�����: ����һ���ұ߽� +/- ImageScanInterval ��Χ��ɨ��
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // �޷���[0, 93]
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ============================================================
         * �������: ����һ����߽� +/- ImageScanInterval ��Χ��ɨ��
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ============================================================
         * �����������ͽ��б��ߴ���:
         * 'T'=����: ����Ϊ�ҵ��ı���λ��
         * 'W'=ȫ��: ʹ����һ�б���ֵ (����+1)
         * 'H'=ȫ��: ɨ��������û�а׵�, ��������
         * ============================================================ */
        if (JumpPoint[0].type == 'W')           // ������а�(����)
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // ������һ����߽�
            ImageStatus.Miss_Left_lines++;      // ��ඪʧ����+1
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // �ҵ�����, ���㶪ʧ����
        }

        if (JumpPoint[1].type == 'W')           // �Ҳ����а�(����)
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // ������һ���ұ߽�
            ImageStatus.Miss_Right_lines++;     // �Ҳඪʧ����+1
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // �ҵ�����, ���㶪ʧ����
        }

        /* ---- ��¼�ҵ�״̬ ---- */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ---- ���м���(����ͬʱΪ��) ---- */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // �ۼư�ɫ�м���
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* ������м��� */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ---- ���㱾���������� ---- */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /*
         * H�����޸�: ȫ���г����������������ұ�
         * ��ĳ��Ϊȫ��Hʱ, �ӱ߽����ڲ������ױ�ڵ�
         */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ---- ��H��: �ӵ�ǰ��߽���������, Ѱ������(�׵�)�ض����� ---- */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i - 1) == 0)  // ��->������: �ҵ�������߽�
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ---- ��H��: �ӵ�ǰ�ұ߽���������, Ѱ������(�׵�)�ض����� ---- */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 1 && *(PicTemp + i + 1) == 0)  // ��->������: �ҵ������ұ߽�
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ---- �޸������¼������� ---- */
            /* H���޸�ʧ��ʱ������һ�б߽�, ����������Ļ��Ե�γ�ֱ�� */
            if (ImageDeal[row].IsLeftFind == 'H')  { ImageDeal[row].LeftBorder  = ImageDeal[row + 1].LeftBorder; }
            if (ImageDeal[row].IsRightFind == 'H') { ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; }
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ============================================================
         * OFFLine�����ж�: ����ͬʱ������ʧ����3�С�
         * ����: ʮ��Ԫ��˫��ȫ��(W)������OFFLine, ��֤WhiteLine���ۻ���8�������ߡ�
         * ============================================================ */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3
            && !(JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W'))  /* ˫��ȫ��=ʮ��, ���������� */
        {
            ImageStatus.OFFLine = row;          // ��¼������ʼ�к�
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



/* ================================================================
 * Ԫ��ʶ������ (TC264: 94�п�, AnCaiԭ��x1.175��ӳ��)
 * ================================================================ */
const uint8 Half_Road_Wide[60] = {           /* ���·���(����->Զ���ݼ�) */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* ��������� */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* ================================================================
 * ͼ��Ԫ�ر�־
 * ================================================================ */
ImageFlagtypedef ImageFlag;                  /* ͼ��Ԫ�ر�־ */

/* ================================================================
 * Helper: Straight_Judge - ֱ���б�
 * dir=1: ����߽����, dir=2: ���ұ߽����
 * ���ؾ�����S, S<1 ��Ϊֱ��
 * ================================================================ */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* ��߽� */
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
    case 2: /* �ұ߽� */
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
 * ��ֱ���ж��봦��
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
 * б��ֱ���ж� (�����������)
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
 * ���ʶ�� (���ұ���+����)
 * ================================================================ */
void Element_Judgment_Bend(void)
{
    /*
     * ����������: ����������Ԫ��״̬ʱ���Լ�⡣
     * ԭ OFFLine < 3 �߼�����: OFFLine ��������ͬʱ���߲Ŵ���,
     * �����ֻ������, OFFLine ʼ��Ϊ2, ���������Զ�޷�ʶ��
     * ����: ����˫�ඪ�߼������ж� �� ˫�඼׷�����ò��Ǵ�ֱ����
     */
    if (ImageFlag.image_element_rings != 0
        || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1)
        return;
    /* ponytail直道守卫: OFFLine<5时赛道完全可见，不可能是弯道
       防止噪声导致Miss计数累积引发误判 (安财原始OFFLine>=14, TC264适配60行->5) */
    if (ImageStatus.OFFLine < 5)
        return;

    if (ImageStatus.Miss_Left_lines < 4
        && ImageStatus.Miss_Right_lines < 4)
        return;  /* ˫�඼׷������ = ��ֱ��, ��������� */

    /* ����: �ұ߽翿��(<59), �Ҳ�׷������, ��ඪ�߶��� (����ת������) */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 12
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 1;              /* ���� */
    }

    /* ����: ��߽翿��(>35), ���׷������, �Ҳඪ�߶��� (����ת�Ҳ����) */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 12
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 3.0f)
    {
        ImageFlag.Bend_Road = 2;              /* ���� */
    }
}

/* ================================================================
 * �������: �õ�·����ȫ������
 * ================================================================ */
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
            LimitL(ImageDeal[row].Center);    /* 限幅 >= 0 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* 右弯: 只靠左边界可见, center=左边界+半宽(向右偏移) */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* 限幅 <= 93 */
        }
    }
}

/* ================================================================
 * ��Բ��ʶ��
 * ================================================================ */
static void Ring_State_Update(void)
{
    if (ImageFlag.image_element_rings_flag == RING_STATE_ENTRY)
    {
        /* ������ǰ�ж�˵�������Ѿ�����Բ�����塣 */
        if (ImageStatus.OFFLine >= 5)
        {
            ImageFlag.image_element_rings_flag = RING_STATE_INSIDE;
        }
    }
    else if (ImageFlag.image_element_rings_flag == RING_STATE_INSIDE)
    {
        /* ˫�������ȶ��ɼ�����������������ֹ�ٴ�ʶ��ͬһԲ���� */
        if (ImageStatus.OFFLine <= 2
            && ImageStatus.Miss_Left_lines < 4
            && ImageStatus.Miss_Right_lines < 4)
        {
            ImageFlag.image_element_rings_flag = RING_STATE_EXIT;
            s_ring_exit_stable_count = 0U;
        }
    }
    else if (ImageFlag.image_element_rings_flag == RING_STATE_EXIT)
    {
        if (ImageStatus.OFFLine <= 2
            && ImageStatus.Miss_Left_lines < 4
            && ImageStatus.Miss_Right_lines < 4)
        {
            if (s_ring_exit_stable_count < RING_EXIT_STABLE_FRAMES)
            {
                s_ring_exit_stable_count++;
            }
            if (s_ring_exit_stable_count >= RING_EXIT_STABLE_FRAMES)
            {
                ImageFlag.image_element_rings = 0;
                ImageFlag.image_element_rings_flag = RING_STATE_IDLE;
                ImageFlag.ring_big_small = 0;
                s_ring_exit_stable_count = 0U;
            }
        }
        else
        {
            /* �ٴζ���ֻ�ؼ��ȶ�֡�����������˵��뻷״̬�� */
            s_ring_exit_stable_count = 0U;
        }
    }
}

void Element_Judgment_Left_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Left_Less_Num = 0;

    /* ����ͬԴ�ż�����Բ�������ȳ�������������ߣ�ֱ���������ô����� */
    if (ImageStatus.Miss_Right_lines > 3
        || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 2 || Straight_Judge(2, 5, SCAN_BASE_END_ROW) > 3.0f   /* ����������ֵ: ����Բ���΢���� */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* ��������ʱ��ֹ����Բ�����ߣ���ֹ����ֱ�����ġ� */

    /* ����Ƿ������е�����߶���'W'(ȫ��) */
    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: ����TC264�з�Χ48->44 */
        {
            if (ImageDeal[r].IsLeftFind == 'W') return;
        }
    }

    /* ��������ߴ���������� */
    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 1].LeftBorder) > 4  /* abs: ������ⷽ�������� */)
        {
            Left_Less_Num++;
            /* �ۼ�������������� */
            if (Left_Less_Num == 1) {
                /* ��һ������ʱ, ���ڴ˼�¼��ʼ�к� */
            }
        }
    }

    if (Left_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 1;    /* ��Բ�� */
        ImageFlag.image_element_rings_flag = RING_STATE_ENTRY;
        s_ring_exit_stable_count = 0U;
    }
}

/* ================================================================
 * ��Բ��ʶ�� (�߼��Գ�)
 * ================================================================ */
void Element_Judgment_Right_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Right_Less_Num = 0;

    /* ����ͬԴ�ż�����Բ�������ȳ����Ҳ��������ߣ�ֱ���������ô����� */
    if (ImageStatus.Miss_Left_lines > 3
        || ImageStatus.Miss_Right_lines < 15
        || ImageStatus.OFFLine > 2 || Straight_Judge(1, 5, SCAN_BASE_END_ROW) > 3.0f   /* ����������ֵ: ����Բ���΢���� */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;  /* ��������ʱ��ֹ����Բ�����ߣ���ֹ����ֱ�����ġ� */

    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: ����TC264�з�Χ */
        {
            if (ImageDeal[r].IsRightFind == 'W') return;
        }
    }

    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (abs(ImageDeal[Ysite].RightBorder - ImageDeal[Ysite - 1].RightBorder) > 4  /* abs: ������ⷽ�������� */)
        {
            Right_Less_Num++;
        }
    }

    if (Right_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 2;    /* ��Բ�� */
        ImageFlag.image_element_rings_flag = RING_STATE_ENTRY;
        s_ring_exit_stable_count = 0U;
    }
}

/* ================================================================
 * ��Բ������: �õ�·������
 * ================================================================ */
void Element_Handle_Left_Rings(void)
{
    int row;

    Ring_State_Update();

    if (ImageFlag.image_element_rings_flag == RING_STATE_ENTRY
        || ImageFlag.image_element_rings_flag == RING_STATE_INSIDE)
    {
        /* ���߲���: ����=�����+������ */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }

}

/* ================================================================
 * ��Բ��ʶ��
 * ================================================================ */
void Element_Handle_Right_Rings(void)
{
    int row;

    Ring_State_Update();

    if (ImageFlag.image_element_rings_flag == RING_STATE_ENTRY
        || ImageFlag.image_element_rings_flag == RING_STATE_INSIDE)
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }

}

/* ================================================================
 * ������ʶ��: ����20~32��Χ�ڼ�������ܶ�
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

    if (NUM > 8)                              /* �����ܶȴ��: �ж�Ϊ������ */
    {
        if (ImageDeal[SCAN_BASE_START_ROW].Center > 47)  /* TC264: ͼ���94����λ47, ����ƫ��=��౻�ڵ� */
            ImageFlag.Zebra_Flag = 2;           /* �Ҳ�ɼ�, ������ */
        else                                  /* ����ƫ��=�Ҳ౻�ڵ� */
            ImageFlag.Zebra_Flag = 1;           /* ���ɼ�, �Ҳ���� */
    }
}

/* ================================================================
 * �����ߴ���: �õ�·����ȫ������
 * ================================================================ */
void Element_Handle_Zebra(void)
{
    int row;

    if (ImageFlag.Zebra_Flag == 1)            /* �����: ����ڵ�, ���ұ����������� */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Road_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }
    else if (ImageFlag.Zebra_Flag == 2)       /* �Ұ���: �Ҳ��ڵ�, ��������������� */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Road_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }
}

/* ================================================================
 * �µ�ʶ��: OFFLine��� + ��� + ���ļ��
 * ================================================================ */
void Element_Judgment_Ramp(void)
{
        return;                              /* �ݲ�����, �������Ż� */
    int Ysite;
    int i = 0;                           /* ��Ч�м����� */

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

        if (i >= 3)                           /* �ۼ�3������ */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ================================================================
 * ͼ��Ԫ�ر�־
 * ================================================================ */
void Element_Handle_Ramp(void)
{
    /* Ramp������δʵ��, ���������� */

}

/* ================================================================
 * ��·ʶ��: OFFLine��� + ���ߺ������ҵ�����
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
 * ��·����: ��������ָ�����
 * ================================================================ */
void Element_Handle_OutRoad(void)
{
    int Ysite, Xsite;
    int gray_sum = 0;

    /* �����������(����)�׵��� */
    for (Ysite = 35; Ysite < 55; Ysite++)
    {
        for (Xsite = 30; Xsite < 64; Xsite++) /* TC264ͼ������ */
        {
            gray_sum += Pixle[Ysite][Xsite];
        }
    }

    /* �׵��㹻�� -> �˳���·״̬ */
    if (gray_sum > 400 && ImageStatus.OFFLine < 20)
    {
        ImageFlag.Out_Road = 0;
    }
}

/* ================================================================
 * ʮ�ֲ���: ���ȫ����, ��������Ч���Ƶ�����
 * ================================================================ */
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
/* ================================================================
 * Ԫ��ɨ�����: ���ø�Ԫ��ʶ����
 * ע��: Ԫ��ʶ�������ȼ��ͻ����ϵ
 * ================================================================ */
void Scan_Element(void)
{
    /* ����������Ԫ��״̬�½���Ԫ��ʶ�� */
    if (ImageFlag.Out_Road == 0 && ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0)  /* ֱ��/�����������Ԫ�ؼ�� */
    {
        Element_Judgment_OutRoad();           /* ��· */
        Element_Judgment_Left_Rings();        /* ��Բ�� */
        Element_Judgment_Right_Rings();       /* ��Բ�� */
        Element_Judgment_Zebra();             /* ������ */
        Element_Judgment_Bend();              /* ��� */
        Element_Judgment_Ramp();              /* �µ� */
        Straight_long_judge();                /* ��ֱ�� */
    }

    /* ���״̬���Լ���· */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();
        if (ImageFlag.Out_Road) ImageFlag.Bend_Road = 0;
    }

    /* ���״̬���Լ������� */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ================================================================
 * Ԫ�ش������: ����ʶ���Ԫ�ص��ö�Ӧ�������
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
        Get_ExtensionLine();                  /* ʮ����������ֱͨ����������ߡ� */
    else if (ImageFlag.straight_long)
        Straight_long_handle();
    else if (ImageFlag.Bend_Road != 0)
        Element_Handle_Bend();
}
/* ================================================================
 * ��Բ��ʶ��
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
//  @brief          Camera_ShowElementStatus - ��ʾ��ǰԪ��״̬
//  ��IPS200�ײ���ʾ��ǰʶ�𵽵�����Ԫ��(��д��ʶ)
//  ��д: zhi=ֱ�� wan_L/R=��� shi=ʮ�� huan_L/R=Բ�� banma=���� po=�µ� duan=��·
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* �ײ�״̬��: ����, ���� */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /*
     * ������ʾ��ǰԪ��, �����ȼ����ϵ���
     * λ��: y=225 (��Ļ240��, �ײ�15px�и�)
     */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* ��Բ�� */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* ��Բ�� */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* ʮ�� */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* ��Ԫ�� */
    }

    /* ������м��� */
    /* �����Ҳ���ʾ��ǰͼ��ƫ���Ԫ��״̬ͬ֡ˢ�¡� */
    ips200_show_string(120, 225, "Err:");
    ips200_show_float(152, 225, Err, 3, 2);

    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


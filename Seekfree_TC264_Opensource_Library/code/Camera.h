#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ips200.h"

/*
 * Camera.h --- MT9V03X ����ͷ���� + ͼ��ѹ�� + OTSU��ֵ�� + IPS200������ʾ
 *
 * ������� zf_device_mt9v03x ��, ��չ����:
 *   1. ����ͷ��ʼ�� (UART ���� + ERU �ⲿ�ж� + DMA ���ݰ���)
 *   2. ͼ��ѹ�� - �� 188x120 ԭʼ�Ҷ�ͼ�ȱ�ѹ���� 94x60
 *   3. OTSU ��򷨶�ֵ�� - ����Ӧ����������ֵ, �Ҷ�ת�ڰ�
 *   4. IPS200������ʾ - ԭʼͼ+ѹ��ͼ+��ֵ������ʾ
 *
 * ����ͼ�������� (����/����/Ԫ��ʶ��) ������ Image_Process() ��ʵ��
 *
 * ���߶���:
 *   TXD   -> P02_3 (UART1 RX)        VCC  -> 3.3V
 *   RXD   -> P02_2 (UART1 TX)        GND  -> GND
 *   PCLK  -> P02_1 (ERU_CH2)         ������������
 *   VSY   -> P02_0 (ERU_CH3)
 *   D0-D7 -> P00_0 ~ P00_7
 */

/* ---- ԭʼͼ��ߴ� (������ɿ�) ---- */
#define CAMERA_W       MT9V03X_W        // 188
#define CAMERA_H       MT9V03X_H        // 120

/* ---- ѹ����ͼ��ߴ� (2:1 �ȱ�ѹ��) ---- */
#define LCDW           94               // ѹ������� (��) = 188/2
#define LCDH           60               // ѹ����߶� (��) = 120/2

/* ---- OTSU�����ֵ (��ֹ����/���ص����쳣) ---- */
#define OTSU_MIN       30               // ��С��ֵ
#define OTSU_MAX       220              // �����ֵ
#define OTSU_BIAS      20               // ��ֵƫ�ã�����threshold=clamp(otsu)+bias

/*
 * ��Ļ����˵��:
 *   188x120 ԭʼͼ -> ��ʾ���� 188x120
 *   94x60  ѹ��ͼ -> ��ʾ���� 94x60 (����)
 *   �ܸ߶� 120 + 10(���) + 60 = 190 < 240 ��Ļ�߶�
 */
/* ---- ����ͷ�������� (Camera) ---- */
// TC264: 94�п�, ���� = 94/2 = 47
#define ImageSensorMid    (LCDW / 2)           // ͼ�񴫸�������λ��: 47

// ɨ��˵��: �ӵ�59~57��Ԥɨ, �ӵ�56�п�ʼ������5��(56->52)
// ��ͼ������(ImageSensorMid=47)����������, ȷ������
// 5��ȫɨһ��, ȷ����������
#define SCAN_BASE_START_ROW    59              // ɨ����ʼ�� (��ײ�,���복���,������ɿ�,AnCai���)
#define SCAN_BASE_END_ROW      55              // ɨ������� (��5�л���: 59,58,57,56,55)

// �޷���, ��L/H������[0, LCDW-1]
#define LimitL(L)  ((L) = ((L) < 1)  ? 1  : (L))    // AnCai: L>=1��֤p[i-1]��Խ��
#define LimitH(H)  ((H) = ((H) > (LCDW - 2)) ? (LCDW - 2) : (H))  // AnCai: H<=92��֤p[i+1]��Խ��

/* ---- ͼ�������ݽṹ ---- */
typedef struct {
    uint8 IsRightFind;
    uint8 IsLeftFind;
    int   Wide;
    int   LeftBorder;
    int   RightBorder;
    int   Center;
} ImageDealDatatypedef;


/* ---- ȫ��ͼ������ ---- */
extern uint8  Pixle[LCDH][LCDW];                // ��ֵͼ (0=��/����, 1=��/����)
extern uint8 *Image_Use[LCDH][LCDW];            // ѹ����Ҷ�ͼ��ָ������
extern uint8  Camera_Threshold;                 // ��ǰOTSU��ֵ (0~255)

/* ---- ͼ������ ---- */
extern ImageDealDatatypedef ImageDeal[LCDH];   // ÿ��ͼ�������


/* ---- ��ʼ�� ---- */
void Camera_Init(void);
void Camera_CompressInit(void);                  // ͼ��ѹ����ʼ�� (�������һ��)

/* ---- ͼ��ɼ� ---- */
uint8 Camera_IsFrameReady(void);                 // ��� mt9v03x_finish_flag ��־λ
uint8 (*Camera_GetImage(void))[CAMERA_W];        // ���� mt9v03x_image ԭʼͼ��ָ��

/* ---- ͼ���� ---- */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row);
                                                 // �������Ѷ�ֵ����ֵ
void  Camera_GetBinaryImage(void);               // �Ҷ�ͼ -> ��ֵ�� (�Զ�����OTSU)

/* ---- IPS200������ʾ ---- */
void  Camera_ShowDebug(void);                    // IPS200 ��ʾԭʼͼ+ѹ��ͼ+��ֵ
void  Camera_ShowBinaryFast(void);              // ������ʾ��ֵͼ (�Ż�, SPI��������С)


/* ---- ͼ������ ---- */
void  Camera_ShowElementStatus(void);            // ��ʾ��ǰԪ��״̬(��д��ʶ)
void  Get_BaseLine(void);                       // ��ȡ��׼��: ��56->52, 5��
// ��������: �ڵ�ǰ����һ�б���λ��+/-ImageScanInterval��Χ��
#define ZEBRA_SCAN_LEFT            17
#define ZEBRA_SCAN_RIGHT           77

#define ImageScanInterval  5                   // ��������(����)

/* ---- �����ṹ ---- */
typedef struct {
    int   point;                               // ���������
    uint8 type;                                // ����: 'T'=����, 'W'=ȫ�׶���, 'H'=ȫ��
} JumpPointtypedef;

/* ---- ͼ�������ݽṹ ---- */
typedef struct {
    int16 OFFLine;                             // ������: �Ӹ��п�ʼ������
    int16 Miss_Left_lines;                     // ���������ʧ����
    int16 Miss_Right_lines;                    // �Ҳ�������ʧ����
    int16 WhiteLine;                           /* ��ɫ�м���(ʮ��) */
    int16 OFFLineBoundary;                     /* ���߽߱� */
    int16 Det_True;                            /* ��Ч����־ */
    int16 WhiteLine_L;                         /* ������ */
    int16 WhiteLine_R;                         /* �Ҳ���� */
} ImageStatustypedef;

extern ImageStatustypedef ImageStatus;         // ͼ��״̬ȫ�ֱ���

/* Բ��ֻ���߸��׶ε����ƽ���������image_element_rings�������档 */

/* ---- �ڶ���ⷨ���� ---- */
#define IMG_BLACK                   0
#define IMG_WHITE                   1
#define BH_BOTTOM_START_ROW        52
#define BH_LEFT_COL_MIN             1
#define BH_LEFT_COL_MAX            12
#define BH_RIGHT_COL_MIN           82
#define BH_RIGHT_COL_MAX           93
#define VALLEY_SCAN_START_ROW      59
#define VALLEY_SCAN_COL_LEFT       10
#define VALLEY_SCAN_COL_RIGHT      75
#define VALLEY_MAX_ROW             59
#define VALLEY_MIN_ROW             28
#define EXIT_LOST_MIN               8
#define FILL_ENTRY_OFFSET          10
#define FILL_INSIDE_OFFSET         14
#define FILL_EXIT_OFFSET            8
#define FILL_RECOVERY_OFFSET        0

#define RING_JUMP_THRESHOLD         2 //断点判定
#define RING_JUMP_MIN_COUNT         3
#define RING_JUMP_OTHER_MAX         2

#define RING_STATE_IDLE       0
#define RING_STATE_CONFIRM    1
#define RING_STATE_APPROACH   2
#define RING_STATE_ENTRY      3
#define RING_STATE_INSIDE     4
#define RING_STATE_EXIT       5
#define RING_STATE_RECOVERY   6

/* ����֡�볬ʱ������Ϊ���Ӿ���������ֹ��֡���к�״̬������ */
#define RING_CONFIRM_FRAMES       3U
#define RING_EXIT_CONFIRM_FRAMES  2U
#define RING_EXIT_STABLE_FRAMES   8U
#define RING_RECOVERY_FRAMES      12U
#define RING_CONFIRM_MAX_FRAMES   30U
#define RING_APPROACH_MAX_FRAMES  24U
#define RING_ENTRY_MAX_FRAMES     30U
#define RING_INSIDE_MAX_FRAMES    120U
#define RING_EXIT_MAX_FRAMES      60U
#define RING_RECOVERY_MAX_FRAMES  40U

void  Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q);
                                               // ��ȡ��������������
void  Get_AllLine(void);                       // ȫ��ɨ��: ��51������ɨ��0

/* ---- ͼ���־�ṹ ---- */
typedef struct {
    int16 Bend_Road;                           /* ���: 0=ֱ�� 1=���� 2=���� */
    int16 image_element_rings;                 /* Բ��: 0=�� 1=��Բ�� 2=��Բ�� */
    int16 ring_big_small;                      /* Բ����С: 0=�� 1=�� 2=С�� */
    int16 image_element_rings_flag;            /* Բ��������־ */
    int16 straight_long;                       /* ��ֱ����־ */
    int16 straight_xie;                        /* б��ֱ����־ */
    int16 Zebra_Flag;                          /* ������: 0=�� 1=��� 2=�Ҳ� */
    int16 Ramp;                                /* �µ�: 0=�� 1=��⵽ */

} ImageFlagtypedef;

/* ---- ͼ�������ݽṹ ---- */
// ����Ԫ��: WhiteLine(ʮ��), OFFLineBoundary(���߽߱�), Det_True(��Ч���)
// ״̬: OFFLine/Miss_Left_lines/Miss_Right_lines ��ImageStatus��

extern ImageFlagtypedef ImageFlag;             /* ͼ���־ȫ�ֱ��� */
extern int16_t g_ZebraSum;          /* �����߼���ֵ֮�ͣ�����Ļ��ʾ */
extern volatile int g_corner_black_max;   /* ???????????? */
extern volatile int g_bottom_black_width; /* ????W-B???? */
extern volatile int g_ring_miss_cnt;      /* ?????? */
extern volatile uint8 g_left_jump_count;  /* left border jump count */
extern volatile uint8 g_right_jump_count; /* right border jump count */
extern volatile int g_approach_valley_row; /* debug: approach valley row */
extern volatile uint8 g_edge_squeezed_dbg;  /* debug: squeeze state */
extern volatile uint8 g_ring_phase_dbg;     /* debug: valley phase 0/1/2 */

/* ---- ��·���ȳ��� (TC264�п�94, AnCaiԭ��x1.175��ӳ��) ---- */
extern const uint8 Half_Road_Wide[60];         /* ���·����: ����~Զ�� */
extern const uint8 Half_Bend_Wide[60];         /* ������ */

/* ---- ͼ�������ݽṹ ---- */
float Straight_Judge(uint8 dir, uint8 start, uint8 end);     // ֱ���ж�(S<1Ϊֱ��)
void  Straight_long_judge(void);                             // ��ֱ���ж�
void  Straight_long_handle(void);                            // ��ֱ������
void  Straight_xie_judge(void);                              // б��ֱ���ж�
void  Element_Judgment_Bend(void);                           // ���ʶ��
void  Element_Handle_Bend(void);                             // �������
void  Element_Judgment_Left_Rings(void);                     // ��Բ��ʶ��
void  Element_Handle_Left_Rings(void);                       // ��Բ������
void  Element_Judgment_Right_Rings(void);                    // ��Բ��ʶ��
void  Element_Handle_Right_Rings(void);                      // ��Բ������
void  Element_Judgment_Zebra(void);                          // ������ʶ��
void  Element_Handle_Zebra(void);                            // �����ߴ���
void  Element_Judgment_Ramp(void);                           // �µ�ʶ��
void  Element_Handle_Ramp(void);                             // �µ�����


void  Get_ExtensionLine(void);                               // ʮ�ֲ���
void  Scan_Element(void);                                    // Ԫ��ɨ�����
void  Element_Handle(void);                                  // Ԫ�ش������
void  Flag_init(void);                                       // ��־��ʼ��

void  Camera_ShowElementStatus(void);            // ��ʾ��ǰԪ��״̬(��д��ʶ)

#endif

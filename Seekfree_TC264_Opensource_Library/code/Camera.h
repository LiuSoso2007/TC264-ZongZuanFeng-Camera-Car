#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ips200.h"

/*
 * Camera.h --- MT9V03X 摄像头驱动 + 图像压缩 + OTSU二值化 + IPS200调试显示
 *
 * 基于逐飞 zf_device_mt9v03x 库, 扩展功能:
 *   1. 摄像头初始化 (UART 配置 + ERU 外部中断 + DMA 数据搬运)
 *   2. 图像压缩 - 将 188x120 原始灰度图等比压缩至 94x60
 *   3. OTSU 大津法二值化 - 自适应计算最优阈值, 灰度转黑白
 *   4. IPS200调试显示 - 原始图+压缩图+阈值叠加显示
 *
 * 更多图像处理功能 (搜线/补线/元素识别) 另行在 Image_Process() 中实现
 *
 * 接线定义:
 *   TXD   -> P02_3 (UART1 RX)        VCC  -> 3.3V
 *   RXD   -> P02_2 (UART1 TX)        GND  -> GND
 *   PCLK  -> P02_1 (ERU_CH2)         其余引脚悬空
 *   VSY   -> P02_0 (ERU_CH3)
 *   D0-D7 -> P00_0 ~ P00_7
 */

/* ---- 原始图像尺寸 (来自逐飞库) ---- */
#define CAMERA_W       MT9V03X_W        // 188
#define CAMERA_H       MT9V03X_H        // 120
#define CAMERA_SIZE    (CAMERA_W * CAMERA_H)

/* ---- 压缩后图像尺寸 (2:1 等比压缩) ---- */
#define LCDW           94               // 压缩后宽度 (列) = 188/2
#define LCDH           60               // 压缩后高度 (行) = 120/2

/* ---- OTSU???? (???/???????) ---- */
#define OTSU_MIN       30               // ????
#define OTSU_MAX       220              // ????

/*
 * 屏幕布局说明:
 *   188x120 原始图 -> 显示区域 188x120
 *   94x60  压缩图 -> 显示区域 94x60 (居中)
 *   总高度 120 + 10(间隔) + 60 = 190 < 240 屏幕高度
 */
/* ---- ??????? (Camera) ---- */
// TC264: 94???, ??? = 94/2 = 47
#define ImageSensorMid    (LCDW / 2)           // ?????????: 47

// ????: ???59~57, ??56????????5?(56->52)
// ?????(ImageSensorMid=47)????, ?????
// 5???????, ??????
#define SCAN_BASE_START_ROW    56              // ?????
#define SCAN_BASE_END_ROW      52              // ????? (5?)
#define SCAN_VALIDATE_COUNT    5               // ??????

// ?????, ??????????[0, LCDW-1]?
#define LimitL(L)  ((L) = ((L) < 0)  ? 0  : (L))
#define LimitH(H)  ((H) = ((H) > (LCDW - 1)) ? (LCDW - 1) : (H))

/* ---- ?????? ---- */
typedef struct {
    uint8 IsRightFind;
    uint8 IsLeftFind;
    int   Wide;
    int   LeftBorder;
    int   RightBorder;
    int   Center;
} ImageDealDatatypedef;


/* ---- 全局图像数组 ---- */
extern uint8  Pixle[LCDH][LCDW];                // 二值化图像 (0=黑, 1=白)
extern uint8 *Image_Use[LCDH][LCDW];            // 压缩后灰度图像指针数组
extern uint8  Camera_Threshold;                 // 当前OTSU阈值 (0~255)

/* ---- ???? ---- */
extern ImageDealDatatypedef ImageDeal[LCDH];   // ??????


/* ---- 初始化 ---- */
void Camera_Init(void);
void Camera_CompressInit(void);                  // 图像压缩初始化 (仅需调用一次)

/* ---- 图像采集 ---- */
uint8 Camera_IsFrameReady(void);                 // 检查 mt9v03x_finish_flag 标志位
uint8 (*Camera_GetImage(void))[CAMERA_W];        // 返回 mt9v03x_image 原始图像指针

/* ---- 图像处理 ---- */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row);
                                                 // 大津法求最佳二值化阈值
void  Camera_GetBinaryImage(void);               // 灰度图 -> 二值化 (自动调用OTSU)

/* ---- IPS200调试显示 ---- */
void  Camera_ShowDebug(void);                    // IPS200 显示原始图+压缩图+阈值
void  Camera_ShowBinaryFast(void);              // ?????? (??, SPI?????)


/* ---- ???? ---- */
void  Camera_ShowElementStatus(void);            // ??????????(????)
void  Get_BaseLine(void);                       // ????: ?56->52, 5?????
// ??????: ???????????+/-ImageScanInterval????
#define ImageScanInterval  5                   // ??????(?)

/* ---- ????? ---- */
typedef struct {
    int   point;                               // ?????
    uint8 type;                                // ??: 'T'=??, 'W'=????, 'H'=???
} JumpPointtypedef;

/* ---- ?????? ---- */
typedef struct {
    int16 OFFLine;                             // ???: ???????????
    int16 Miss_Left_lines;                     // ???????
    int16 Miss_Right_lines;                    // ???????
    int16 WhiteLine;                           /* ??????(????) */
    int16 OFFLineBoundary;                     /* ????? */
    int16 Det_True;                            /* ?????? */
    int16 WhiteLine_L;                         /* ????? */
    int16 WhiteLine_R;                         /* ????? */
} ImageStatustypedef;

extern ImageStatustypedef ImageStatus;         // ????????

/* ---- ?????? ---- */
void  Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q);
                                               // ?????????
void  Get_AllLine(void);                       // ????: ??51??????0

/* ---- ??????? ---- */
typedef struct {
    int16 Bend_Road;                           /* ??: 0=?? 1=?? 2=?? */
    int16 image_element_rings;                 /* ??: 0=? 1=??? 2=??? */
    int16 ring_big_small;                      /* ????: 0=?? 1=??? 2=??? */
    int16 image_element_rings_flag;            /* ?????? */
    int16 straight_long;                       /* ????? */
    int16 straight_xie;                        /* ?????? */
    int16 Zebra_Flag;                          /* ???: 0=? 1=??? 2=??? */
    int16 Ramp;                                /* ??: 0=? 1=??? */
    int16 Out_Road;                            /* ??: 0=? 1=?? */
} ImageFlagtypedef;

/* ---- ?????? ---- */
// ????: WhiteLine(????), OFFLineBoundary(????), Det_True(????)
// ??: OFFLine/Miss_Left_lines/Miss_Right_lines ??????

extern ImageFlagtypedef ImageFlag;             /* ???????? */

/* ---- ????? (TC264??, AnCai?x1.175??) ---- */
extern const uint8 Half_Road_Wide[60];         /* ????: ???~??? */
extern const uint8 Half_Bend_Wide[60];         /* ???? */

/* ---- ?????? ---- */
float Straight_Judge(uint8 dir, uint8 start, uint8 end);     // ?????(S<1???)
void  Straight_long_judge(void);                             // ?????
void  Straight_long_handle(void);                            // ?????
void  Straight_xie_judge(void);                              // ??????
void  Element_Judgment_Bend(void);                           // ????
void  Element_Handle_Bend(void);                             // ????
void  Element_Judgment_Left_Rings(void);                     // ?????
void  Element_Handle_Left_Rings(void);                       // ?????
void  Element_Judgment_Right_Rings(void);                    // ?????
void  Element_Handle_Right_Rings(void);                      // ?????
void  Element_Judgment_Zebra(void);                          // ?????
void  Element_Handle_Zebra(void);                            // ?????
void  Element_Judgment_Ramp(void);                           // ????
void  Element_Handle_Ramp(void);                             // ????
void  Element_Judgment_OutRoad(void);                        // ????
void  Element_Handle_OutRoad(void);                          // ????
void  Get_ExtensionLine(void);                               // ?????
void  Scan_Element(void);                                    // ??????
void  Element_Handle(void);                                  // ??????
void  Flag_init(void);                                       // ?????

void  Camera_ShowElementStatus(void);            // ??????????(????)

#endif
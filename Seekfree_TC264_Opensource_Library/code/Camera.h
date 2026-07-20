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

/* ---- OTSU大津法阈值 (防止过暗/过曝导致异常) ---- */
#define OTSU_MIN       30               // 最小阈值
#define OTSU_MAX       220              // 最大阈值
#define OTSU_BIAS      20               // 阈值偏置，最终threshold=clamp(otsu)+bias

/*
 * 屏幕布局说明:
 *   188x120 原始图 -> 显示区域 188x120
 *   94x60  压缩图 -> 显示区域 94x60 (居中)
 *   总高度 120 + 10(间隔) + 60 = 190 < 240 屏幕高度
 */
/* ---- 摄像头参数设置 (Camera) ---- */
// TC264: 94列宽, 中线 = 94/2 = 47
#define ImageSensorMid    (LCDW / 2)           // 图像传感器中线位置: 47

// 扫描说明: 从第59~57行预扫, 从第56行开始往下搜5行(56->52)
// 从图像中线(ImageSensorMid=47)向两边搜索, 确定赛道
// 5行全扫一遍, 确定基础边线
#define SCAN_BASE_START_ROW    59              // 扫描起始行 (最底部,距离车最近,数据最可靠,AnCai借鉴)
#define SCAN_BASE_END_ROW      55              // 扫描结束行 (共5行基线: 59,58,57,56,55)
#define SCAN_VALIDATE_COUNT    5               // 验证行数

// 限幅宏, 将L/H限制在[0, LCDW-1]
#define LimitL(L)  ((L) = ((L) < 1)  ? 1  : (L))    // AnCai: L>=1保证p[i-1]不越界
#define LimitH(H)  ((H) = ((H) > (LCDW - 2)) ? (LCDW - 2) : (H))  // AnCai: H<=92保证p[i+1]不越界

/* ---- 图像处理数据结构 ---- */
typedef struct {
    uint8 IsRightFind;
    uint8 IsLeftFind;
    int   Wide;
    int   LeftBorder;
    int   RightBorder;
    int   Center;
} ImageDealDatatypedef;


/* ---- 全局图像数组 ---- */
extern uint8  Pixle[LCDH][LCDW];                // 二值图 (0=黑/背景, 1=白/赛道)
extern uint8 *Image_Use[LCDH][LCDW];            // 压缩后灰度图像指针数组
extern uint8  Camera_Threshold;                 // 当前OTSU阈值 (0~255)

/* ---- 图像数据 ---- */
extern ImageDealDatatypedef ImageDeal[LCDH];   // 每行图像处理结果


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
void  Camera_ShowBinaryFast(void);              // 快速显示二值图 (优化, SPI传输量最小)


/* ---- 图像数据 ---- */
void  Camera_ShowElementStatus(void);            // 显示当前元素状态(缩写标识)
void  Get_BaseLine(void);                       // 获取基准线: 从56->52, 5行
// 搜索区间: 在当前行上一行边线位置+/-ImageScanInterval范围内
#define ImageScanInterval  5                   // 搜索区间(像素)

/* ---- 跳变点结构 ---- */
typedef struct {
    int   point;                               // 跳变点坐标
    uint8 type;                                // 类型: 'T'=跳变, 'W'=全白丢线, 'H'=全黑
} JumpPointtypedef;

/* ---- 图像处理数据结构 ---- */
typedef struct {
    int16 OFFLine;                             // 丢线行: 从该行开始无赛道
    int16 Miss_Left_lines;                     // 左侧连续丢失行数
    int16 Miss_Right_lines;                    // 右侧连续丢失行数
    int16 WhiteLine;                           /* 白色行计数(十字) */
    int16 OFFLineBoundary;                     /* 丢线边界 */
    int16 Det_True;                            /* 有效检测标志 */
    int16 WhiteLine_L;                         /* 左侧白行 */
    int16 WhiteLine_R;                         /* 右侧白行 */
} ImageStatustypedef;

extern ImageStatustypedef ImageStatus;         // 图像状态全局变量

/* 圆环只按七个阶段单向推进，方向由image_element_rings单独保存。 */
#define RING_STATE_IDLE       0
#define RING_STATE_CONFIRM    1
#define RING_STATE_APPROACH   2
#define RING_STATE_ENTRY      3
#define RING_STATE_INSIDE     4
#define RING_STATE_EXIT       5
#define RING_STATE_RECOVERY   6

/* 连续帧与超时参数均为纯视觉保护，防止单帧误判和状态卡死。 */
#define RING_CONFIRM_FRAMES       3U
#define RING_EXIT_CONFIRM_FRAMES  2U
#define RING_EXIT_STABLE_FRAMES   8U
#define RING_RECOVERY_FRAMES      12U
#define RING_CONFIRM_MAX_FRAMES   8U
#define RING_APPROACH_MAX_FRAMES  24U
#define RING_ENTRY_MAX_FRAMES     30U
#define RING_INSIDE_MAX_FRAMES    90U
#define RING_EXIT_MAX_FRAMES      60U
#define RING_RECOVERY_MAX_FRAMES  40U

/* 94x60图像的初始标定值，实车只需调整这些参数。 */
#define RING_ENTRY_CORNER_ROW        38
#define RING_INSIDE_CORNER_ROW       48
#define RING_EXIT_MISS_MIN            8
#define RING_APPROACH_CENTER_OFFSET   4
#define RING_ENTRY_CENTER_OFFSET     10
#define RING_INSIDE_CENTER_OFFSET    14
#define RING_EXIT_CENTER_OFFSET       8
#define RING_RECOVERY_CENTER_OFFSET   4

/* ---- 图像处理数据结构 ---- */
void  Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q);
                                               // 获取跳变点与边线类型
void  Get_AllLine(void);                       // 全行扫描: 从51行向下扫到0

/* ---- 图像标志结构 ---- */
typedef struct {
    int16 Bend_Road;                           /* 弯道: 0=直道 1=左弯 2=右弯 */
    int16 image_element_rings;                 /* 圆环: 0=无 1=左圆环 2=右圆环 */
    int16 ring_big_small;                      /* 圆环大小: 0=无 1=大环 2=小环 */
    int16 image_element_rings_flag;            /* 圆环处理标志 */
    int16 straight_long;                       /* 长直道标志 */
    int16 straight_xie;                        /* 斜入直道标志 */
    int16 Zebra_Flag;                          /* 斑马线: 0=无 1=左侧 2=右侧 */
    int16 Ramp;                                /* 坡道: 0=无 1=检测到 */
    int16 Out_Road;                            /* 断路: 0=无 1=断路 */
} ImageFlagtypedef;

/* ---- 图像处理数据结构 ---- */
// 其他元素: WhiteLine(十字), OFFLineBoundary(丢线边界), Det_True(有效检测)
// 状态: OFFLine/Miss_Left_lines/Miss_Right_lines 在ImageStatus中

extern ImageFlagtypedef ImageFlag;             /* 图像标志全局变量 */
extern int16_t g_ZebraSum;          /* 斑马线检测差值之和，供屏幕显示 */

/* ---- 道路宽度常量 (TC264列宽94, AnCai原版x1.175倍映射) ---- */
extern const uint8 Half_Road_Wide[60];         /* 半道路宽度: 近景~远景 */
extern const uint8 Half_Bend_Wide[60];         /* 弯道半宽 */

/* ---- 图像处理数据结构 ---- */
float Straight_Judge(uint8 dir, uint8 start, uint8 end);     // 直道判断(S<1为直道)
void  Straight_long_judge(void);                             // 长直道判断
void  Straight_long_handle(void);                            // 长直道处理
void  Straight_xie_judge(void);                              // 斜入直道判断
void  Element_Judgment_Bend(void);                           // 弯道识别
void  Element_Handle_Bend(void);                             // 弯道处理
void  Element_Judgment_Left_Rings(void);                     // 左圆环识别
void  Element_Handle_Left_Rings(void);                       // 左圆环处理
void  Element_Judgment_Right_Rings(void);                    // 右圆环识别
void  Element_Handle_Right_Rings(void);                      // 右圆环处理
void  Element_Judgment_Zebra(void);                          // 斑马线识别
void  Element_Handle_Zebra(void);                            // 斑马线处理
void  Element_Judgment_Ramp(void);                           // 坡道识别
void  Element_Handle_Ramp(void);                             // 坡道处理
void  Element_Judgment_OutRoad(void);                        // 断路识别
void  Element_Handle_OutRoad(void);                          // 断路处理
void  Get_ExtensionLine(void);                               // 十字补线
void  Scan_Element(void);                                    // 元素扫描入口
void  Element_Handle(void);                                  // 元素处理入口
void  Flag_init(void);                                       // 标志初始化

void  Camera_ShowElementStatus(void);            // 显示当前元素状态(缩写标识)

#endif

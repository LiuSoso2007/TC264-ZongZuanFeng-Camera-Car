#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ips200.h"

/*
 * Camera.h --- MT9V03X 摄像头采集 + 图像压缩 + OTSU二值化 + IPS200屏幕显示
 *
 * 基于 zf_device_mt9v03x 库, 扩展了:
 *   1. 摄像头初始化 (UART 配置 + ERU 外部中断 + DMA 乒乓传输)
 *   2. 图像压缩 - 将 188x120 原始灰度图等比压缩为 94x60
 *   3. OTSU 大津法二值化 - 自适应光照计算阈值, 灰度转二值
 *   4. IPS200屏幕显示 - 原始图+压缩图+二值图同屏显示
 *
 * 高级图像处理 (环岛/弯道/元素识别) 在 Camera.c 中实现
 *
 * 硬件接线:
 *   TXD   -> P02_3 (UART1 RX)        VCC  -> 3.3V
 *   RXD   -> P02_2 (UART1 TX)        GND  -> GND
 *   PCLK  -> P02_1 (ERU_CH2)         数据同步时钟
 *   VSY   -> P02_0 (ERU_CH3)
 *   D0-D7 -> P00_0 ~ P00_7
 */

/* ---- 原始图像尺寸 (不可修改) ---- */
#define CAMERA_W       MT9V03X_W        // 188 列
#define CAMERA_H       MT9V03X_H        // 120 行

/* ---- 压缩后图像尺寸 (2:1 等比压缩) ---- */
#define LCDW           94               // 压缩后宽 (列) = 188/2
#define LCDH           60               // 压缩后高 (行) = 120/2

/* ---- OTSU阈值相关 (防止低光照/高光照异常) ---- */
#define OTSU_MIN       30               // 最小阈值 (低光照保护)
#define OTSU_MAX       220              // 最大阈值 (避免全白异常)
#define OTSU_BIAS      20               // 阈值偏置, 最终threshold=clamp(otsu)+bias

/*
 * 屏幕布局说明:
 *   188x120 原始图 -> 显示在 (0,0) ~ (188,120)
 *   94x60  压缩图 -> 显示在 (47,150) ~ (47+94, 150+60) 偏右居中
 *   总高度 120 + 10(间隙) + 60 = 190 < 240 屏幕高度
 */
/* ---- 摄像头图像参数 (Camera) ---- */
// TC264: 94列宽, 图像中线 = 94/2 = 47
#define ImageSensorMid    (LCDW / 2)           // 图像传感器中线位置: 47

// 扫描说明: 靠近车体 59~57 行预扫, 从 56 行开始真正巡线 5 行 (56->52)
// 从图像中线 (ImageSensorMid=47) 向外搜索, 确保找到边界
// 5 行全扫一次, 确保基线稳定
#define SCAN_BASE_START_ROW    59              // 扫描起始行 (靠近车体最近, 参考安财代码)
#define SCAN_BASE_END_ROW      55              // 扫描结束行 (共5行基线: 59,58,57,56,55)

// 限幅宏, 保证L/H在有效索引范围内
#define LimitL(L)  ((L) = ((L) < 1)  ? 1  : (L))        // L>=1 保证p[i-1]不越界
#define LimitH(H)  ((H) = ((H) > (LCDW - 2)) ? (LCDW - 2) : (H))  // H<=92 保证p[i+1]不越界

/* ---- 图像行数据结构 ---- */
typedef struct {
    uint8 IsRightFind;    // 右边界找到标志: 'T'=找到 'F'=丢失 'W'=全白
    uint8 IsLeftFind;     // 左边界找到标志: 'T'=找到 'F'=丢失 'W'=全白
    int   Wide;           // 赛道宽度 = RightBorder - LeftBorder
    int   LeftBorder;     // 左边界列坐标
    int   RightBorder;    // 右边界列坐标
    int   Center;         // 赛道中线 = (LeftBorder + RightBorder) / 2
} ImageDealDatatypedef;


/* ---- 全局图像数据 ---- */
extern uint8  Pixle[LCDH][LCDW];                // 二值图 (0=黑/赛道, 1=白/背景)
extern uint8 *Image_Use[LCDH][LCDW];            // 压缩后的像素指针索引
extern uint8  Camera_Threshold;                 // 当前OTSU阈值 (0~255)

/* ---- 图像行处理结果 ---- */
extern ImageDealDatatypedef ImageDeal[LCDH];   // 每行图像处理结果数组


/* ---- 初始化 ---- */
void Camera_Init(void);
void Camera_CompressInit(void);                  // 图像压缩初始化 (调用一次)

/* ---- 图像采集 ---- */
uint8 Camera_IsFrameReady(void);                 // 查询 mt9v03x_finish_flag 标志位
uint8 (*Camera_GetImage(void))[CAMERA_W];        // 返回 mt9v03x_image 原始图指针

/* ---- 图像处理 ---- */
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row);
                                                 // 基于灰度直方图计算OTSU阈值
void  Camera_GetBinaryImage(void);               // 灰度图 -> 二值图 (自适应OTSU)

/* ---- IPS200屏幕显示 ---- */
void  Camera_ShowDebug(void);                    // IPS200 显示原始图+压缩图+二值
void  Camera_ShowBinaryImage(void);              // 快速显示二值图 (轻量版, SPI传输量小)


/* ---- 图像巡线 ---- */
void  Camera_ShowElementStatus(void);            // 显示当前元素状态(调试用)
void  Get_BaseLine(void);                       // 获取基准线: 从56->52, 5行
// 巡线扫描区间: 以上一行边界位置 +/- ImageScanInterval 范围内
#define ZEBRA_SCAN_LEFT            17
#define ZEBRA_SCAN_RIGHT           77

#define ImageScanInterval  5                   // 扫描搜索区间(像素)

/* ---- 跳变点结构 ---- */
typedef struct {
    int   point;                               // 跳变点位置(列坐标)
    uint8 type;                                // 跳变类型: 'T'=黑白跳变, 'W'=全白条, 'H'=全黑
} JumpPointtypedef;

/* ---- 图像状态结构 ---- */
typedef struct {
    int16 OFFLine;                             // 丢线行: 向上开始丢线的行号
    int16 Miss_Left_lines;                     // 左边界连续丢失行数
    int16 Miss_Right_lines;                    // 右边界连续丢失行数
    int16 WhiteLine;                           // 白色行数(十字路口判定)
    int16 OFFLineBoundary;                     // 丢线边界
    int16 Det_True;                            // 有效检测标志
    int16 WhiteLine_L;                         // 左侧白行
    int16 WhiteLine_R;                         // 右侧白行
} ImageStatustypedef;

extern ImageStatustypedef ImageStatus;         // 图像状态全局变量

/* 圆环只在指定阶段调用补线, 其余阶段由 image_element_rings 标志控制。 */

/* ---- 圆环黑洞法参数 ---- */
#define IMG_BLACK                   0
#define IMG_WHITE                   1
#define BH_BOTTOM_START_ROW        52   // 黑洞检测起始行
#define BH_LEFT_COL_MIN             1   // 左黑洞区扫描左边界
#define BH_LEFT_COL_MAX            12   // 左黑洞区扫描右边界
#define BH_RIGHT_COL_MIN           82   // 右黑洞区扫描左边界
#define BH_RIGHT_COL_MAX           93   // 右黑洞区扫描右边界
#define VALLEY_SCAN_START_ROW      59   // 谷底扫描起始行
#define VALLEY_SCAN_COL_LEFT       10   // 谷底扫描列左边界
#define VALLEY_SCAN_COL_RIGHT      75   // 谷底扫描列右边界
#define VALLEY_MAX_ROW             59   // 谷底扫描最大行
#define VALLEY_MIN_ROW             28   // 谷底扫描最小行
#define EXIT_LOST_MIN               8   // 出环时最小丢线数
#define FILL_ENTRY_OFFSET          10   // 入环补线偏移
#define FILL_INSIDE_OFFSET         14
#define FILL_EXIT1_OFFSET          8
#define FILL_EXIT2_OFFSET          6    // 环中补线偏移
#define FILL_RECOVERY_OFFSET        0   // 恢复补线偏移

#define RING_JUMP_THRESHOLD         1   // 断点判定: 连续跳变像素阈值
#define RING_JUMP_SCAN_MIN_ROW     7    // 断点统计最小行(含)
#define RING_JUMP_SCAN_MAX_ROW     40   // 断点统计最大行(含)
#define RING_HOMESIDE_MIN_ROW      7    // 目标侧丢线统计最小行(含)
#define RING_HOMESIDE_MAX_ROW      30   // 目标侧丢线统计最大行(含)
#define RING_HOMESIDE_LOST_THRESH  17   // 目标侧丢线最少行数
#if (RING_JUMP_SCAN_MIN_ROW < 0) || (RING_JUMP_SCAN_MAX_ROW >= LCDH) || (RING_JUMP_SCAN_MIN_ROW > RING_JUMP_SCAN_MAX_ROW)
#error "RING_JUMP_SCAN_ROW range is invalid"
#endif
#define RING_JUMP_MIN_COUNT         2   // 最小跳变次数
#define RING_JUMP_OTHER_MAX         0   // 另一侧最大跳变次数
#define RING_EXIT_POINT_HOLD_FRAMES 1U  // EXIT1/EXIT2单点短时丢失保持帧数
#define RING_EXIT2_PASS_ROW         40  // EXIT2下移到该行后判定已经通过黄色点
#define RING_RECOVERY_SCAN_MIN_ROW  3   //
#define RING_RECOVERY_SCAN_MAX_ROW  59  //
#define RING_RECOVERY_STREAK_MIN    10  // 恢复阶段同列的最小行数

/* ---- 圆环状态机 ---- */
#define RING_STATE_IDLE       0    // 空闲: 无环
#define RING_STATE_CONFIRM    1    // 确认: 跳变点检测到环
#define RING_STATE_APPROACH   2    // 接近: 向环入口靠近
#define RING_STATE_ENTRY      3    // 入环: 进入环形赛道
#define RING_STATE_INSIDE     4    // 环中: 在环形赛道内部
#define RING_STATE_EXIT1      5    // 出环第一阶段: 连接EXIT1与EXIT2
#define RING_STATE_EXIT2      6    // 出环第二阶段: 图像底部连接EXIT2
#define RING_STATE_RECOVERY   7    // 恢复: 跟踪出环拐点3

/* ---- 圆环阶段帧计数 ---- */
#define RING_CONFIRM_FRAMES       2U    // 确认阶段最小帧数
#define RING_EXIT_CONFIRM_FRAMES  2U    // 出环确认最小帧数
#define RING_CONFIRM_MAX_FRAMES   300U   // 确认阶段超时帧数
#define RING_ENTRY_MAX_FRAMES     2000U   // 入环阶段超时帧数
#define RING_INSIDE_MAX_FRAMES    2000U
#define RING_EXIT1_MAX_FRAMES     45U
#define RING_RECOVERY_ACQUIRE_MAX_FRAMES 10U // RECOVERY首次找点最多等待帧数

void  Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q);
                                               // 获取边界跳变点位置和类型
void  Get_AllLine(void);                       // 全图扫描: 从51行向上扫描到0

/* ---- 图像标志结构 ---- */
typedef struct {
    int16 Bend_Road;                           /* 弯道: 0=直道 1=左弯 2=右弯 */
    int16 image_element_rings;                 /* 圆环: 0=无 1=左圆环 2=右圆环 */
    int16 ring_big_small;                      /* 圆环大小: 0=无 1=大环 2=小环 */
    int16 image_element_rings_flag;            /* 圆环状态标志 */
    int16 straight_long;                       /* 长直道标志 */
    int16 straight_xie;                        /* 斜入直道标志 */
    int16 Zebra_Flag;                          /* 斑马线: 0=无 1=左侧 2=右侧 */
    int16 Ramp;                                /* 坡道: 0=无 1=检测到 */

} ImageFlagtypedef;

/* ---- 图像标志结构说明 ---- */
// 已移除的元素字段: WhiteLine(十字), OFFLineBoundary(丢线边界), Det_True(有效检测)
// 状态: OFFLine/Miss_Left_lines/Miss_Right_lines 在ImageStatus结构体中

extern ImageFlagtypedef ImageFlag;             /* 图像标志全局变量 */
extern int16_t g_ZebraSum;                     /* 斑马线检测有效行数, 用于屏幕显示 */
extern volatile int g_corner_black_max;        /* 调试: 拐角处最大黑宽度 */
extern volatile int g_bottom_black_width;      /* 调试: W-B底部黑宽度 */
extern volatile int g_ring_miss_cnt;           /* 调试: 圆环丢线计数(Miss_Left/Miss_Right) */
extern volatile uint8 g_left_jump_count;       /* 调试: 左边界跳变计数 */
extern volatile uint8 g_right_jump_count;      /* 调试: 右边界跳变计数 */
extern volatile int g_approach_valley_row;     /* 调试: APPROACH阶段谷底行 */
extern volatile uint8 g_edge_squeezed_dbg;     /* 调试: 挤压状态 */
extern volatile uint8 g_ring_phase_dbg;        /* 调试: 谷底阶段 0/1/2 */

/* ---- 道路半宽查找表 (TC264列宽94, 安财原版x1.175映射) ---- */
extern const uint8 Half_Road_Wide[60];         /* 半道路宽: 近处~远处 */
extern const uint8 Half_Bend_Wide[60];         /* 弯道半宽 */

/* ---- 元素处理函数 ---- */
float Straight_Judge(uint8 dir, uint8 start, uint8 end);     // 直道判定(S<1为直道)
void  Straight_long_judge(void);                             // 长直道判定
void  Straight_long_handle(void);                            // 长直道处理
void  Straight_xie_judge(void);                              // 斜入直道判定
void  Element_Judgment_Bend(void);                           // 弯道判断
void  Element_Handle_Bend(void);                             // 弯道处理
void  Element_Judgment_Left_Rings(void);                     // 左圆环识别
void  Element_Handle_Left_Rings(void);                       // 左圆环处理
void  Element_Judgment_Right_Rings(void);                    // 右圆环识别
void  Element_Handle_Right_Rings(void);                      // 右圆环处理
uint8 Ring_Should_Hold_Err(void);                            // EXIT2跳变帧保持上一帧Err
void  Element_Judgment_Zebra(void);                          // 斑马线识别
void  Element_Handle_Zebra(void);                            // 斑马线处理
void  Element_Judgment_Ramp(void);                           // 坡道识别
void  Element_Handle_Ramp(void);                             // 坡道处理


void  Get_ExtensionLine(void);                               // 十字补线
void  Scan_Element(void);                                    // 元素扫描与判定
void  Element_Handle(void);                                  // 元素处理与补线
void  Flag_init(void);                                       // 标志初始化

void  Camera_ShowElementStatus(void);                        // 显示当前元素状态(调试用)

#endif

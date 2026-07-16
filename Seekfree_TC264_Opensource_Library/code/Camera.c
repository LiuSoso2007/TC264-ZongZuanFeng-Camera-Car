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
ImageDealDatatypedef ImageDeal[LCDH];        // 图像处理数据结构 (每行一条)
ImageStatustypedef ImageStatus;              // 图像状态(OFFLine/丢线等)
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
    uint32 hist[256] = {0};
    uint16 i, j;
    uint16 t;
    uint32 total = (uint32)col * row;
    uint64 totalSum = 0;
    uint32 w0 = 0;
    uint64 sum0 = 0;
    float maxVar = 0.0f;
    uint8 bestThr = 128;

    /* 第二步: 计算灰度总和 */
    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++)
            hist[*image[i][j]]++;

    /* 第二步: 计算灰度总和 */
    for (t = 0; t < 256; t++)
        totalSum += (uint64)t * hist[t];

    /* 第三步: 遍历阈值, 寻找最大类间方差 (不提前退出, 全遍历) */
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

    /* 第四步: 阈值限幅, 防止过暗/过曝导致异常 */
    if (bestThr < OTSU_MIN) bestThr = OTSU_MIN;
    if (bestThr > OTSU_MAX) bestThr = OTSU_MAX;
    return bestThr;
}

/*
 * Camera_GetBinaryImage - 灰度图二值化
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
    /* legend removed */
    Camera_ShowElementStatus();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


//-------------------------------------------------------------------------------
//  @brief          Get_BaseLine - 获取赛道基准线
//  说明: 从第59-57行预扫, 从第56行开始往下搜5行(56->52)
//  从图像中线(ImageSensorMid=47)向两边搜, 找到从赛道到背景的跳变
//  5行全扫一遍, 确定基础边线
//  输入 Pixle[][] 二值图像 (0=黑/背景, 1=白/赛道)
//-------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    uint8 *PicTemp;                             // 当前行像素指针
    int   Xsite;                                // 列扫描位置
    int   row;                                  // 当前扫描行号

    /* ---- 第1步: 从第56行开始扫 (基准行) ---- */
    PicTemp = Pixle[SCAN_BASE_START_ROW];       // 从第56行开始

    // 从中线向右侧搜索, 找白到黑的跳变
    for (Xsite = ImageSensorMid; Xsite < (LCDW - 1); Xsite++)
    {
        if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 0)
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

    // 从中线向右侧搜索, 找白到黑的跳变
    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)
    {
        if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 0)
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

    // 第56行用找边结果计算中心线
    ImageDeal[SCAN_BASE_START_ROW].Center
        = (ImageDeal[SCAN_BASE_START_ROW].LeftBorder
         + ImageDeal[SCAN_BASE_START_ROW].RightBorder) / 2;
    ImageDeal[SCAN_BASE_START_ROW].Wide
        = ImageDeal[SCAN_BASE_START_ROW].RightBorder
        - ImageDeal[SCAN_BASE_START_ROW].LeftBorder;
    /* 若之前未标记为'F', 则标记为'T' */
    if (ImageDeal[SCAN_BASE_START_ROW].IsLeftFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsLeftFind  = 'T';
    if (ImageDeal[SCAN_BASE_START_ROW].IsRightFind != 'F')
        ImageDeal[SCAN_BASE_START_ROW].IsRightFind = 'T';

    /* ---- 第2步: 继续向下扫描55->52行 ---- */
    for (row = SCAN_BASE_START_ROW - 1; row >= SCAN_BASE_END_ROW; row--)
    {
        PicTemp = Pixle[row];

        // 标记为找到跳变
        for (Xsite = ImageDeal[row + 1].Center; Xsite < (LCDW - 1); Xsite++)
        {
            if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 0)
            {
                ImageDeal[row].RightBorder = Xsite;
                break;
            }
            else if (Xsite == (LCDW - 2))
            {
                ImageDeal[row].RightBorder = LCDW - 1;
                ImageDeal[row].IsRightFind = 'F';   // 右侧未找到边线
                break;
            }
        }

        // 标记为找到跳变
        for (Xsite = ImageDeal[row + 1].Center; Xsite > 0; Xsite--)
        {
            if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 0)
            {
                ImageDeal[row].LeftBorder = Xsite;
                break;
            }
            else if (Xsite == 1)
            {
                ImageDeal[row].LeftBorder = 0;
                ImageDeal[row].IsLeftFind = 'F';    // 左侧未找到边线
                break;
            }
        }

        // 更新本行中心线位置
        ImageDeal[row].Center
            = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide
            = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        /* 若之前未标记为'F', 则标记为'T' */
        if (ImageDeal[row].IsLeftFind != 'F')
            ImageDeal[row].IsLeftFind  = 'T';
        if (ImageDeal[row].IsRightFind != 'F')
            ImageDeal[row].IsRightFind = 'T';
    }

    /* ---- 第3步: 5行扫描完成 (基础边线) ---- */
    // TODO: 此处可添加基础边线有效性校验逻辑
}

//-------------------------------------------------------------------------------
//  @brief          Get_Border_And_SideType - 获取跳变点与边线类型
//  在[L, H]范围内, 从指定方向搜索白到黑的跳变点
//  类型: 'T'=找到跳变, 'W'=整行白(丢线), 'H'=整行黑(断路)
//  @parameter      p    当前行像素数据指针
//  @parameter      type 搜索方向: 'L'=向左搜索, 'R'=向右搜索
//  @parameter      L, H 搜索范围边界
//  @parameter      Q    跳变点结果结构体
//  @return         void
//  Sample usage:   Get_Border_And_SideType(PicTemp, 'R', low, high, &jp);
//-------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p, uint8 type, int L, int H, JumpPointtypedef* Q)
{
    int i;
    /* ---- 安全校验: 将L/H限制在合法范围[0, LCDW-1] ---- */
    LimitL(L);
    LimitH(H);

    if (type == 'L')                            // 向左搜索: 从右往左扫描
    {
        for (i = H; i >= L; i--)
        {
            // 白(1)->黑(0)跳变: 找到边线, 返回位置
            if (*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;                   // 记录跳变点列坐标
                Q->type  = 'T';                 // 标记为找到跳变
                break;
            }
            else if (i == (L + 1))              // 扫描到底仍未找到跳变
            {
                if (*(p + (L + H) / 2) != 0)    // 图像处理数据结构 (每行一条)
                {
                    Q->point = (L + H) / 2;     // 列扫描位置
                    Q->type  = 'W';             // 整行白(丢失边线)
                }
                else                            // 图像处理数据结构 (每行一条)
                {
                    Q->point = H;               // 列扫描位置
                    Q->type  = 'H';             // 标记为找到跳变
                }
                break;
            }
        }
    }
    else if (type == 'R')                       // 向右搜索: 从左往右扫描
    {
        for (i = L; i <= H; i++)
        {
            // 白(1)->黑(0)跳变: 找到边线, 返回位置
            if (*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;                   // 记录跳变点列坐标
                Q->type  = 'T';                 // 标记为找到跳变
                break;
            }
            else if (i == (H - 1))              // 扫描到底仍未找到跳变
            {
                if (*(p + (L + H) / 2) != 0)    // 图像处理数据结构 (每行一条)
                {
                    Q->point = (L + H) / 2;     // 列扫描位置
                    Q->type  = 'W';             // 整行白(丢失边线)
                }
                else                            // 图像处理数据结构 (每行一条)
                {
                    Q->point = L;               // 列扫描位置
                    Q->type  = 'H';             // 标记为找到跳变
                }
                break;
            }
        }
    }
}


//-------------------------------------------------------------------------------
//  @brief          Get_AllLine - 从基准线向下扫描全部赛道边线
//  在Get_BaseLine(56->52)之后, 从51行开始利用52行结果向下递推扫描到0行
//  搜索策略: 在当前行上一行边线位置+/-ImageScanInterval范围内搜索跳变
//  异常处理: 连续多行找不到边线则触发丢线; 整行白触发OFFLine
//  @parameter      void
//  @return         void
//  @note           输入 ImageDeal[52] (基准线数据) 和 Pixle[][] (二值图像)
//  @note           OFFLine: 连续多行左右同时丢线判断为车辆偏离赛道
//  Sample usage:   Get_AllLine();
//-------------------------------------------------------------------------------
void Get_AllLine(void)
{
    uint8 *PicTemp;                             // 当前行像素指针
    int   row;                                  // 当前扫描行号
    int   IntervalLow, IntervalHigh;            // 左右搜索区间边界
    int   i;                                    // 标记为找到跳变

    /* ---- 初始化状态变量 ---- */
    ImageStatus.OFFLine          = 2;           // 丢线行号(初始为2)
    ImageStatus.Miss_Left_lines  = 0;           // 列扫描位置
    ImageStatus.Miss_Right_lines = 0;           // 列扫描位置
    ImageStatus.WhiteLine        = 0;           // 图像处理数据结构 (每行一条)
    ImageStatus.WhiteLine_L      = 0;           // 列扫描位置
    ImageStatus.WhiteLine_R      = 0;           // 列扫描位置
    ImageStatus.OFFLineBoundary  = 0;           // 列扫描位置
    ImageStatus.Det_True         = 0;           // 图像处理数据结构 (每行一条)

    /*
     * 从51行开始, 以52行(基准线)为参考向下扫描
     * 逐行递推搜索直到0行或触发OFFLine丢线
     */
    for (row = SCAN_BASE_END_ROW - 1; row > ImageStatus.OFFLine; row--)
    {
        JumpPointtypedef JumpPoint[2];          // [0]=?, [1]=?
        PicTemp = Pixle[row];

        /* ============================================================
         * 右侧搜索: 在上一行右边界 +/- ImageScanInterval 范围内扫描
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].RightBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].RightBorder + ImageScanInterval;
        LimitL(IntervalLow);                    // 限幅到[0, 93]
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /* ============================================================
         * 右侧搜索: 在上一行右边界 +/- ImageScanInterval 范围内扫描
         * ============================================================ */
        IntervalLow  = ImageDeal[row + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[row + 1].LeftBorder + ImageScanInterval;
        LimitL(IntervalLow);
        LimitH(IntervalHigh);

        Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        /* ============================================================
         * 根据跳变类型进行边线处理:
         * 'T'=跳变: 更新为找到的边线位置
         * 'W'=全白: 使用上一行边线值 (补线+1)
         * 'H'=全黑: 扫描区域内没有白点, 触发丢线
         * ============================================================ */
        if (JumpPoint[0].type == 'W')           // 图像处理数据结构 (每行一条)
        {
            ImageDeal[row].LeftBorder = ImageDeal[row + 1].LeftBorder;  // 标记为找到跳变
            ImageStatus.Miss_Left_lines++;      // 列扫描位置
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].LeftBorder = JumpPoint[0].point;
            ImageStatus.Miss_Left_lines = 0;    // 找到边线, 清零丢失计数
        }

        if (JumpPoint[1].type == 'W')           // 图像处理数据结构 (每行一条)
        {
            ImageDeal[row].RightBorder = ImageDeal[row + 1].RightBorder; // 标记为找到跳变
            ImageStatus.Miss_Right_lines++;     // 列扫描位置
        }
        else                                    // 'T' ? 'H'
        {
            ImageDeal[row].RightBorder = JumpPoint[1].point;
            ImageStatus.Miss_Right_lines = 0;   // 找到边线, 清零丢失计数
        }

        /* ---- 记录找到状态 ---- */
        ImageDeal[row].IsLeftFind  = JumpPoint[0].type;
        ImageDeal[row].IsRightFind = JumpPoint[1].type;

        /* ---- 白行计数(左右同时为白) ---- */
        if (JumpPoint[0].type == 'W' && JumpPoint[1].type == 'W')
        {
            ImageStatus.WhiteLine++;            // 图像处理数据结构 (每行一条)
        }
        else
        {
            if (ImageStatus.WhiteLine > 0) ImageStatus.WhiteLine--;
        }
        /* 单侧白行计数 */
        if (JumpPoint[0].type == 'W')
            ImageStatus.WhiteLine_L++;
        else
            ImageStatus.WhiteLine_L = 0;
        if (JumpPoint[1].type == 'W')
            ImageStatus.WhiteLine_R++;
        else
            ImageStatus.WhiteLine_R = 0;

        /* ---- 计算本行中心与宽度 ---- */
        ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
        ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;

        /*
         * H类型修复: 全黑行尝试向内收缩重新找边
         * 当某行为全黑H时, 从边界向内侧搜索白变黑点
         */
        if (ImageDeal[row].IsLeftFind == 'H' || ImageDeal[row].IsRightFind == 'H')
        {
            /* ---- 左H型: 从左边界+1向内侧搜索最近的白变黑点 ---- */
            if (ImageDeal[row].IsLeftFind == 'H')
            {
                for (i = ImageDeal[row].LeftBorder + 1; i <= ImageDeal[row].RightBorder; i++)
                {
                    if (*(PicTemp + i) == 0)    // 标记为找到跳变
                    {
                        ImageDeal[row].LeftBorder = i;
                        ImageDeal[row].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            /* ---- 右H型: 从右边界-1向内侧搜索最近的白变黑点 ---- */
            if (ImageDeal[row].IsRightFind == 'H')
            {
                for (i = ImageDeal[row].RightBorder - 1; i >= ImageDeal[row].LeftBorder; i--)
                {
                    if (*(PicTemp + i) == 0)    // 标记为找到跳变
                    {
                        ImageDeal[row].RightBorder = i;
                        ImageDeal[row].IsRightFind = 'T';
                        break;
                    }
                }
            }

            /* ---- 修复后重新计算中心 ---- */
            ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;
            ImageDeal[row].Wide   = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
        }

        /* ============================================================
         * OFFLine丢线判断: 左右同时连续丢失超过3行
         * 根据跳变类型进行边线处理:
         * ============================================================ */
        if (ImageStatus.Miss_Left_lines > 3 && ImageStatus.Miss_Right_lines > 3)
        {
            ImageStatus.OFFLine = row;          // 标记为找到跳变
            break;
        }
    }
}



/* ================================================================
 * 元素识别常量区 (TC264: 94列宽, AnCai原版x1.175倍映射)
 * ================================================================ */
const uint8 Half_Road_Wide[60] = {           /* 半道路宽度(近景->远景递减) */
     5, 6, 6, 7, 7, 7, 8, 8, 9, 9,
    11,11,12,12,12,13,14,14,15,15,
    15,16,16,18,18,19,19,20,20,20,
    21,21,22,22,24,24,24,25,25,26,
    27,27,27,28,28,29,29,29,31,31,
    32,33,33,33,34,35,36,36,36,38,
};

const uint8 Half_Bend_Wide[60] = {           /* 弯道半宽补偿 */
    39,39,39,39,39,39,39,39,39,39,
    39,39,38,38,35,35,34,34,33,32,
    33,32,32,31,31,29,29,28,28,27,
    26,25,25,26,26,26,27,28,28,28,
    29,29,29,31,31,31,32,32,33,33,
    33,34,34,35,35,36,36,38,38,39,
};

/* ================================================================
 * 图像元素标志
 * ================================================================ */
ImageFlagtypedef ImageFlag;                  /* 弯道半宽补偿 */

/* ================================================================
 * Helper: Straight_Judge - 直道判别
 * dir=1: 用左边界拟合, dir=2: 用右边界拟合
 * 返回均方差S, S<1 视为直道
 * ================================================================ */
float Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    float S = 0.0f, Sum = 0.0f, Err = 0.0f, k = 0.0f;
    switch (dir)
    {
    case 1: /* 左边界 */
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
    case 2: /* 右边界 */
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
 * 长直道判断与处理
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
 * 斜入直道判断 (误判弯道矫正)
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
 * 弯道识别 (左右边线+补线)
 * ================================================================ */
void Element_Judgment_Bend(void)
{
    if (ImageFlag.image_element_rings != 0 || ImageStatus.OFFLine < 3   /* ponytail: TC264行号48-44, OFFLine阈值调为2 */
        || ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1)
        return;

    /* 左弯: 左边线靠右(>30), 左侧未丢失, 右侧丢失多行 */
    if (ImageDeal[ImageStatus.OFFLine + 1].LeftBorder > 35  /* ponytail: 30*94/80=35, 80列->94列映射 */
     && ImageStatus.Miss_Left_lines < 4
     && ImageStatus.Miss_Right_lines > 8
     && Straight_Judge(1, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 1.0f)
    {
        ImageFlag.Bend_Road = 1;              /* 左弯 */
    }

    /* 右弯: 右边线靠左(<50), 右侧未丢失, 左侧丢失多行 */
    if (ImageDeal[ImageStatus.OFFLine + 1].RightBorder < 59  /* ponytail: 50*94/80=59, 80列->94列映射 */
     && ImageStatus.Miss_Right_lines < 4
     && ImageStatus.Miss_Left_lines > 8
     && Straight_Judge(2, ImageStatus.OFFLine + 2, SCAN_BASE_START_ROW - 1) > 1.0f)
    {
        ImageFlag.Bend_Road = 2;              /* 右弯 */
    }
}

/* ================================================================
 * 弯道处理: 用道路半宽补全中心线
 * ================================================================ */
void Element_Handle_Bend(void)
{
    int row;                                  /* 用int避免uchar溢出 */
    if (ImageStatus.OFFLine < 3)  { ImageFlag.Bend_Road = 0; return; }  /* ponytail: TC264 OFFLine阈值2 */

    if (ImageFlag.Bend_Road == 1)             /* 左弯: center=左边界+半宽 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);    /* 限幅 <= 93 */
        }
    }
    else if (ImageFlag.Bend_Road == 2)        /* 右弯: center=右边界-半宽 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Bend_Wide[row];
            LimitL(ImageDeal[row].Center);    /* 限幅 >= 0 */
        }
    }
}

/* ================================================================
 * 左圆环识别
 * ================================================================ */
void Element_Judgment_Left_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Left_Less_Num = 0;

    if (ImageStatus.Miss_Right_lines > 3 || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 2 || Straight_Judge(2, 5, SCAN_BASE_END_ROW) > 1.0f   /* ponytail: TC264 OFFLine阈值2 */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    /* 检查是否所有行的左边线都是'W'(全白) */
    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: 适配TC264行范围48->44 */
        {
            if (ImageDeal[r].IsLeftFind == 'W') return;
        }
    }

    /* 搜索左边线大幅外扩的行 */
    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 1].LeftBorder > 4)
        {
            Left_Less_Num++;
            /* 累计左边线外扩次数 */
            if (Left_Less_Num == 1) {
                /* 第一次外扩时, 可在此记录起始行号 */
            }
        }
    }

    if (Left_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 1;    /* 左圆环 */
        ImageFlag.image_element_rings_flag = 1;
    }
}

/* ================================================================
 * 右圆环识别 (逻辑对称)
 * ================================================================ */
void Element_Judgment_Right_Rings(void)
{
    int Ysite, ring_ysite = 25;
    int Right_Less_Num = 0;

    if (ImageStatus.Miss_Left_lines > 3 || ImageStatus.Miss_Right_lines < 13
        || ImageStatus.OFFLine > 2 || Straight_Judge(1, 5, SCAN_BASE_END_ROW) > 1.0f   /* ponytail: TC264 OFFLine阈值2 */
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    {
        int r;
        for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)   /* ponytail: 适配TC264行范围 */
        {
            if (ImageDeal[r].IsRightFind == 'W') return;
        }
    }

    for (Ysite = (SCAN_BASE_START_ROW - 1); Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBorder - ImageDeal[Ysite].RightBorder > 4)
        {
            Right_Less_Num++;
        }
    }

    if (Right_Less_Num >= 2)
    {
        ImageFlag.image_element_rings = 2;    /* 右圆环 */
        ImageFlag.image_element_rings_flag = 2;
    }
}

/* ================================================================
 * 左圆环处理: 用道路半宽补线
 * ================================================================ */
void Element_Handle_Left_Rings(void)
{
    int row;

    if (ImageFlag.image_element_rings_flag == 1)
    {
        /* 补线策略: 中心=左边线+弯道半宽 */
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Bend_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }

    /* 退出条件: OFFLine达到阈值 (已过圆环) */
    if (ImageStatus.OFFLine >= 5)     /* ponytail: TC264行范围减小, 阈值调小 */
    {
        ImageFlag.image_element_rings = 0;
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.ring_big_small = 0;
    }
}

/* ================================================================
 * 左圆环识别
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

    if (ImageStatus.OFFLine >= 5)     /* ponytail: TC264行范围减小, 阈值调小 */
    {
        ImageFlag.image_element_rings = 0;
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.ring_big_small = 0;
    }
}

/* ================================================================
 * 斑马线识别: 在行20~32范围内检测跳变密度
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

    if (NUM > 8)                              /* 跳变密度达标: 判定为斑马线 */
    {
        if (ImageDeal[SCAN_BASE_START_ROW].Center > 47)  /* TC264: 图像宽94列中位47 */        /* 中心偏右 -> 左侧斑马 */
            ImageFlag.Zebra_Flag = 1;
        else                                  /* 中心偏右 -> 左侧斑马 */
            ImageFlag.Zebra_Flag = 2;
    }
}

/* ================================================================
 * 斑马线处理: 用道路半宽补全中心线
 * ================================================================ */
void Element_Handle_Zebra(void)
{
    int row;

    if (ImageFlag.Zebra_Flag == 1)            /* 左侧斑马: 用右边线推算 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].RightBorder - Half_Road_Wide[row];
            LimitL(ImageDeal[row].Center);
        }
    }
    else if (ImageFlag.Zebra_Flag == 2)       /* 左侧斑马: 用右边线推算 */
    {
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLineBoundary + 1; row--)
        {
            ImageDeal[row].Center = ImageDeal[row].LeftBorder + Half_Road_Wide[row];
            LimitH(ImageDeal[row].Center);
        }
    }
}

/* ================================================================
 * 坡道识别: OFFLine检测 + 宽度 + 中心检测
 * ================================================================ */
void Element_Judgment_Ramp(void)
{
    return;                              /* ponytail: 坡道暂不启用, 由IMU俯仰角处理 */
    int Ysite;
    int i = 0;                           /* 有效行计数器 */

    if (ImageStatus.WhiteLine >= 3) return;

    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if (ImageDeal[Ysite].Wide > 18
             && ImageDeal[Ysite].IsRightFind == 'T'
             && ImageDeal[Ysite].IsLeftFind == 'T'
             && ImageDeal[Ysite].LeftBorder < 40
             && ImageDeal[Ysite].RightBorder > 55   /* TC264: >55(原>40) */
             && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center - 2] == 1
             && Pixle[Ysite][ImageDeal[Ysite].Center + 2] == 1
             && ImageStatus.Miss_Left_lines < 7
             && ImageStatus.Miss_Right_lines < 7)
            {
                i++;
            }
        }

        if (i >= 3)                           /* 累计3行满足 */
        {
            ImageFlag.Ramp = 1;
        }
    }
}

/* ================================================================
 * 图像元素标志
 * ================================================================ */
void Element_Handle_Ramp(void)
{
    /* 坡道处理暂未实现, 留待IMU方案替代 */
    /* 替代方案: 改用imu俯仰角判断 */
}

/* ================================================================
 * 断路识别: OFFLine检测 + 丢线后重新找到边线
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
 * 断路处理: 检测赛道恢复条件
 * ================================================================ */
void Element_Handle_OutRoad(void)
{
    int Ysite, Xsite;
    int gray_sum = 0;

    /* 检测中央区域(近景)白点数 */
    for (Ysite = 35; Ysite < 55; Ysite++)
    {
        for (Xsite = 30; Xsite < 64; Xsite++) /* TC264图像区域 */
        {
            gray_sum += Pixle[Ysite][Xsite];
        }
    }

    /* 白点足够多 -> 退出断路状态 */
    if (gray_sum > 400 && ImageStatus.OFFLine < 20)
    {
        ImageFlag.Out_Road = 0;
    }
}

/* ================================================================
 * 十字补线: 针对全白行, 用上下有效行推导边线
 * ================================================================ */
void Get_ExtensionLine(void)
{
    int Ysite, TFSite = SCAN_BASE_END_ROW - 1;   /* ponytail: TC264底行 */
    int left_FTSite = 0, right_FTSite = 0;

    if (ImageStatus.WhiteLine < 8) return;

    /* 第二步: 计算灰度总和 */
    for (Ysite = (SCAN_BASE_END_ROW - 2); Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
    {
        if (ImageDeal[Ysite].IsLeftFind == 'W')
        {
            if (ImageDeal[Ysite + 1].LeftBorder >= 82  /* ponytail: 70*94/80=82, 80列->94列映射 */)
            {
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
            /* 单侧白行计数 */
            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].LeftBorder;
        }
    }

    /* 第二步: 计算灰度总和 */
    for (Ysite = (SCAN_BASE_END_ROW - 2); Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
    {
        if (ImageDeal[Ysite].IsRightFind == 'W')
        {
            if (ImageDeal[Ysite + 1].RightBorder <= 23)   /* TC264: 左侧边界 */
            {
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].RightBorder;
        }
    }

    /* 单侧白行计数 */
    for (Ysite = TFSite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder
                                 + ImageDeal[Ysite].RightBorder) / 2;
    }
}

/* ================================================================
 * 元素扫描入口: 调用各元素识别函数
 * 注意: 元素识别有优先级和互斥关系
 * ================================================================ */
void Scan_Element(void)
{
    /* 仅在无其他元素状态下进行元素识别 */
    if (ImageFlag.Out_Road == 0 && ImageFlag.Zebra_Flag == 0
     && ImageFlag.image_element_rings == 0
     && ImageFlag.Ramp == 0 && ImageFlag.Bend_Road == 0
     && ImageFlag.straight_long == 0)
    {
        Element_Judgment_OutRoad();           /* 断路 */
        Element_Judgment_Left_Rings();        /* 左圆环 */
        Element_Judgment_Right_Rings();       /* 右圆环 */
        Element_Judgment_Zebra();             /* 斑马线 */
        Element_Judgment_Bend();              /* 弯道 */
        Element_Judgment_Ramp();              /* 坡道 */
        Straight_long_judge();                /* 长直道 */
    }

    /* 弯道状态下仍检测断路 */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();
        if (ImageFlag.Out_Road) ImageFlag.Bend_Road = 0;
    }

    /* 弯道状态下仍检测斑马线 */
    if (ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();
        if (ImageFlag.Zebra_Flag) ImageFlag.Bend_Road = 0;
    }
}

/* ================================================================
 * 元素处理入口: 根据识别的元素调用对应处理函数
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
        Get_ExtensionLine();                  /* 第二步: 计算灰度总和 */
}

/* ================================================================
 * 左圆环识别
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
//  @brief          Camera_ShowElementStatus - 显示当前元素状态
//  在IPS200底部显示当前识别到的赛道元素(缩写标识)
//  缩写: zhi=直道 wan_L/R=弯道 shi=十字 huan_L/R=圆环 banma=斑马 po=坡道 duan=断路
//  @parameter      void
//  @return         void
//  Sample usage:   Camera_ShowElementStatus();
//-------------------------------------------------------------------------------
void Camera_ShowElementStatus(void)
{
    /* 底部状态栏: 白字, 蓝底 */
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    /*
     * 互斥显示当前元素, 按优先级从上到下
     * 位置: y=225 (屏幕240高, 底部15px行高)
     */
        if    (ImageFlag.image_element_rings == 1)
    {
        ips200_show_string(2, 225, "ELEM: yuan_L ");     /* 左圆环 */
    }
    else if (ImageFlag.image_element_rings == 2)
    {
        ips200_show_string(2, 225, "ELEM: yuan_R ");     /* 右圆环 */
    }
    else if (ImageFlag.Zebra_Flag == 1)
    {
        ips200_show_string(2, 225, "ELEM: ban_L");     /* 斑马线-左侧 */
    }
    else if (ImageFlag.Zebra_Flag == 2)
    {
        ips200_show_string(2, 225, "ELEM: ban_R");     /* 斑马线-左侧 */
    }
    else if (ImageFlag.Ramp != 0)
    {
        ips200_show_string(2, 225, "ELEM: po     ");     /* 坡道 */
    }
    else if (ImageFlag.Bend_Road == 1)
    {
        ips200_show_string(2, 225, "ELEM: wan_L  ");     /* 左弯 */
    }
    else if (ImageFlag.Bend_Road == 2)
    {
        ips200_show_string(2, 225, "ELEM: wan_R  ");     /* 右弯 */
    }
    else if (ImageFlag.straight_long)
    {
        ips200_show_string(2, 225, "ELEM: zhi    ");     /* 长直道 */
    }
    else if (ImageStatus.WhiteLine >= 8)
    {
        ips200_show_string(2, 225, "ELEM: shi    ");     /* 十字 */
    }
    else
    {
        ips200_show_string(2, 225, "ELEM: ---    ");     /* 无元素 */
    }

    /* 单侧白行计数 */
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}


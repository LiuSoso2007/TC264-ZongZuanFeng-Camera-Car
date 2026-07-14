/*
 * Camera.c --- MT9V03X 摄像头驱动 + 图像压缩 + OTSU二值化 + IPS200调试显示
 *
 * 基于逐飞 zf_device_mt9v03x 库, 提供完整图像处理管线:
 *   1. Camera_Init()              -> mt9v03x_init(), 摄像头硬件初始化
 *   2. Camera_IsFrameReady()      -> 检查 mt9v03x_finish_flag 标志位
 *   3. Camera_GetImage()          -> 返回 mt9v03x_image 原始图像指针
 *   4. Camera_CompressInit()      -> 图像压缩初始化, 将 188x120 映射到 94x60
 *   5. Camera_OTSU_GetThreshold() -> OTSU大津法计算最优阈值
 *   6. Camera_GetBinaryImage()    -> 灰度图二值化处理
 *   7. Camera_ShowDebug()         -> IPS200调试显示: 原始图+压缩图+阈值
 *
 * 严禁修改逐飞 zf_device_mt9v03x 库中的摄像头相关函数。
 * 调用者: CPU0 主循环, 每帧执行一次图像处理。
 */

#include "Camera.h"

/* ---- 全局图像数组定义 ---- */
uint8  Pixle[LCDH][LCDW];                // 二值化图像结果 (0=黑, 1=白)
uint8 *Image_Use[LCDH][LCDW];            // 压缩后灰度图像指针数组
uint8  Camera_Threshold = 128;           // 当前OTSU阈值, 默认中值130附近

/* ---- 压缩步长宏 (编译期常量) ---- */
// 行压缩步长 = 120/60 = 2, 列压缩步长 = 188/94 = 2
#define COMPRESS_STEP_H   (MT9V03X_H / LCDH)    // 行采样步长 = 2
#define COMPRESS_STEP_W   (MT9V03X_W / LCDW)    // 列采样步长 = 2

/*
 * Camera_Init - 摄像头初始化
 *
 * 调用 mt9v03x_init(), 内部完成:
 *   1. UART1 9600bps 摄像头配置通信
 *   2. camera_init() 配置 GPIO/DMA_CH5/ERU_CH3(PCLK+VSYNC)
 *   3. 通过 UART 或 SCCB 配置曝光/FPS/增益
 *   4. 注册 ISR: camera_vsync_handler / camera_dma_handler / camera_uart_handler
 *
 * 前提: 需要先调用 clock_init() 初始化系统时钟
 */
void Camera_Init(void)
{
    system_delay_ms(200);        // 等待摄像头上电稳定
    mt9v03x_init();              // 逐飞库摄像头初始化
}

/*
 * Camera_IsFrameReady - 检测帧就绪标志
 *
 * 读取 mt9v03x_finish_flag 并立即清零, 实现边沿检测.
 *   返回 1: 新的一帧图像已就绪, 存于 mt9v03x_image
 *   返回 0: 无新帧
 *
 * 注意: mt9v03x_finish_flag 由 DMA_CH5 传输完成中断置位
 */
uint8 Camera_IsFrameReady(void)
{
    uint8 flag = mt9v03x_finish_flag;
    mt9v03x_finish_flag = 0;
    return flag;
}

/*
 * Camera_GetImage - 获取原始图像
 *
 * 返回 mt9v03x_image[CAMERA_H][CAMERA_W] 的二维数组指针.
 * 每个像素为 1 字节灰度值 (0~255).
 */
uint8 (*Camera_GetImage(void))[CAMERA_W]
{
    return mt9v03x_image;
}

//-------------------------------------------------------------------------------
//  @brief          Camera_CompressInit - 图像压缩初始化
//  @brief          建立Image_Use指针数组到 mt9v03x_image 的映射关系
//  @brief          将 188x120 原始图以 2:1 等比压缩为 94x60 尺寸
//  @brief          即 Image_Use[i][j] 指向 mt9v03x_image[i*2][j*2]
//  @parameter      void
//  @return         void
//  @note           仅需调用一次, 建议在 Camera_Init() 之后立即调用
//  Sample usage:   Camera_CompressInit();
//-------------------------------------------------------------------------------
void Camera_CompressInit(void)
{
    uint8 i, j;
    uint16 row, line;       // 原始图像中的行列索引

    for (i = 0; i < LCDH; i++)
    {
        row = (uint16)i * COMPRESS_STEP_H;   // i * 2, 向下采样

        for (j = 0; j < LCDW; j++)
        {
            line = (uint16)j * COMPRESS_STEP_W;  // j * 2, 向右采样

            /*
             * Image_Use[i][j] 指向 mt9v03x_image[row][line] 的地址.
             * 通过指针间接访问, 不复制数据, 节省内存.
             */
            Image_Use[i][j] = &mt9v03x_image[row][line];
        }
    }
}

//-------------------------------------------------------------------------------
//  @brief          Camera_OTSU_GetThreshold - OTSU大津法求阈值
//  @brief          遍历灰度直方图, 计算使类间方差最大的分割阈值
//  @brief          算法: 遍历0-255灰度级, 计算背景和前景的类间方差
//  @brief          类间方差 = w0*(u0-u)^2 + w1*(u1-u)^2
//  @brief          取使方差最大的灰度值作为阈值
//  @parameter      image   压缩后的灰度图像指针数组, 即 Image_Use
//  @parameter      col     图像宽度 (列数)
//  @parameter      row     图像高度 (行数)
//  @return         uint8   最佳分割阈值 (0~255)
//  @note           灰度级固定为256, 时间复杂度O(256*N)
//  Sample usage:   uint8 thr = Camera_OTSU_GetThreshold(Image_Use, LCDW, LCDH);
//-------------------------------------------------------------------------------
uint8 Camera_OTSU_GetThreshold(uint8 *image[][LCDW], uint16 col, uint16 row)
{
    #define GRAY_SCALE  256                     // 灰度级(0-255)
    uint16 width  = col;
    uint16 height = row;
    uint32 pixelSum;                             // 总像素数
    uint32 pixelCount[GRAY_SCALE];               // 每个灰度级的像素计数
    float  pixelPro[GRAY_SCALE];                 // 每个灰度级的像素比例
    uint32 gray_sum = 0;                         // 灰度值总和
    uint8  threshold = 0;                        // 最佳阈值
    uint16 i, j;

    /* ---- 计算总像素数 ---- */
    pixelSum = (uint32)width * height;

    /* ---- 初始化直方图 ---- */
    for (i = 0; i < GRAY_SCALE; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i]  = 0.0f;
    }

    /* ---- 统计每个灰度值(0-255)在图像中出现的次数 ---- */
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            uint8 gray_val = *image[i][j];
            pixelCount[gray_val]++;              // 计数
            gray_sum += gray_val;
        }
    }

    /* ---- 计算每个灰度值在整幅图像中所占的比例 ---- */
    for (i = 0; i < GRAY_SCALE; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / (float)pixelSum;
    }

    /* ---- OTSU主循环: 遍历所有灰度级找最大类间方差 ---- */
    {
        float w0 = 0.0f, w1 = 0.0f;              // 背景/前景比例
        float u0tmp = 0.0f, u1tmp = 0.0f;        // 背景/前景累积灰度
        float u0, u1, u;                         // 背景平均灰度, 前景平均灰度, 全局平均灰度
        float deltaTmp, deltaMax = 0.0f;
        float gray_avg;                           // 全局灰度平均值
        uint8  jj;

        gray_avg = (float)gray_sum / (float)pixelSum;

        for (jj = 0; jj < GRAY_SCALE; jj++)
        {
            w0    += pixelPro[jj];               // 背景部分比例累加
            u0tmp += (float)jj * pixelPro[jj];

            /* ---- w0=0 或 w1=0 时跳过 (除零保护) ---- */
            if (w0 < 1e-6f || (1.0f - w0) < 1e-6f)
            {
                continue;
            }

            w1    = 1.0f - w0;
            u1tmp = gray_avg - u0tmp;

            u0    = u0tmp / w0;                  // 背景平均灰度
            u1    = u1tmp / w1;                  // 前景平均灰度
            u     = u0tmp + u1tmp;               // 全局平均灰度

            /* ---- 类间方差 = w0*(u0-u)^2 + w1*(u1-u)^2 ---- */
            deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);

            if (deltaTmp > deltaMax)
            {
                deltaMax  = deltaTmp;
                threshold = jj;
            }
            /*
             * 优化: 方差开始递减说明已过最佳点, 提前退出.
             * 这是安财代码中的优化技巧, 可减少约一半的计算量.
             */
            else if (deltaTmp < deltaMax)
            {
                break;
            }
        }
    }

    return threshold;
}

//-------------------------------------------------------------------------------
//  @brief          Camera_GetBinaryImage - 灰度图二值化
//  @brief          先调用 Camera_OTSU_GetThreshold() 计算最优阈值, 再遍历 Image_Use
//  @brief          结果存入 Pixle[][] 中: 灰度值大于阈值 -> 1(白/赛道), 否则 -> 0(黑/边界)
//  @brief          同时将阈值存入全局变量 Camera_Threshold, 供其他模块使用
//  @parameter      void
//  @return         void
//  @note           需要先调用 Image_CompressInit() 建立映射
//  Sample usage:   Camera_GetBinaryImage();
//-------------------------------------------------------------------------------
void Camera_GetBinaryImage(void)
{
    uint8  threshold;                            // 局部阈值
    uint8  i, j;

    /* ---- OTSU计算最优分割阈值 ---- */
    threshold = Camera_OTSU_GetThreshold(Image_Use, (uint16)LCDW, (uint16)LCDH);

    /* ---- 保存阈值到全局变量, 便于其他模块使用 ---- */
    Camera_Threshold = threshold;

    /* ---- 遍历整幅图像进行二值化 ---- */
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j < LCDW; j++)
        {
            if (*Image_Use[i][j] > threshold)
            {
                Pixle[i][j] = 1;   // 白 (赛道区域)
            }
            else
            {
                Pixle[i][j] = 0;   // 黑 (边界/赛道外)
            }
        }
    }
}

//-------------------------------------------------------------------------------
//  @brief          Camera_ShowDebug - IPS200调试显示
//  @brief          在IPS200屏幕上分三区域显示:
//  @brief            上部 (y=0~119):    原始灰度图 188x120
//  @brief            中部 (y=130):      OTSU计算出的阈值数值
//  @brief            下部 (y=150~209):  二值化图像 94x60 (居中显示)
//  @brief          屏幕底行: 图例文字说明
//  @parameter      void
//  @return         void
//  @note           需在 Camera_GetBinaryImage() 之后调用
//  Sample usage:   Camera_GetBinaryImage(); Camera_ShowDebug();
//-------------------------------------------------------------------------------
void Camera_ShowDebug(void)
{
    uint16  x_offset;    // 二值化图像水平居中偏移量

    /*
     * 1. 显示原始灰度图像 (y=0, 全分辨率 188x120)
     *    使用逐飞库 ips200_show_gray_image, 根据CASET/RASET自动定位
     */
    ips200_show_gray_image(
        0, 0,                             // 起始坐标(0,0)
        mt9v03x_image[0],                 // 原始图像数据
        MT9V03X_W, MT9V03X_H,            // 源尺寸 188x120
        MT9V03X_W, MT9V03X_H,            // 显示尺寸 188x120 (1:1)
        0                                 // 阈值=0: 不做二值化, 直接显示灰度
    );

    /*
     * 2. 显示 OTSU 阈值数值
     *    黄色文字, 黑色背景, 便于阅读
     */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    {
        uint16 txt_x = 2;
        uint16 txt_y = 125;
        ips200_show_string(txt_x, txt_y, "OTSU Thr:");
        ips200_show_uint(txt_x + 80, txt_y, Camera_Threshold, 3);
    }

    /*
     * 3. 显示二值化图像 (y=150, 居中 94x60)
     *    Pixle[] 每字节存一个像素 (0或1), 使用 grey_image 函数
     *    阈值=0 实现: 0(黑)<=0 -> 黑色, 1(白)>0 -> 白色
     */
    x_offset = (uint16)((MT9V03X_W - LCDW) / 2);    // (188-94)/2 = 47, 居中

    ips200_show_gray_image(
        x_offset, 150,                    // 起始坐标 (47, 150) 居中
        Pixle[0],                         // 二值化图像数据
        LCDW, LCDH,                       // 源尺寸 94x60
        LCDW, LCDH,                       // 显示尺寸 94x60 (1:1)
        0                                 // 阈值=0: 0为黑, 非0为白
    );

    /*
     * 4. 显示图例说明
     *    白色文字, 标注二值化含义
     */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    {
        uint16 txt_x = 2;
        uint16 txt_y = 215;
        ips200_show_string(txt_x, txt_y, "[0=黑赛道, 1=白赛道]");
    }

    /* ---- 恢复默认颜色为红色 ---- */
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}
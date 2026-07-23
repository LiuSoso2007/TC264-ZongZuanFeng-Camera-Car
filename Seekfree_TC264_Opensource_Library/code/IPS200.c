/******************************************************************************
 * IPS200.c - TC264 IPS200屏幕驱动 (基于逐飞库 zf_device_ips200)
 *
 * 接口: 软件SPI
 * 引脚: SCL=P15_4, SDA=P15_2, RST=P15_0, DC=P15_1, CS=P15_5, BLK=P15_3
 * 分辨率: 320x240
 *
 * 此文件仅提供轻量封装, 所有底层驱动在 zf_device_ips200 库中。
 * 严禁修改逐飞设备库函数。
 ******************************************************************************/
#include "IPS200.h"
#include "IfxPort_reg.h"

#define IPS200_SCREEN_WIDTH      (240U)
#define IPS200_SCREEN_HEIGHT     (320U)

/* 屏幕软件SPI的四根信号线均位于P15，直接写OMR可避免通用GPIO函数开销。 */
#define IPS200_SCL_SET_MASK      (1U << 4)
#define IPS200_SCL_CLEAR_MASK    (1U << 20)
#define IPS200_SDA_SET_MASK      (1U << 2)
#define IPS200_SDA_CLEAR_MASK    (1U << 18)
#define IPS200_DC_SET_MASK       (1U << 1)
#define IPS200_DC_CLEAR_MASK     (1U << 17)
#define IPS200_CS_SET_MASK       (1U << 5)
#define IPS200_CS_CLEAR_MASK     (1U << 21)

static uint16 ips200_gray_rgb565[256];

/*
 * 按SPI模式0发送数据：下降沿切换数据、上升沿由屏幕采样。
 * 每位仅写两次P15 OMR寄存器，P15总线访问间隔同时限制了最高时钟频率。
 */
static inline void IPS200_WriteBitsDirect(uint32 data, uint8 bit_count)
{
    uint32 data_mask;
    uint32 port_value;

    data_mask = 1U << (bit_count - 1U);
    while (data_mask != 0U)
    {
        port_value = IPS200_SCL_CLEAR_MASK;
        if ((data & data_mask) != 0U)
        {
            port_value |= IPS200_SDA_SET_MASK;
        }
        else
        {
            port_value |= IPS200_SDA_CLEAR_MASK;
        }

        MODULE_P15.OMR.U = port_value;
        MODULE_P15.OMR.U = IPS200_SCL_SET_MASK;
        data_mask >>= 1U;
    }

    MODULE_P15.OMR.U = IPS200_SCL_CLEAR_MASK;
}

static inline void IPS200_WriteCommandDirect(uint8 command)
{
    MODULE_P15.OMR.U = IPS200_DC_CLEAR_MASK;
    IPS200_WriteBitsDirect(command, 8U);
    MODULE_P15.OMR.U = IPS200_DC_SET_MASK;
}

static void IPS200_InitGrayTable(void)
{
    uint16 gray;

    for (gray = 0; gray < 256U; gray++)
    {
        ips200_gray_rgb565[gray] = (uint16)(((gray & 0xF8U) << 8)
            | ((gray & 0xFCU) << 3) | (gray >> 3));
    }
}

/*
 * IPS200_Init - 初始化IPS200显示屏
 * 配置: SPI接口, 竖屏模式, 8x16字体, 清屏
 */
void IPS200_Init(void)
{
    ips200_init(IPS200_TYPE_SPI);           /* 按当前接线初始化软件SPI */
    ips200_set_dir(IPS200_PORTAIT);        /* 竖屏模式 */
    ips200_set_font(IPS200_8X16_FONT);     /* 8x16字体 */
    IPS200_InitGrayTable();                 /* 预计算灰度到RGB565映射 */
}

/* 使用现有软件SPI引脚直接刷新灰度图，避免逐飞通用软件SPI的函数和延时开销。 */
void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height)
{
    uint32 pixel_count;

    if (image == NULL || width == 0U || height == 0U
        || width > IPS200_SCREEN_WIDTH || height > IPS200_SCREEN_HEIGHT)
    {
        return;
    }

    pixel_count = (uint32)width * height;

    MODULE_P15.OMR.U = IPS200_CS_CLEAR_MASK | IPS200_SCL_CLEAR_MASK;

    /* 设置从屏幕左上角开始、与输入图像等大的连续写入区域。 */
    IPS200_WriteCommandDirect(0x2AU);
    IPS200_WriteBitsDirect(0U, 16U);
    IPS200_WriteBitsDirect(width - 1U, 16U);

    IPS200_WriteCommandDirect(0x2BU);
    IPS200_WriteBitsDirect(0U, 16U);
    IPS200_WriteBitsDirect(height - 1U, 16U);

    IPS200_WriteCommandDirect(0x2CU);
    while (pixel_count > 0U)
    {
        IPS200_WriteBitsDirect(ips200_gray_rgb565[*image], 16U);
        image++;
        pixel_count--;
    }

    MODULE_P15.OMR.U = IPS200_CS_SET_MASK | IPS200_SCL_CLEAR_MASK;
}

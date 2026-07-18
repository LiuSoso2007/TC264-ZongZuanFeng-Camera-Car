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
#include "IfxQspi_reg.h"

#define IPS200_QSPI_FIFO_DEPTH (4U)
#define IPS200_QSPI_TIMEOUT    (1000000U)
#define IPS200_SCREEN_WIDTH    (240U)
#define IPS200_SCREEN_HEIGHT   (320U)

static uint16 ips200_gray_rgb565[256];

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
    ips200_init(IPS200_TYPE_SPI);          /* QSPI2硬件接口初始化 */
    ips200_clear();                         /* 清屏 */
    ips200_set_dir(IPS200_PORTAIT);        /* 竖屏模式 */
    ips200_set_font(IPS200_8X16_FONT);     /* 8x16字体 */
    IPS200_InitGrayTable();                 /* 预计算灰度到RGB565映射 */
}

/*
 * 使用QSPI2发送FIFO直接刷新灰度图，避免逐字节等待FIFO清空。
 * 每个32位数据项打包两个RGB565像素，高位像素先发。
 */
void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height)
{
    Ifx_QSPI_BACON stream_config;
    uint32 pixel_count;
    uint32 pair_count;
    uint32 non_final_pairs;
    uint32 packed_pixels;
    uint32 wait_count;
    uint8 has_tail;

    if (image == NULL || width == 0U || height == 0U
        || width > IPS200_SCREEN_WIDTH || height > IPS200_SCREEN_HEIGHT)
    {
        return;
    }

    pixel_count = (uint32)width * height;
    pair_count = pixel_count / 2U;
    has_tail = (uint8)(pixel_count & 1U);
    non_final_pairs = pair_count;
    if (has_tail == 0U)
    {
        non_final_pairs--;
    }

    gpio_low(IPS200_CS_PIN_SPI);

    /* 设置连续写入区域，少量命令继续使用已验证的逐飞接口。 */
    gpio_low(IPS200_DC_PIN_SPI);
    spi_write_8bit(IPS200_SPI, 0x2AU);
    gpio_high(IPS200_DC_PIN_SPI);
    spi_write_16bit(IPS200_SPI, 0U);
    spi_write_16bit(IPS200_SPI, width - 1U);

    gpio_low(IPS200_DC_PIN_SPI);
    spi_write_8bit(IPS200_SPI, 0x2BU);
    gpio_high(IPS200_DC_PIN_SPI);
    spi_write_16bit(IPS200_SPI, 0U);
    spi_write_16bit(IPS200_SPI, height - 1U);

    gpio_low(IPS200_DC_PIN_SPI);
    spi_write_8bit(IPS200_SPI, 0x2CU);
    gpio_high(IPS200_DC_PIN_SPI);

    wait_count = IPS200_QSPI_TIMEOUT;
    while (MODULE_QSPI2.STATUS.B.TXFIFOLEVEL != 0U)
    {
        if (--wait_count == 0U) goto transfer_failed;
    }

    MODULE_QSPI2.FLAGSCLEAR.U = 0xFFFFU;
    stream_config.U = MODULE_QSPI2.BACON.U;

    if (non_final_pairs > 0U)
    {
        /* 每个FIFO数据项连续发送两个RGB565像素，减少一半寄存器写入。 */
        stream_config.B.DL = 31;
        stream_config.B.LAST = 0;
        MODULE_QSPI2.BACONENTRY.U = stream_config.U;

        while (non_final_pairs > 0U)
        {
            wait_count = IPS200_QSPI_TIMEOUT;
            while (MODULE_QSPI2.STATUS.B.TXFIFOLEVEL >= IPS200_QSPI_FIFO_DEPTH)
            {
                if (--wait_count == 0U) goto transfer_failed;
            }

            packed_pixels = ((uint32)ips200_gray_rgb565[image[0]] << 16)
                | ips200_gray_rgb565[image[1]];
            MODULE_QSPI2.DATAENTRY[0].U = packed_pixels;
            image += 2;
            non_final_pairs--;
        }
    }

    /* 末尾BACON和最后一个数据项各占一个FIFO槽位。 */
    wait_count = IPS200_QSPI_TIMEOUT;
    while (MODULE_QSPI2.STATUS.B.TXFIFOLEVEL > (IPS200_QSPI_FIFO_DEPTH - 2U))
    {
        if (--wait_count == 0U) goto transfer_failed;
    }

    stream_config.B.LAST = 1;
    if (has_tail != 0U)
    {
        stream_config.B.DL = 15;
        packed_pixels = ips200_gray_rgb565[image[0]];
    }
    else
    {
        stream_config.B.DL = 31;
        packed_pixels = ((uint32)ips200_gray_rgb565[image[0]] << 16)
            | ips200_gray_rgb565[image[1]];
    }

    MODULE_QSPI2.BACONENTRY.U = stream_config.U;
    MODULE_QSPI2.DATAENTRY[0].U = packed_pixels;

    wait_count = IPS200_QSPI_TIMEOUT;
    while (MODULE_QSPI2.STATUS.B.TXFIFOLEVEL != 0U)
    {
        if (--wait_count == 0U) goto transfer_failed;
    }

    wait_count = IPS200_QSPI_TIMEOUT;
    while (MODULE_QSPI2.STATUS.B.PT1F == 0U)
    {
        if (--wait_count == 0U) goto transfer_failed;
    }

    MODULE_QSPI2.FLAGSCLEAR.U = 0xFFFFU;
    gpio_high(IPS200_CS_PIN_SPI);
    return;

transfer_failed:
    /* 超时后复位状态机和收发FIFO，避免残留数据破坏下一帧。 */
    gpio_high(IPS200_CS_PIN_SPI);
    MODULE_QSPI2.GLOBALCON.B.RESETS = 7U;
    MODULE_QSPI2.FLAGSCLEAR.U = 0xFFFFU;
}

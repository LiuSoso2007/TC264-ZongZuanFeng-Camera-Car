#ifndef __IPS200_WRAP_H__
#define __IPS200_WRAP_H__

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"

/*
 * IPS200.h --- IPS200显示屏轻量封装
 *
 * 基于逐飞 zf_device_ips200 库, 提供最简初始化接口。
 * 所有显示函数 (画点/线/字符/图像/波形) 直接使用 zf_device_ips200.h 中的API。
 *
 * 常用函数速查:
 *   ips200_clear()                  -- 清屏
 *   ips200_show_string(x, y, str)   -- 显示字符串
 *   ips200_show_uint(x, y, v, n)    -- 显示无符号整数
 *   ips200_show_float(x, y, v, n, p)-- 显示浮点数
 *   ips200_show_gray_image(...)     -- 显示灰度图 (支持缩放+阈值)
 *   ips200_show_binary_image(...)   -- 显示二值图 (位打包格式)
 *   ips200_show_rgb565_image(...)   -- 显示RGB565彩色图
 *   ips200_set_color(pen, bg)       -- 设置前景/背景色
 */

void IPS200_Init(void);
void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height);



void IPS200_Backlight_Off(void);
#endif
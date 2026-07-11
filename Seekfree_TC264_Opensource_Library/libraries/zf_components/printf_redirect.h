/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          printf_redirect
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2025-04-19       pudding            first version
********************************************************************************************************************/
#ifndef _PRINTF_REDIRECT_H_
#define _PRINTF_REDIRECT_H_

#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_driver_spi.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_spi.h"


#define  PRINTF_OUTPUT_TYPE     0                       // printf 打印输出类型 0 使用串口   1 使用SPI


#if PRINTF_OUTPUT_TYPE == 0

// 逐飞科技开源库 串口输出模块 默认串口宏定义如下：
// DEBUG_UART_INDEX                                     // debug 调试串口
// BLE6A20_INDEX                                        // ble6a20 无线蓝牙模块
// BLUETOOTH_CH9141_INDEX                               // ch9141 蓝牙模块
// WIFI_UART_INDEX                                      // wifi 串口模块
// WIRELESS_UART_INDEX                                  // 无线串口模块

#define  PRINTF_USE_UART        (DEBUG_UART_INDEX)      // 默认 printf 使用 DEBUG_UART_INDEX 串口输出


// 如果 PRINTF_OUTPUT_TYPE 为 1，下方配置则为有效状态 printf 将会通过 SPI 模块输出
#else

// ====================================  注意  ====================================
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// ====================================  注意  ====================================
#define  PRINTF_USE_SPI         (WIFI_SPI_INDEX)        // 默认 printf 使用 WIFI_SPI_INDEX 模块输出(该模式暂时仅支持 WIFI-SPI)

// 逐飞科技开源库 SPI输出模块 默认串口宏定义如下：
// WIFI_SPI_INDEX               // WIFI-SPI 模块

#endif


#endif

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
* 文件名称          zf_device_imu660rx
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2025-03-6        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   // 硬件 SPI 引脚
*                   SCL/SPC           查看 zf_device_imu660rx.h 中 IMU660RX_SPC_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660rx.h 中 IMU660RX_SDI_PIN 宏定义
*                   SA0/SDO           查看 zf_device_imu660rx.h 中 IMU660RX_SDO_PIN 宏定义
*                   CS                查看 zf_device_imu660rx.h 中 IMU660RX_CS_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*
*                   // 软件 IIC 引脚
*                   SCL/SPC           查看 zf_device_imu660rx.h 中 IMU660RX_SCL_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660rx.h 中 IMU660RX_SDA_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_imu660rx_h_
#define _zf_device_imu660rx_h_
#include "zf_device_imu660rb.h"
#include "zf_device_imu660ra.h"
#include "zf_common_typedef.h"

// IMU660RX_USE_SOFT_IIC定义为0表示使用硬件SPI驱动 定义为1表示使用软件IIC驱动
// 当更改IMU660RX_USE_SOFT_IIC定义后，需要先编译并下载程序，单片机与模块需要断电重启才能正常通讯
#define IMU660RX_USE_SOFT_IIC         (0)                                       // 默认使用硬件 SPI 方式驱动
#if IMU660RX_USE_SOFT_IIC                                                       // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 IIC 驱动====================================================
#define IMU660RX_SOFT_IIC_DELAY     (60)                                       // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define IMU660RX_SCL_PIN            (P20_11)                                       // 软件 IIC SCL 引脚 连接 IMU660RX 的 SCL 引脚
#define IMU660RX_SDA_PIN            (P20_14)                                       // 软件 IIC SDA 引脚 连接 IMU660RX 的 SDA 引脚
//====================================================软件 IIC 驱动====================================================
#else

//====================================================硬件 SPI 驱动====================================================
#define IMU660RX_SPI_SPEED          (10 * 1000 * 1000)                          // 硬件 SPI 速率
#define IMU660RX_SPI                (SPI_0)                                     // 硬件 SPI 号
#define IMU660RX_SPC_PIN            (SPI0_SCLK_P20_11 )                             // 硬件 SPI SCK 引脚
#define IMU660RX_SDI_PIN            (SPI0_MOSI_P20_14)                             // 硬件 SPI MOSI 引脚
#define IMU660RX_SDO_PIN            (SPI0_MISO_P20_12)                             // 硬件 SPI MISO 引脚
//====================================================硬件 SPI 驱动====================================================
#endif
#define IMU660RX_CS_PIN             (P20_13)                                       // CS 片选引脚
#define IMU660RX_CS(x)              ((x) ? (gpio_high(IMU660RX_CS_PIN)) : (gpio_low(IMU660RX_CS_PIN)))

#define IMU660RX_TIMEOUT_COUNT      (0x00FF)                                    // IMU660 超时计数

//================================================定义 IMU660RX 内部地址================================================
#define IMU660RX_DEV_ADDR           (0x6B)                                      // SA0接地：0x6B SA0上拉：0x69 模块默认上拉

#define IMU660RX_SPI_W              (0x00)
#define IMU660RX_SPI_R              (0x80)

#define IMU660RX_CHIP_IDA           (0x00)                                     //660RAID
#define IMU660RX_CHIP_IDB           (0x0F)                                     //660RBID


//================================================声明 IMU660RX 全局变量================================================
extern int16 imu660rx_gyro_x, imu660rx_gyro_y, imu660rx_gyro_z;                   // 三轴陀螺仪数据      gyro (陀螺仪)
extern int16 imu660rx_acc_x, imu660rx_acc_y, imu660rx_acc_z;                      // 三轴加速度计数据     acc (accelerometer 加速度计)
extern float imu660rx_transition_factor[2];
//================================================声明 IMU660RX 全局变量================================================


//================================================声明 IMU660RX 基础函数================================================
void  imu660rx_get_acc            (void);                                         // 获取 IMU660RX 加速度计数据
void  imu660rx_get_gyro           (void);                                         // 获取 IMU660RX 陀螺仪数据
uint8 imu660rx_init               (void);                                         // 初始化 IMU660RX
//================================================声明 IMU660RX 基础函数================================================


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RX 加速度计数据转换为实际物理数据
// 参数说明     acc_value       任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = imu660ra_acc_transition(imu660ra_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define imu660rx_acc_transition(acc_value)      ((float)(acc_value) / imu660rx_transition_factor[0])

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = imu660ra_gyro_transition(imu660ra_gyro_x);         // 单位为 °/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define imu660rx_gyro_transition(gyro_value)    ((float)(gyro_value) / imu660rx_transition_factor[1])



#endif


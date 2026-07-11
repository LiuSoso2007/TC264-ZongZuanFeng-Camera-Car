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
* 文件名称          zf_device_imu660rb
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  // 硬件 SPI 引脚
*                  SCL/SPC            查看 zf_device_imu660rb.h 中 IMU660RB_SPC_PIN 宏定义
*                  SDA/DSI            查看 zf_device_imu660rb.h 中 IMU660RB_SDI_PIN 宏定义
*                  SA0/SDO            查看 zf_device_imu660rb.h 中 IMU660RB_SDO_PIN 宏定义
*                  CS                 查看 zf_device_imu660rb.h 中 IMU660RB_CS_PIN  宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*
*                  // 软件 IIC 引脚
*                  SCL/SPC            查看 zf_device_imu660rb.h 中 IMU660RB_SCL_PIN 宏定义
*                  SDA/DSI            查看 zf_device_imu660rb.h 中 IMU660RB_SDA_PIN 宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_imu660rb.h"

int16 imu660rb_gyro_x = 0, imu660rb_gyro_y = 0, imu660rb_gyro_z = 0;       // 三轴陀螺仪数据      GYRO (陀螺仪)
int16 imu660rb_acc_x = 0,  imu660rb_acc_y = 0,  imu660rb_acc_z = 0;        // 三轴加速度计数据     ACC  (accelerometer 加速度计)
float imu660rb_transition_factor[2] = {4098, 14.3};                  // 转换实际值的比例

#if IMU660RB_USE_SOFT_IIC
static soft_iic_info_struct imu660rb_iic_struct;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660rb_write_acc_gyro_register(IMU660RB_SLV0_CONFIG, 0x00);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
#define imu660rb_write_acc_gyro_register(reg,data)       (soft_iic_write_8bit_register(&imu660rb_iic_struct,reg,data))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     imu660rb_read_acc_gyro_register(IMU660RB_STATUS_MASTER);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
#define imu660rb_read_acc_gyro_register(reg)             (soft_iic_sccb_read_register(&imu660rb_iic_struct,reg))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 读数据 内部调用
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu660rb_read_acc_gyro_registers(IMU660RB_OUTX_L_A, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
#define imu660rb_read_acc_gyro_registers(reg,data,len)   (soft_iic_read_8bit_registers(&imu660rb_iic_struct,reg,data,len))
#else
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660rb_write_acc_gyro_register(IMU660RB_SLV0_CONFIG, 0x00);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660rb_write_acc_gyro_register (uint8 reg, uint8 data)
{
    IMU660RB_CS(0);
    spi_write_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_W, data);

    IMU660RB_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     imu660rb_read_acc_gyro_register(IMU660RB_STATUS_MASTER);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660rb_read_acc_gyro_register (uint8 reg)
{
    uint8 data = 0;
    IMU660RB_CS(0);
    data = spi_read_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_R);

    IMU660RB_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 读数据 内部调用
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu660rb_read_acc_gyro_registers(IMU660RB_OUTX_L_A, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660rb_read_acc_gyro_registers (uint8 reg, uint8 *data, uint32 len)
{
    IMU660RB_CS(0);
    spi_read_8bit_registers(IMU660RB_SPI, reg | IMU660RB_SPI_R, data, len);

    IMU660RB_CS(1);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 六轴自检 内部调用
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     imu660rb_acc_gyro_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660rb_acc_gyro_self_check (void)
{
    uint8 return_state = 0;
    uint8 dat = 0;
    uint16 timeout_count = 0;

    while(0x6B != dat)                                                          // 判断 ID 是否正确
    {
        if(IMU660RB_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        dat = imu660rb_read_acc_gyro_register(IMU660RB_WHO_AM_I);
        system_delay_ms(10);
    }
    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RB 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rb_get_acc();
// 备注信息     执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu660rb_get_acc (void)
{
    uint8 dat[6];

    imu660rb_read_acc_gyro_registers(IMU660RB_OUTX_L_A, dat, 6);
    imu660rb_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rb_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rb_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RB 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rb_get_gyro();
// 备注信息     执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu660rb_get_gyro (void)
{
    uint8 dat[6];

    imu660rb_read_acc_gyro_registers(IMU660RB_OUTX_L_G, dat, 6);
    imu660rb_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rb_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rb_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RB
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     imu660rb_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660rb_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(10);                                                        // 上电延时

#if IMU660RB_USE_SOFT_IIC
    soft_iic_init(&imu660rb_iic_struct, IMU660RB_DEV_ADDR, IMU660RB_SOFT_IIC_DELAY, IMU660RB_SCL_PIN, IMU660RB_SDA_PIN);
#else
    spi_init(IMU660RB_SPI, SPI_MODE0, IMU660RB_SPI_SPEED, IMU660RB_SPC_PIN, IMU660RB_SDI_PIN, IMU660RB_SDO_PIN, SPI_CS_NULL);
    gpio_init(IMU660RB_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
#endif

    do
    {
        imu660rb_write_acc_gyro_register(IMU660RB_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL3_C, 0x01);               // 复位设备
        system_delay_ms(2);
        imu660rb_write_acc_gyro_register(IMU660RB_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
        if(imu660rb_acc_gyro_self_check())
        {
            zf_log(0, "IMU660RB acc and gyro self check error.");
            return_state = 1;
            break;
        }

        imu660rb_write_acc_gyro_register(IMU660RB_INT1_CTRL, 0x03);             // 开启陀螺仪 加速度数据就绪中断

        // IMU660RB_CTRL1_XL 寄存器
        // 设置为 0x30 加速度量程为 ±2  G    获取到的加速度计数据除以 16393  可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x38 加速度量程为 ±4  G    获取到的加速度计数据除以 8197   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x3C 加速度量程为 ±8  G    获取到的加速度计数据除以 4098   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x34 加速度量程为 ±16 G    获取到的加速度计数据除以 2049   可以转化为带物理单位的数据 单位 g(m/s^2)
        switch(IMU660RB_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                zf_log(0, "IMU660RB_ACC_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case IMU660RB_ACC_SAMPLE_SGN_2G:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL1_XL, 0x30);
                imu660rb_transition_factor[0] = 16393;
            }break;
            case IMU660RB_ACC_SAMPLE_SGN_4G:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL1_XL, 0x38);
                imu660rb_transition_factor[0] = 8197;
            }break;
            case IMU660RB_ACC_SAMPLE_SGN_8G:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL1_XL, 0x3C);
                imu660rb_transition_factor[0] = 4098;
            }break;
            case IMU660RB_ACC_SAMPLE_SGN_16G:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL1_XL, 0x34);
                imu660rb_transition_factor[0] = 2049;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        // IMU660RB_CTRL2_G 寄存器
        // 设置为 0x52 陀螺仪量程为 ±125  dps    获取到的陀螺仪数据除以 228.6   可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x50 陀螺仪量程为 ±250  dps    获取到的陀螺仪数据除以 114.3   可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x54 陀螺仪量程为 ±500  dps    获取到的陀螺仪数据除以 57.1    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x58 陀螺仪量程为 ±1000 dps    获取到的陀螺仪数据除以 28.6    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x5C 陀螺仪量程为 ±2000 dps    获取到的陀螺仪数据除以 14.3    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x51 陀螺仪量程为 ±4000 dps    获取到的陀螺仪数据除以 7.1     可以转化为带物理单位的数据 单位为 °/s
        switch(IMU660RB_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                zf_log(0, "IMU660RB_GYRO_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_125DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x52);
                imu660rb_transition_factor[1] = 228.6;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_250DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x50);
                imu660rb_transition_factor[1] = 114.3;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_500DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x54);
                imu660rb_transition_factor[1] = 57.1;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_1000DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x58);
                imu660rb_transition_factor[1] = 28.6;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_2000DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x5C);
                imu660rb_transition_factor[1] = 14.3;
            }break;
            case IMU660RB_GYRO_SAMPLE_SGN_4000DPS:
            {
                imu660rb_write_acc_gyro_register(IMU660RB_CTRL2_G, 0x51);
                imu660rb_transition_factor[1] = 7.1;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        imu660rb_write_acc_gyro_register(IMU660RB_CTRL3_C, 0x44);                   // 使能陀螺仪数字低通滤波器
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL4_C, 0x02);                   // 使能数字低通滤波器
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL5_C, 0x00);                   // 加速度计与陀螺仪四舍五入
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL6_C, 0x00);                   // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL7_G, 0x00);                   // 开启陀螺仪高性能模式 关闭高通滤波
        imu660rb_write_acc_gyro_register(IMU660RB_CTRL9_XL, 0x01);                  // 关闭I3C接口

    }while(0);
    return return_state;
}

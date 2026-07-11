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

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_config.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"

#include "zf_device_imu660rx.h"


int16 imu660rx_gyro_x = 0, imu660rx_gyro_y = 0, imu660rx_gyro_z = 0;            // 三轴陀螺仪数据   gyro (陀螺仪)
int16 imu660rx_acc_x = 0, imu660rx_acc_y = 0, imu660rx_acc_z = 0;               // 三轴加速度计数据  acc  (accelerometer 加速度计)
float imu660rx_transition_factor[2] = {4096, 16.4};


uint8 imu660rx_self_check_id = 0;
uint8  imu660rx_acc_address = 0, imu660rx_gyro_address = 0;

#if IMU660RX_USE_SOFT_IIC
static soft_iic_info_struct imu660rx_iic_struct;
#define imu660rx_write_register(reg, data)        (soft_iic_write_8bit_register (&imu660rx_iic_struct, (reg), (data)))
#define imu660rx_write_registers(reg, data, len)  (soft_iic_write_8bit_registers(&imu660rx_iic_struct, (reg), (data), (len)))
#define imu660rx_read_registers(reg, data, len)   (soft_iic_read_8bit_registers (&imu660rx_iic_struct, (reg), (data), (len)))

#else

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RX 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660rx_write_register(IMU660RB_PWR_CONF, 0x00);                   // 关闭高级省电模式
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660rx_write_register(uint8 reg, uint8 data)
{
    IMU660RX_CS(0);
    spi_write_8bit_register(IMU660RX_SPI, reg | IMU660RX_SPI_W, data);
    IMU660RX_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RX 写数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660rx_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660rx_write_registers(uint8 reg, const uint8 *data, uint32 len)
{
    IMU660RX_CS(0);
    spi_write_8bit_registers(IMU660RX_SPI, reg | IMU660RX_SPI_W, data, len);
    IMU660RX_CS(1);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RX 读数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu660rx_read_registers(IMU660RB_ACC_ADDRESS, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660rx_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    uint8 temp_data[8];

    IMU660RX_CS(0);
    if (imu660rx_self_check_id == IMU660RX_CHIP_IDA)
    {
        spi_read_8bit_registers(IMU660RX_SPI, reg | IMU660RX_SPI_R, temp_data, len + 1);
        IMU660RX_CS(1);
        for(int i = 0; i < len; i ++)
        {
            *(data ++) = temp_data[i + 1];
        }
    }
    else if (imu660rx_self_check_id == IMU660RX_CHIP_IDB)
    {
        spi_read_8bit_registers(IMU660RX_SPI, reg | IMU660RX_SPI_R, data, len);
        IMU660RX_CS(1);
    }

}

#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RX 读寄存器
// 参数说明     reg             寄存器地址
// 参数说明     bit             寄存器位数
// 返回参数     uint8           数据
// 使用示例     imu660rx_read_register(IMU660RA_CHIP_ID1,2);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660rx_read_register(uint8 reg, uint8 bit)
{
    uint8 data[2];
#if IMU660RX_USE_SOFT_IIC
    return soft_iic_read_8bit_register (&imu660rx_iic_struct, (reg));
#else
    IMU660RX_CS(0);
    spi_read_8bit_registers(IMU660RX_SPI, reg | IMU660RX_SPI_R, data, bit);
    IMU660RX_CS(1);
#endif

    return data[bit - 1];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RX 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     imu660rx_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660rx_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    do
    {
        if(timeout_count ++ > IMU660RX_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
#if IMU660RX_USE_SOFT_IIC
        imu660rx_iic_struct.addr = IMU660RA_DEV_ADDR;
#endif
        dat = imu660rx_read_register(IMU660RX_CHIP_IDA,2);
        system_delay_ms(1);
        if(0x24 == dat)                                             // 读取设备ID1是否等于0X24，如果不是0X24则认为没检测到设备
        {
            imu660rx_self_check_id = IMU660RX_CHIP_IDA;             //记录读取到的设备ID
            break;
        }
#if IMU660RX_USE_SOFT_IIC
        imu660rx_iic_struct.addr = IMU660RB_DEV_ADDR;
#endif
        dat = imu660rx_read_register(IMU660RX_CHIP_IDB,1);
        system_delay_ms(1);
        if(0x6B == dat)                                              // 读取设备ID2是否等于0X6B，如果不是0X6B则认为没检测到设备
        {
            imu660rx_self_check_id = IMU660RX_CHIP_IDB;               //记录读取到的设备ID
            break;
        }
    }while(1);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RX 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rx_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us        采集加速度计的时间与采集陀螺仪的时间一致的原因是都只是读取寄存器数据
//-------------------------------------------------------------------------------------------------------------------
void imu660rx_get_acc (void)
{
    uint8 dat[6];

    imu660rx_read_registers(imu660rx_acc_address, dat, 6);
    imu660rx_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rx_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rx_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RX 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rx_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us
//-------------------------------------------------------------------------------------------------------------------
void imu660rx_get_gyro (void)
{
    uint8 dat[6];

    imu660rx_read_registers(imu660rx_gyro_address, dat, 6);
    imu660rx_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rx_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rx_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RX
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     imu660rx_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660rx_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(20);                                                        // 等待设备上电成功

#if IMU660RX_USE_SOFT_IIC
    soft_iic_init(&imu660rx_iic_struct, IMU660RX_DEV_ADDR, IMU660RX_SOFT_IIC_DELAY, IMU660RX_SCL_PIN, IMU660RX_SDA_PIN);        // 配置 IMU660RX 的 IIC 端口
#else
    spi_init(IMU660RX_SPI, SPI_MODE0, IMU660RX_SPI_SPEED, IMU660RX_SPC_PIN, IMU660RX_SDI_PIN, IMU660RX_SDO_PIN, SPI_CS_NULL);   // 配置 IMU660RX 的 SPI 端口
    gpio_init(IMU660RX_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);                  // 配置 IMU660RX 的 CS 端口

#endif

    do{
        if(imu660rx_self_check())                                               // IMU660RX 自检
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 IMU660RX 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            zf_log(0, "imu660rx self check error.");
            return_state = 1;
            break;
        }

        if (imu660rx_self_check_id == IMU660RX_CHIP_IDA)
        {
            imu660rx_acc_address = IMU660RA_ACC_ADDRESS;
            imu660rx_gyro_address = IMU660RA_GYRO_ADDRESS;

            imu660rx_write_register(IMU660RA_PWR_CONF, 0x00);                       // 关闭高级省电模式
            system_delay_ms(1);
            imu660rx_write_register(IMU660RA_INIT_CTRL, 0x00);                      // 开始对模块进行初始化配置
            imu660rx_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));   // 输出配置文件
            imu660rx_write_register(IMU660RA_INIT_CTRL, 0x01);                      // 初始化配置结束
            system_delay_ms(20);
            if(1 != imu660rx_read_register(IMU660RA_INT_STA,2))                       // 检查是否配置完成
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是 IMU660RA 配置初始化文件出错了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
                zf_log(0, "imu660ra init error.");
                return_state = 1;
                break;
            }
            imu660rx_write_register(IMU660RA_PWR_CTRL, 0x0E);                       // 开启性能模式  使能陀螺仪、加速度、温度传感器
            imu660rx_write_register(IMU660RA_ACC_CONF, 0xA7);                       // 加速度采集配置 性能模式 正常采集 50Hz  采样频率
            imu660rx_write_register(IMU660RA_GYR_CONF, 0xA9);                       // 陀螺仪采集配置 性能模式 正常采集 200Hz 采样频率

            // IMU660RA_ACC_SAMPLE 寄存器
            // 设置为 0x00 加速度计量程为 ±2  g   获取到的加速度计数据除以 16384  可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
            // 设置为 0x01 加速度计量程为 ±4  g   获取到的加速度计数据除以 8192   可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
            // 设置为 0x02 加速度计量程为 ±8  g   获取到的加速度计数据除以 4096   可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
            // 设置为 0x03 加速度计量程为 ±16 g   获取到的加速度计数据除以 2048   可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
            switch(IMU660RA_ACC_SAMPLE_DEFAULT)
            {
                default:
                {
                    zf_log(0, "IMU660RA_ACC_SAMPLE_DEFAULT set error.");
                    return_state = 1;
                }break;
                case IMU660RA_ACC_SAMPLE_SGN_2G:
                {
                    imu660rx_write_register(IMU660RA_ACC_RANGE, 0x00);
                    imu660rx_transition_factor[0] = 16384;
                }break;
                case IMU660RA_ACC_SAMPLE_SGN_4G:
                {
                    imu660rx_write_register(IMU660RA_ACC_RANGE, 0x01);
                    imu660rx_transition_factor[0] = 8192;
                }break;
                case IMU660RA_ACC_SAMPLE_SGN_8G:
                {
                    imu660rx_write_register(IMU660RA_ACC_RANGE, 0x02);
                    imu660rx_transition_factor[0] = 4096;
                }break;
                case IMU660RA_ACC_SAMPLE_SGN_16G:
                {
                    imu660rx_write_register(IMU660RA_ACC_RANGE, 0x03);
                    imu660rx_transition_factor[0] = 2048;
                }break;
            }
            if(1 == return_state)
            {
                break;
            }

            // IMU660RA_GYR_RANGE 寄存器
            // 设置为 0x04 陀螺仪量程为 ±125  dps    获取到的陀螺仪数据除以 262.4   可以转化为带物理单位的数据 单位为 °/s
            // 设置为 0x03 陀螺仪量程为 ±250  dps    获取到的陀螺仪数据除以 131.2   可以转化为带物理单位的数据 单位为 °/s
            // 设置为 0x02 陀螺仪量程为 ±500  dps    获取到的陀螺仪数据除以 65.6    可以转化为带物理单位的数据 单位为 °/s
            // 设置为 0x01 陀螺仪量程为 ±1000 dps    获取到的陀螺仪数据除以 32.8    可以转化为带物理单位的数据 单位为 °/s
            // 设置为 0x00 陀螺仪量程为 ±2000 dps    获取到的陀螺仪数据除以 16.4    可以转化为带物理单位的数据 单位为 °/s
            switch(IMU660RA_GYRO_SAMPLE_DEFAULT)
            {
                default:
                {
                    zf_log(0, "IMU660RA_GYRO_SAMPLE_DEFAULT set error.");
                    return_state = 1;
                }break;
                case IMU660RA_GYRO_SAMPLE_SGN_125DPS:
                {
                    imu660rx_write_register(IMU660RA_GYR_RANGE, 0x04);
                    imu660rx_transition_factor[1] = 262.4;
                }break;
                case IMU660RA_GYRO_SAMPLE_SGN_250DPS:
                {
                    imu660rx_write_register(IMU660RA_GYR_RANGE, 0x03);
                    imu660rx_transition_factor[1] = 131.2;
                }break;
                case IMU660RA_GYRO_SAMPLE_SGN_500DPS:
                {
                    imu660rx_write_register(IMU660RA_GYR_RANGE, 0x02);
                    imu660rx_transition_factor[1] = 65.6;
                }break;
                case IMU660RA_GYRO_SAMPLE_SGN_1000DPS:
                {
                    imu660rx_write_register(IMU660RA_GYR_RANGE, 0x01);
                    imu660rx_transition_factor[1] = 32.8;
                }break;
                case IMU660RA_GYRO_SAMPLE_SGN_2000DPS:
                {
                    imu660rx_write_register(IMU660RA_GYR_RANGE, 0x00);
                    imu660rx_transition_factor[1] = 16.4;
                }break;
            }
            if(1 == return_state)
            {
                break;
            }
        }
        else if (imu660rx_self_check_id == IMU660RX_CHIP_IDB)
        {
            imu660rx_acc_address = IMU660RB_OUTX_L_A;
            imu660rx_gyro_address = IMU660RB_OUTX_L_G;

            imu660rx_write_register(IMU660RB_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
            imu660rx_write_register(IMU660RB_CTRL3_C, 0x01);               // 复位设备
            system_delay_ms(2);
            imu660rx_write_register(IMU660RB_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问

            imu660rx_write_register(IMU660RB_INT1_CTRL, 0x03);             // 开启陀螺仪 加速度数据就绪中断

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
                    imu660rx_write_register(IMU660RB_CTRL1_XL, 0x30);
                    imu660rx_transition_factor[0] = 16393;
                }break;
                case IMU660RB_ACC_SAMPLE_SGN_4G:
                {
                    imu660rx_write_register(IMU660RB_CTRL1_XL, 0x38);
                    imu660rx_transition_factor[0] = 8197;
                }break;
                case IMU660RB_ACC_SAMPLE_SGN_8G:
                {
                    imu660rx_write_register(IMU660RB_CTRL1_XL, 0x3C);
                    imu660rx_transition_factor[0] = 4098;
                }break;
                case IMU660RB_ACC_SAMPLE_SGN_16G:
                {
                    imu660rx_write_register(IMU660RB_CTRL1_XL, 0x34);
                    imu660rx_transition_factor[0] = 2049;
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
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x52);
                    imu660rx_transition_factor[1] = 228.6;
                }break;
                case IMU660RB_GYRO_SAMPLE_SGN_250DPS:
                {
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x50);
                    imu660rx_transition_factor[1] = 114.3;
                }break;
                case IMU660RB_GYRO_SAMPLE_SGN_500DPS:
                {
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x54);
                    imu660rx_transition_factor[1] = 57.1;
                }break;
                case IMU660RB_GYRO_SAMPLE_SGN_1000DPS:
                {
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x58);
                    imu660rx_transition_factor[1] = 28.6;
                }break;
                case IMU660RB_GYRO_SAMPLE_SGN_2000DPS:
                {
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x5C);
                    imu660rx_transition_factor[1] = 14.3;
                }break;
                case IMU660RB_GYRO_SAMPLE_SGN_4000DPS:
                {
                    imu660rx_write_register(IMU660RB_CTRL2_G, 0x51);
                    imu660rx_transition_factor[1] = 7.1;
                }break;
            }
            if(1 == return_state)
            {
                break;
            }

            imu660rx_write_register(IMU660RB_CTRL3_C, 0x44);                   // 使能陀螺仪数字低通滤波器
            imu660rx_write_register(IMU660RB_CTRL4_C, 0x02);                   // 使能数字低通滤波器
            imu660rx_write_register(IMU660RB_CTRL5_C, 0x00);                   // 加速度计与陀螺仪四舍五入
            imu660rx_write_register(IMU660RB_CTRL6_C, 0x00);                   // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
            imu660rx_write_register(IMU660RB_CTRL7_G, 0x00);                   // 开启陀螺仪高性能模式 关闭高通滤波
            imu660rx_write_register(IMU660RB_CTRL9_XL, 0x01);                  // 关闭I3C接口
        }
    }while(0);
    return return_state;
}



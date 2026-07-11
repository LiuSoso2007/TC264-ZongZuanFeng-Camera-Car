/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
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
* 文件名称          seekfree_assistant
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期             作者             备注
* 2024-1-11        SeekFree         first version
********************************************************************************************************************/
#ifndef _seekfree_assistant_h_
#define _seekfree_assistant_h_

#include "zf_common_typedef.h"

// CCD 数据类型枚举
typedef enum
{
    SEEKFREE_ASSISTANT_DATA_TYPE_UINT8                      ,
    SEEKFREE_ASSISTANT_DATA_TYPE_UINT16                     ,
}seekfree_assistant_data_type_enum;

// CCD 数据类型枚举
typedef enum
{
    SEEKFREE_ASSISTANT_CCD_RESOLUTION_8BIT                  ,                   // 8位分辨率
    SEEKFREE_ASSISTANT_CCD_RESOLUTION_10BIT                 ,                   // 10位分辨率
    SEEKFREE_ASSISTANT_CCD_RESOLUTION_12BIT                 ,                   // 12位分辨率
}seekfree_assistant_ccd_resolution_enum;

// 摄像头类型枚举
typedef enum
{
    // 按照图像类型定义
    SEEKFREE_ASSISTANT_CAMERA_TYPE_BINARY           = 0x01  ,
    SEEKFREE_ASSISTANT_CAMERA_TYPE_GRAY                     ,
    SEEKFREE_ASSISTANT_CAMERA_TYPE_RGB565                   ,

    // 摄像头型号匹配定义
    SEEKFREE_ASSISTANT_CAMERA_TYPE_OV7725_BIN       = 0x01  ,
    SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X                  ,
    SEEKFREE_ASSISTANT_CAMERA_TYPE_SCC8660                  ,
}seekfree_assistant_camera_type_enum;

#define SEEKFREE_ASSISTANT_STC_DATA_ENABLE          ( 0     )                   // STC 单片机需要使能这个宏定义

#define SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX         ( 16    )                   // 定义示波器的最大通道数量
#define SEEKFREE_ASSISTANT_CCD_RESOLUTION_DEFAULT   ( SEEKFREE_ASSISTANT_CCD_RESOLUTION_12BIT ) // 默认 CCD 的精度 用来确认上位机纵轴长度

#define SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE       ( 1     )                   // 1：使能参数调节的功能  0：关闭参数调节的功能
#if (SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE)
#define SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX          ( 8     )                   // 定义调试参数的最大通道数量
#define SEEKFREE_ASSISTANT_DEBUG_PARAM_BUFFER_SIZE  ( 64    )                   // 定义调试参数的缓冲区大小
#endif

typedef struct
{
    struct
    {
        uint8   head                        ;                                   // 帧头
        uint8   check_sum                   ;                                   // 和校验
        uint8   cmd                         ;                                   // 命令功能字
        uint8   channel_index               ;                                   // 通道索引
        uint8   data_lenght_buffer[2]       ;                                   // CCD 数据长度 缓冲
        uint8   channel_color_buffer[2]     ;                                   // 通道颜色 缓冲
        uint8   channel_data_max[2]         ;                                   // 通道最大值 缓冲
        uint8   data_type                   ;                                   // 数据类型 用于确认大小端
        uint8   reserve                     ;                                   // 保留
    }config;
    uint16      buffer_byte_size            ;                                   // 数据缓冲区 实际字节大小
    void        *data_buffer                ;                                   // 数据缓冲区
}seekfree_assistant_ccd_struct;

typedef struct
{
    struct
    {
        uint8   head                        ;                                   // 帧头
        uint8   check_sum                   ;                                   // 和校验
        uint8   cmd                         ;                                   // 命令功能字
        uint8   camera_type                 ;                                   // 摄像头类型
        uint8   image_width_buffer[2]       ;                                   // 图像宽度 缓冲
        uint8   image_height_buffer[2]      ;                                   // 图像高度 缓冲
        uint8   data_type                   ;                                   // 数据类型 用于确认大小端
        uint8   reserve[3]                  ;                                   // 保留
    }config;
    uint32      buffer_byte_size            ;                                   // 数据缓冲区 实际字节大小
    void        *data_buffer                ;                                   // 数据缓冲区
}seekfree_assistant_camera_struct;

typedef struct
{
    struct
    {
        uint8   head                        ;                                   // 帧头
        uint8   check_sum                   ;                                   // 和校验
        uint8   cmd                         ;                                   // 命令功能字
        uint8   data_type                   ;                                   // 数据类型 用于确认大小端
        uint8   boundary_lenght_buffer[2]   ;                                   // 边线长度 缓冲
        uint8   color_buffer[2]             ;                                   // 通道颜色 缓冲
    }config;
    uint32      buffer_byte_size            ;                                   // 数据缓冲区 实际字节大小
    void        *data_buffer                ;                                   // 数据缓冲区
}seekfree_assistant_camera_boundary_struct;

typedef struct
{
    uint8       head                        ;                                   // 帧头
    uint8       check_sum                   ;                                   // 和校验
    uint8       cmd                         ;                                   // 命令功能字
    uint8       reserve                     ;                                   // 保留
    uint8       x_axis_buffer[2]            ;                                   // X 轴坐标 缓冲
    uint8       y_axis_buffer[2]            ;                                   // Y 轴坐标 缓冲
    uint8       width_buffer[2]             ;                                   // 矩形宽度 缓冲
    uint8       height_buffer[2]            ;                                   // 矩形高度 缓冲
    uint8       color_buffer[2]             ;                                   // 矩形颜色 缓冲
}seekfree_assistant_camera_rectangular_struct;

typedef struct
{
    struct
    {
        uint8   head                        ;                                   // 帧头
        uint8   check_sum                   ;                                   // 和校验
        uint8   cmd                         ;                                   // 命令功能字
        uint8   channel_max                 ;                                   // 通道数量
        uint8   data_type                   ;                                   // 数据类型 用于确认大小端
        uint8   reserve[3]                  ;                                   // 保留
    }config;
    uint32      buffer_byte_size            ;                                   // 数据缓冲区 实际字节大小
    void        *data_buffer                ;                                   // 数据缓冲区
}seekfree_assistant_oscilloscope_struct;

typedef union
{
    uint8   data_uint8[4]                   ;
    float   data_float                      ;
}seekfree_assistant_param_struct;

typedef struct
{
    uint8       head                        ;                                   // 帧头
    uint8       check_sum                   ;                                   // 和校验
    uint8       cmd                         ;                                   // 命令功能字
    uint8       channel_index               ;                                   // 通道索引
    seekfree_assistant_param_struct dat     ;                                   // 一个数据

}seekfree_assistant_debug_param_cmd_struct;

typedef struct
{
    uint8 x;
    uint8 y;
}seekfree_assistant_camera_boundary_uint8_struct;


typedef struct
{
    uint8 x;
    uint8 y;
}seekfree_assistant_camera_boundary_uint16_struct;

typedef struct
{
    float   data[SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX]        ;
    uint8   update_flag[SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX] ;
}seekfree_assistant_debug_param_struct;

typedef uint32 (*seekfree_assistant_transfer_callback_function) (const uint8 *buff, uint32 length);
typedef uint32 (*seekfree_assistant_receive_callback_function)  (uint8 *buff, uint32 length);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CCD 图像 配置
// 参数说明     *ccd_obj                CCD 数据结构体
// 参数说明     data_type               数据类型 详见 seekfree_assistant_data_type_enum
// 参数说明     channel_index           CCD 通道
// 参数说明     channel_color           通道要使用的颜色
// 参数说明     data_lenght             CCD 数据长度
// 参数说明     *data_buffer            CCD 数据缓冲区
// 返回参数     void
// 使用示例     seekfree_assistant_ccd_config(ccd_obj, data_type, channel_index, channel_color, data_lenght, data_buffer);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_ccd_config (
    seekfree_assistant_ccd_struct *ccd_obj,
    seekfree_assistant_data_type_enum data_type,
    uint8   channel_index,
    uint16  channel_color,
    uint16  data_lenght,
    void    *data_buffer);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CCD 图像 发送函数
// 参数说明     *ccd_obj            CCD 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_ccd_send(ccd_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_ccd_send (seekfree_assistant_ccd_struct *ccd_obj);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 图像 配置
// 参数说明     *camera_obj             CAMERA 数据结构体
// 参数说明     camera_type             CAMERA 类型 详见 seekfree_assistant_camera_type_enum
// 参数说明     image_width             CAMERA 图像宽度
// 参数说明     image_height            CAMERA 图像高度
// 参数说明     *data_buffer            CAMERA 图像缓冲区
// 返回参数     void
// 使用示例     seekfree_assistant_camera_config(camera_obj, camera_type, image_width, image_height, data_buffer);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_config (
    seekfree_assistant_camera_struct *camera_obj,
    seekfree_assistant_camera_type_enum camera_type,
    uint16  image_width,
    uint16  image_height,
    void    *data_buffer);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 图像 发送函数
// 参数说明     *camera_obj             CAMERA 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_camera_send(camera_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_send (seekfree_assistant_camera_struct *camera_obj);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 边线 配置
// 参数说明     *camera_boundary_obj    CAMERA 边线 数据结构体
// 参数说明     data_type               数据类型 详见 seekfree_assistant_data_type_enum
// 参数说明     color                   要使用的颜色
// 参数说明     boundary_lenght         CAMERA 边线 长度
// 参数说明     *data_buffer            CAMERA 边线 图像缓冲区
// 返回参数     void
// 使用示例     seekfree_assistant_camera_boundary_config(camera_boundary_obj, data_type, color, boundary_lenght, data_buffer);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_boundary_config (
    seekfree_assistant_camera_boundary_struct *camera_boundary_obj,
    seekfree_assistant_data_type_enum data_type,
    uint16  color,
    uint16  boundary_lenght,
    void    *data_buffer);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 边线 发送函数
// 参数说明     *camera_boundary_obj    CAMERA 边线 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_camera_boundary_send(camera_boundary_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_boundary_send (seekfree_assistant_camera_boundary_struct *camera_boundary_obj);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 矩形框线 发送函数
// 参数说明     x                       X 轴坐标
// 参数说明     y                       Y 轴坐标 缓冲
// 参数说明     width                   矩形宽度
// 参数说明     height                  矩形高度
// 参数说明     color                   矩形颜色
// 返回参数     void
// 使用示例     seekfree_assistant_camera_rectangular_send(x, y, width, height, color);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_rectangular_send (uint16 x, uint16 y, uint16 width, uint16 height, uint16 color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 配置
// 参数说明     *oscilloscope_obj       示波器数据结构体
// 参数说明     channel_max             最大通道数量 范围是 [1, 16] 超过范围会缩限到这个范围
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_config(oscilloscope_obj, 16);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_oscilloscope_config (
    seekfree_assistant_oscilloscope_struct *oscilloscope_obj,
    uint8 channel_max,
    void *data_buffer);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 发送函数
// 参数说明     *seekfree_assistant_oscilloscope  示波器数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_oscilloscope_send (seekfree_assistant_oscilloscope_struct *oscilloscope_obj);

#if (SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE)
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 发送函数
// 参数说明     *debug_param_obj        示波器数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_debug_param_analysis (seekfree_assistant_debug_param_struct *debug_param_obj);
#endif

#endif

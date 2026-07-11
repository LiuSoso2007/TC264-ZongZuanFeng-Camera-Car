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

#include "zf_common_debug.h"

#include "seekfree_assistant.h"

// 通信命令枚举
typedef enum
{
    SEEKFREE_ASSISTANT_SEND_CCD_CMD                 = 0x01  ,
    SEEKFREE_ASSISTANT_SEND_CAMERA_CMD                      ,
    SEEKFREE_ASSISTANT_SEND_CAMERA_BOUNDARY_CMD             ,
    SEEKFREE_ASSISTANT_SEND_CAMERA_RECTANGULAR_CMD          ,

    SEEKFREE_ASSISTANT_SEND_OSCILLOSCOPE_CMD        = 0x10  ,

    SEEKFREE_ASSISTANT_RECEIVE_DEBUG_PARAM_CMD      = 0x20  ,
}seekfree_assistant_cmd_type_enum;

#define SEEKFREE_ASSISTANT_SEND_HEAD                ( 0xAA  )                   // 单片机往上位机发送的帧头
#define SEEKFREE_ASSISTANT_RECEIVE_HEAD             ( 0x55  )                   // 上位机往单片机发送的帧头

// 数据发送函数指针绑定
extern uint32 seekfree_assistant_transfer (const uint8 *buff, uint32 length);
seekfree_assistant_transfer_callback_function seekfree_assistant_transfer_callback = seekfree_assistant_transfer;

// 数据接收函数指针绑定
extern uint32 seekfree_assistant_receive (uint8 *buff, uint32 length);
seekfree_assistant_receive_callback_function seekfree_assistant_receive_callback  = seekfree_assistant_receive;

#if (SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE)
#include "zf_common_fifo.h"
static uint8 seekfree_assistant_debug_param_buffer[SEEKFREE_ASSISTANT_DEBUG_PARAM_BUFFER_SIZE];
static fifo_struct  seekfree_assistant_debug_param_fifo =
{   
    .buffer    = seekfree_assistant_debug_param_buffer, 
    .execution = FIFO_IDLE, 
    .type      = FIFO_DATA_8BIT,    
    .head      = 0, 
    .end       = 0, 
    .size      = SEEKFREE_ASSISTANT_DEBUG_PARAM_BUFFER_SIZE,    
    .max       = SEEKFREE_ASSISTANT_DEBUG_PARAM_BUFFER_SIZE,    
};
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手求和函数
// 参数说明     *buffer                 需要校验的数据地址
// 参数说明     length                  校验长度
// 返回参数     uint8                   和值
// 使用示例
//-------------------------------------------------------------------------------------------------------------------
static uint8 seekfree_assistant_sum (void *buffer, uint32 length)
{
    uint8 temp_sum = 0;
    uint8 *ptr = (uint8 *)buffer;

    while(length--)
    {
        temp_sum += *ptr ++;
    }

    return temp_sum;
}

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
    void    *data_buffer)
{
    zf_assert(NULL != ccd_obj);
    zf_assert(NULL != data_buffer);

    memset(ccd_obj, 0, sizeof(seekfree_assistant_ccd_struct));
    ccd_obj->config.head                    = SEEKFREE_ASSISTANT_SEND_HEAD;
    ccd_obj->config.cmd                     = SEEKFREE_ASSISTANT_SEND_CCD_CMD;
    ccd_obj->config.channel_index           = channel_index;

    uint16 data_max = 256;
    uint8  loop_count = SEEKFREE_ASSISTANT_CCD_RESOLUTION_DEFAULT;
    loop_count = (2 >= loop_count) ? (loop_count) : (2);
    for(uint8 i = 0; loop_count > i; i ++)
    {
        data_max *= 4;
    }

    #if (SEEKFREE_ASSISTANT_STC_DATA_ENABLE)
    // 如果是 STC 那么大小端反一下
    ccd_obj->config.data_lenght_buffer[0]   = (uint8)((data_lenght      >> 8) & 0x00FF);
    ccd_obj->config.data_lenght_buffer[1]   = (uint8)((data_lenght      >> 0) & 0x00FF);
    ccd_obj->config.channel_color_buffer[0] = (uint8)((channel_color    >> 8) & 0x00FF);
    ccd_obj->config.channel_color_buffer[1] = (uint8)((channel_color    >> 0) & 0x00FF);
    ccd_obj->config.channel_data_max[0]     = (uint8)((data_max         >> 8) & 0x00FF);
    ccd_obj->config.channel_data_max[1]     = (uint8)((data_max         >> 0) & 0x00FF);
    #else
    ccd_obj->config.data_lenght_buffer[0]   = (uint8)((data_lenght      >> 0) & 0x00FF);
    ccd_obj->config.data_lenght_buffer[1]   = (uint8)((data_lenght      >> 8) & 0x00FF);
    ccd_obj->config.channel_color_buffer[0] = (uint8)((channel_color    >> 0) & 0x00FF);
    ccd_obj->config.channel_color_buffer[1] = (uint8)((channel_color    >> 8) & 0x00FF);
    ccd_obj->config.channel_data_max[0]     = (uint8)((data_max         >> 0) & 0x00FF);
    ccd_obj->config.channel_data_max[1]     = (uint8)((data_max         >> 8) & 0x00FF);
    #endif

    if(     SEEKFREE_ASSISTANT_STC_DATA_ENABLE
        &&  SEEKFREE_ASSISTANT_DATA_TYPE_UINT16 == data_type)
    {   // 如果是 uint16 类型数据 且是 STC 单片机 那么数据类型修改为 0x02
        ccd_obj->config.data_type           = data_type + 1;
    }
    else
    {   // 否则的话就正常 uint8 数据或者 uint16 数据
        ccd_obj->config.data_type           = data_type;
    }

    // 记录实际数据字节数 如果是 uint16 则需要翻倍数据长度
    ccd_obj->buffer_byte_size               = data_lenght * ((data_type) ? (2) : (1));
    // 然后记录对应数据缓冲区
    ccd_obj->data_buffer                    = data_buffer;

    // 和校验计算
    ccd_obj->config.check_sum  = SEEKFREE_ASSISTANT_SEND_HEAD;
    ccd_obj->config.check_sum += seekfree_assistant_sum(
        &(ccd_obj->config.cmd),
        sizeof(ccd_obj->config) - 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CCD 图像 发送函数
// 参数说明     *ccd_obj                CCD 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_ccd_send(ccd_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_ccd_send (seekfree_assistant_ccd_struct *ccd_obj)
{
    seekfree_assistant_transfer_callback(
        (const uint8 *)ccd_obj,
        sizeof(ccd_obj->config));
    seekfree_assistant_transfer_callback(
        (const uint8 *)(ccd_obj->data_buffer),
        ccd_obj->buffer_byte_size);
}

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
    void    *data_buffer)
{
    zf_assert(NULL != camera_obj);

    memset(camera_obj, 0, sizeof(seekfree_assistant_camera_struct));
    camera_obj->config.head                     = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_obj->config.cmd                      = SEEKFREE_ASSISTANT_SEND_CAMERA_CMD;
    camera_obj->config.camera_type              = camera_type;

    #if (SEEKFREE_ASSISTANT_STC_DATA_ENABLE)
    // 如果是 STC 那么大小端反一下
    camera_obj->config.image_width_buffer[0]    = (uint8)((image_width  >> 8) & 0x00FF);
    camera_obj->config.image_width_buffer[1]    = (uint8)((image_width  >> 0) & 0x00FF);
    camera_obj->config.image_height_buffer[0]   = (uint8)((image_height >> 8) & 0x00FF);
    camera_obj->config.image_height_buffer[1]   = (uint8)((image_height >> 0) & 0x00FF);
    #else
    camera_obj->config.image_width_buffer[0]    = (uint8)((image_width  >> 0) & 0x00FF);
    camera_obj->config.image_width_buffer[1]    = (uint8)((image_width  >> 8) & 0x00FF);
    camera_obj->config.image_height_buffer[0]   = (uint8)((image_height >> 0) & 0x00FF);
    camera_obj->config.image_height_buffer[1]   = (uint8)((image_height >> 8) & 0x00FF);
    #endif

    if(     SEEKFREE_ASSISTANT_STC_DATA_ENABLE
        &&  SEEKFREE_ASSISTANT_CAMERA_TYPE_RGB565 == camera_type
        &&  NULL != data_buffer)
    {
        camera_obj->config.data_type            = 0x02;
    }
    else
    {
        camera_obj->config.data_type            = (NULL == data_buffer) ? (0x00) : (0x01);
    }

    // 如果是 RGB565 图像 那么数据字节数翻倍
    camera_obj->buffer_byte_size                = 
            image_width * image_height
        *   ((SEEKFREE_ASSISTANT_CAMERA_TYPE_RGB565 == camera_type) ? (2) : (1));
    camera_obj->buffer_byte_size                = 
            (SEEKFREE_ASSISTANT_CAMERA_TYPE_BINARY == camera_type)
        ?   (camera_obj->buffer_byte_size / 8)
        :   (camera_obj->buffer_byte_size);
    camera_obj->data_buffer                     = data_buffer;

    // 和校验计算
    camera_obj->config.check_sum  = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_obj->config.check_sum += seekfree_assistant_sum(
        &(camera_obj->config.cmd),
        sizeof(camera_obj->config) - 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 图像 发送函数
// 参数说明     *camera_obj             CAMERA 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_camera_send(camera_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_send (seekfree_assistant_camera_struct *camera_obj)
{
    seekfree_assistant_transfer_callback(
        (const uint8 *)camera_obj,
        sizeof(camera_obj->config));
    if(NULL != camera_obj->data_buffer)
    {
        seekfree_assistant_transfer_callback(
            (const uint8 *)(camera_obj->data_buffer),
            camera_obj->buffer_byte_size);
    }
}

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
    void    *data_buffer)
{
    zf_assert(NULL != camera_boundary_obj);
    zf_assert(NULL != data_buffer);

    memset(camera_boundary_obj, 0, sizeof(seekfree_assistant_camera_boundary_struct));
    camera_boundary_obj->config.head                = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_boundary_obj->config.cmd                 = SEEKFREE_ASSISTANT_SEND_CAMERA_BOUNDARY_CMD;

    if(     SEEKFREE_ASSISTANT_STC_DATA_ENABLE
        &&  SEEKFREE_ASSISTANT_DATA_TYPE_UINT16 == data_type)
    {   // 如果是 uint16 类型数据 且是 STC 单片机 那么数据类型修改为 0x02
        camera_boundary_obj->config.data_type       = data_type + 1;
    }
    else
    {   // 否则的话就正常 uint8 数据或者 uint16 数据
        camera_boundary_obj->config.data_type       = data_type;
    }

    #if (SEEKFREE_ASSISTANT_STC_DATA_ENABLE)
    // 如果是 STC 那么大小端反一下
    camera_boundary_obj->config.boundary_lenght_buffer[0]   = (uint8)((boundary_lenght  >> 8) & 0x00FF);
    camera_boundary_obj->config.boundary_lenght_buffer[1]   = (uint8)((boundary_lenght  >> 0) & 0x00FF);
    camera_boundary_obj->config.color_buffer[0]             = (uint8)((color            >> 8) & 0x00FF);
    camera_boundary_obj->config.color_buffer[1]             = (uint8)((color            >> 0) & 0x00FF);
    #else
    camera_boundary_obj->config.boundary_lenght_buffer[0]   = (uint8)((boundary_lenght  >> 0) & 0x00FF);
    camera_boundary_obj->config.boundary_lenght_buffer[1]   = (uint8)((boundary_lenght  >> 8) & 0x00FF);
    camera_boundary_obj->config.color_buffer[0]             = (uint8)((color            >> 0) & 0x00FF);
    camera_boundary_obj->config.color_buffer[1]             = (uint8)((color            >> 8) & 0x00FF);
    #endif

    camera_boundary_obj->buffer_byte_size   = 
            boundary_lenght * 2
        *   ((SEEKFREE_ASSISTANT_DATA_TYPE_UINT8 == data_type) ? (1) : (2));
    camera_boundary_obj->data_buffer        = data_buffer;

    // 和校验计算
    camera_boundary_obj->config.check_sum  = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_boundary_obj->config.check_sum += seekfree_assistant_sum(
        &(camera_boundary_obj->config.cmd),
        sizeof(camera_boundary_obj->config) - 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 CAMERA 边线 发送函数
// 参数说明     *camera_boundary_obj    CAMERA 边线 数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_camera_boundary_send(camera_boundary_obj);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_boundary_send (seekfree_assistant_camera_boundary_struct *camera_boundary_obj)
{
    seekfree_assistant_transfer_callback(
        (const uint8 *)camera_boundary_obj,
        sizeof(camera_boundary_obj->config));
    seekfree_assistant_transfer_callback(
        (const uint8 *)(camera_boundary_obj->data_buffer),
        camera_boundary_obj->buffer_byte_size);
}

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
void seekfree_assistant_camera_rectangular_send (uint16 x, uint16 y, uint16 width, uint16 height, uint16 color)
{
    seekfree_assistant_camera_rectangular_struct camera_rectangular_obj;
    memset(&camera_rectangular_obj, 0, sizeof(seekfree_assistant_camera_rectangular_struct));
    camera_rectangular_obj.head     = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_rectangular_obj.cmd      = SEEKFREE_ASSISTANT_SEND_CAMERA_RECTANGULAR_CMD;

    #if (SEEKFREE_ASSISTANT_STC_DATA_ENABLE)
    // 如果是 STC 那么大小端反一下
    camera_rectangular_obj.x_axis_buffer[0]     = (uint8)((x        >> 8) & 0x00FF);
    camera_rectangular_obj.x_axis_buffer[1]     = (uint8)((x        >> 0) & 0x00FF);
    camera_rectangular_obj.y_axis_buffer[0]     = (uint8)((y        >> 8) & 0x00FF);
    camera_rectangular_obj.y_axis_buffer[1]     = (uint8)((y        >> 0) & 0x00FF);
    camera_rectangular_obj.width_buffer[0]      = (uint8)((width    >> 8) & 0x00FF);
    camera_rectangular_obj.width_buffer[1]      = (uint8)((width    >> 0) & 0x00FF);
    camera_rectangular_obj.height_buffer[0]     = (uint8)((height   >> 8) & 0x00FF);
    camera_rectangular_obj.height_buffer[1]     = (uint8)((height   >> 0) & 0x00FF);
    camera_rectangular_obj.color_buffer[0]      = (uint8)((color    >> 8) & 0x00FF);
    camera_rectangular_obj.color_buffer[1]      = (uint8)((color    >> 0) & 0x00FF);
    #else
    camera_rectangular_obj.x_axis_buffer[0]     = (uint8)((x        >> 0) & 0x00FF);
    camera_rectangular_obj.x_axis_buffer[1]     = (uint8)((x        >> 8) & 0x00FF);
    camera_rectangular_obj.y_axis_buffer[0]     = (uint8)((y        >> 0) & 0x00FF);
    camera_rectangular_obj.y_axis_buffer[1]     = (uint8)((y        >> 8) & 0x00FF);
    camera_rectangular_obj.width_buffer[0]      = (uint8)((width    >> 0) & 0x00FF);
    camera_rectangular_obj.width_buffer[1]      = (uint8)((width    >> 8) & 0x00FF);
    camera_rectangular_obj.height_buffer[0]     = (uint8)((height   >> 0) & 0x00FF);
    camera_rectangular_obj.height_buffer[1]     = (uint8)((height   >> 8) & 0x00FF);
    camera_rectangular_obj.color_buffer[0]      = (uint8)((color    >> 0) & 0x00FF);
    camera_rectangular_obj.color_buffer[1]      = (uint8)((color    >> 8) & 0x00FF);
    #endif

    // 和校验计算
    camera_rectangular_obj.check_sum            = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_rectangular_obj.check_sum           += seekfree_assistant_sum(
        &(camera_rectangular_obj.cmd),
        sizeof(camera_rectangular_obj) - 2);

    seekfree_assistant_transfer_callback(
        (const uint8 *)&camera_rectangular_obj,
        sizeof(camera_rectangular_obj));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 配置
// 参数说明     *oscilloscope_obj       示波器数据结构体
// 参数说明     channel_max             最大通道数量 范围是 [1, SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX] 超过范围会缩限到这个范围
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_config(oscilloscope_obj, SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_oscilloscope_config (
    seekfree_assistant_oscilloscope_struct *oscilloscope_obj,
    uint8 channel_max,
    void *data_buffer)
{
    zf_assert(NULL != oscilloscope_obj);
    zf_assert(NULL != data_buffer);

    memset(oscilloscope_obj, 0, sizeof(seekfree_assistant_oscilloscope_struct));
    oscilloscope_obj->config.head           = SEEKFREE_ASSISTANT_SEND_HEAD;
    oscilloscope_obj->config.cmd            = SEEKFREE_ASSISTANT_SEND_OSCILLOSCOPE_CMD;
    oscilloscope_obj->config.channel_max    = 
            (SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX <= channel_max)
        ?   (SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX)
        :   ((0 == channel_max) ? (1) : (channel_max));
    oscilloscope_obj->config.data_type      = (SEEKFREE_ASSISTANT_STC_DATA_ENABLE) ? (0x01) : (0x00);
    oscilloscope_obj->buffer_byte_size      = oscilloscope_obj->config.channel_max * 4;
    oscilloscope_obj->data_buffer           = data_buffer;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 发送函数
// 参数说明     *seekfree_assistant_oscilloscope  示波器数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_oscilloscope_send (seekfree_assistant_oscilloscope_struct *oscilloscope_obj)
{
    // 和校验计算
    oscilloscope_obj->config.check_sum  = SEEKFREE_ASSISTANT_SEND_HEAD;
    oscilloscope_obj->config.check_sum += seekfree_assistant_sum(
        &(oscilloscope_obj->config.cmd),
        sizeof(oscilloscope_obj->config) - 2);
    oscilloscope_obj->config.check_sum += seekfree_assistant_sum(
        (oscilloscope_obj->data_buffer),
        oscilloscope_obj->buffer_byte_size);

    seekfree_assistant_transfer_callback(
        (const uint8 *)oscilloscope_obj,
        sizeof(oscilloscope_obj->config));
    seekfree_assistant_transfer_callback(
        (const uint8 *)(oscilloscope_obj->data_buffer),
        oscilloscope_obj->buffer_byte_size);
}

#if (SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE)
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手 虚拟示波器 发送函数
// 参数说明     *debug_param_obj        示波器数据结构体
// 返回参数     void
// 使用示例     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_debug_param_analysis (seekfree_assistant_debug_param_struct *debug_param_obj)
{
    zf_assert(NULL != debug_param_obj);

    seekfree_assistant_debug_param_cmd_struct debug_param_cmd_obj;              // 新建一个数据包
    uint8   *debug_param_cmd_pointer = (uint8 *)&debug_param_cmd_obj;           // 用一个指针指向数据包方便操作
    uint8   temp_sum = 0xFF;                                                    // 校验和临时变量
    uint32  read_length = 0;                                                    // 读取长度用临时变量

    // 尝试读取一个数据包
    // 如果不是自定义的传输方式则从接收回调中读取数据
    read_length = seekfree_assistant_receive_callback(
        debug_param_cmd_pointer,
        sizeof(seekfree_assistant_debug_param_cmd_struct));
    if(read_length)
    {   // 如果有数据 将读取到的数据写入FIFO
        fifo_write_buffer(
            &seekfree_assistant_debug_param_fifo,
            debug_param_cmd_pointer,
            read_length);
    }

    while(  sizeof(seekfree_assistant_debug_param_cmd_struct)
        <=  fifo_used(&seekfree_assistant_debug_param_fifo))
    {   // 如果 FIFO 中数据量大于一个数据包 那么就进行解析
        // 直接读一个数据包长度
        read_length = sizeof(seekfree_assistant_debug_param_cmd_struct);
        fifo_read_buffer(
            &seekfree_assistant_debug_param_fifo,
            debug_param_cmd_pointer,
            &read_length,
            FIFO_READ_ONLY);

        if(     SEEKFREE_ASSISTANT_RECEIVE_HEAD == debug_param_cmd_pointer[0]
            &&  SEEKFREE_ASSISTANT_RECEIVE_DEBUG_PARAM_CMD == debug_param_cmd_pointer[2])
        {   // 直接匹配上帧头 并且功能字也匹配
            temp_sum = debug_param_cmd_obj.check_sum;                           // 读取数据包的校验和
            debug_param_cmd_obj.check_sum = 0;                                  // 清空包内校验和值 方便计算整包校验和
            if( temp_sum ==  seekfree_assistant_sum(debug_param_cmd_pointer, sizeof(seekfree_assistant_debug_param_cmd_struct)))
            {   // 和校验成功保存数据
                debug_param_obj->data[debug_param_cmd_obj.channel_index - 1] = debug_param_cmd_obj.dat.data_float;
                #if (SEEKFREE_ASSISTANT_STC_DATA_ENABLE)
                // 如果是 STC 那么大小端反一下
                debug_param_obj->dat.data_buffer[0] ^= debug_param_obj->dat.data_buffer[3];
                debug_param_obj->dat.data_buffer[3] ^= debug_param_obj->dat.data_buffer[0];
                debug_param_obj->dat.data_buffer[0] ^= debug_param_obj->dat.data_buffer[3];
                debug_param_obj->dat.data_buffer[1] ^= debug_param_obj->dat.data_buffer[2];
                debug_param_obj->dat.data_buffer[2] ^= debug_param_obj->dat.data_buffer[1];
                debug_param_obj->dat.data_buffer[1] ^= debug_param_obj->dat.data_buffer[2];
                #endif
                debug_param_obj->update_flag[debug_param_cmd_obj.channel_index - 1] = 1;
                temp_sum = 0x00;
            }
            else
            {   // 如果校验和匹配错误 置位临时校验和为 0xFF 表示解析错误
                temp_sum = 0xFF;
            }
        }
        else
        {   // 如果匹配不上帧头和功能字 置位临时校验和为 0xFF 表示解析错误
            temp_sum = 0xFF;
        }

        if(0xFF == temp_sum)
        {   // 如果跑完一遍包解析后临时校验和为 0xFF 那么就是解析错误 从数据中寻找下一个帧头
            for(read_length = 1; sizeof(seekfree_assistant_debug_param_cmd_struct) > read_length; read_length ++)
            {   // 因为读取的一整包 所以最大遍历一整包
                if(SEEKFREE_ASSISTANT_RECEIVE_HEAD == debug_param_cmd_pointer[read_length])
                {   // 找到下个帧头就退出
                    break;
                }
            }
        }

        // 丢弃无需使用的数据
        fifo_read_buffer(
            &seekfree_assistant_debug_param_fifo,
            debug_param_cmd_pointer,
            &read_length,
            FIFO_READ_AND_CLEAN);
    }
}
#endif

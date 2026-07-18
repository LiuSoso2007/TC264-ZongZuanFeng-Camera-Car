#ifndef __SHARED_H__
#define __SHARED_H__

#include <stdint.h>

/*
 * Shared.h --- CPU0 <> CPU1 跨核共享数据结构
 *
 * 跨核共享变量, 必须使用 volatile 修饰, 以保证数据可见性。
 * 本文件定义 CPU0(图像处理) 和 CPU1(运动控制) 之间的数据接口。
 *
 * 当前共享变量:
 *   Err:         CPU0 图像偏差 → CPU1 用于 PD 舵机控制 / 电机差速
 *   StopRequest: CPU0 斑马线锁存 → CPU1 双电机停车
 *   EncLeft/Right: CPU1 编码器采样 → CPU0 屏幕显示
 *
 * 后续扩展 (CAMERA.h 中的 Image_Process 完善后):
 *   ImageStatus: 图像状态 (Det_True 误差, OFFLine 丢线, 等)
 *   ImageFlag:   图像标志 (Bend_Road 弯道, Ramp 坡道,
 *                Zebra_Flag 斑马线, Rings 圆环, RoadBlock 路障, Out_Road 断路)
 */

extern volatile float Err;
extern volatile uint8_t StopRequest;
extern volatile int16_t EncLeft;
extern volatile int16_t EncRight;


/*
 * 图像状态 / 图像标志 (待CAMERA.h中Image_Process实现后启用):
 *
 *   ImageStatus.Det_True        - 当前误差 (int)
 *   ImageStatus.OFFLine         - 丢线行数 (int16)
 *
 *   ImageFlag.Bend_Road          - 弯道类型 0=直道, 1=左弯, 2=右弯
 *   ImageFlag.Ramp               - 坡道标志
 *   ImageFlag.Zebra_Flag         - 斑马线标志
 *   ImageFlag.RoadBlock_Flag     - 路障标志
 *   ImageFlag.Out_Road           - 断路标志
 *   ImageFlag.image_element_rings - 圆环标志
 */

#endif

#ifndef __SHARED_H__
#define __SHARED_H__

#include <stdint.h>
#include "IfxCpu.h"

/*
 * Shared.h --- CPU0 <> CPU1 跨核共享数据结构
 *
 * 跨核共享变量使用volatile防止编译器缓存；多变量一致性由原子锁协议保证。
 * 本文件定义 CPU0(图像处理) 和 CPU1(运动控制) 之间的数据接口。
 *
 * 当前共享变量:
 *   Err:         CPU0通过邮箱发布图像偏差，CPU1取出后用于PD舵机控制/电机差速
 *   StopRequest: CPU0 斑马线锁存 → CPU1 双电机停车
 *   RingEntrySlowdown: CPU0 圆环进环阶段 → CPU1 降低电机速度
 *   EncLeft/Right: CPU1 编码器采样 → CPU0 屏幕显示
 *
 * ImageStatus 和 ImageFlag 为 CPU0 的图像处理状态，定义在 Camera.h，
 * 当前不通过本文件跨核共享。
 */

extern volatile float Err;
extern volatile uint8_t ErrReady;
extern IfxCpu_mutexLock ErrMailboxLock;
extern volatile uint8_t StopRequest;
extern volatile uint8_t RingEntrySlowdown;
extern volatile int16_t EncLeft;
extern volatile int16_t EncRight;

/* CPU0覆盖最新帧控制量；Err和减速标志同锁发布，避免CPU1读到跨帧组合。 */
static inline void Shared_PublishErr(float err, uint8_t ring_entry_slowdown)
{
    while (IfxCpu_acquireMutex(&ErrMailboxLock) == FALSE)
    {
        /* CPU1临界区极短，等待其完成一次原子快照。 */
    }
    Err = err;
    RingEntrySlowdown = ring_entry_slowdown;
    ErrReady = 1U;
    IfxCpu_releaseMutex(&ErrMailboxLock);
}

/* CPU1非阻塞获取最新帧快照；锁忙或无新帧时保留上一帧控制量，下个10ms周期重试。 */
static inline uint8_t Shared_TakeErr(float *err, uint8_t *ring_entry_slowdown)
{
    uint8_t has_new_err = 0U;

    if (IfxCpu_acquireMutex(&ErrMailboxLock) != FALSE)
    {
        if (ErrReady != 0U)
        {
            *err = Err;
            *ring_entry_slowdown = RingEntrySlowdown;
            ErrReady = 0U;
            has_new_err = 1U;
        }
        IfxCpu_releaseMutex(&ErrMailboxLock);
    }

    return has_new_err;
}


#endif

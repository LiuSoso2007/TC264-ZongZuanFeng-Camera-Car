#ifndef __KEY_H__
#define __KEY_H__

#include "zf_common_headfile.h"

/*
 * Key.h --- 5键按键驱动接口
 *
 * 引脚: P10_7, P10_8, P10_9, P11_0, P11_1 (上拉输入, 按下为低)
 * 扫描周期: 5ms (PIT定时器驱动 Key_Tick)
 * 去抖时间: 10ms (连续2次扫描确认)
 *
 * 调用流程:
 *   Key_Init()   -- 初始化GPIO (上拉输入)
 *   Key_Tick()   -- 由 isr.c 5ms中断调用, 执行扫描+去抖
 *   Key_GetNum() -- 获取按键值 (1~5), 返回后自动清零
 */

void    Key_Init(void);
uint8_t Key_GetNum(void);
void    Key_Tick(void);

#endif
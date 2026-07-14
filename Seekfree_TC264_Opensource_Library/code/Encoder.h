#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "zf_common_headfile.h"

/*
 * Encoder.h --- 双路编码器接口
 *
 * 左编码器: TIM6 方向编码器模式 (encoder_dir_init)
 * 右编码器: TIM4 正交编码器模式 (encoder_quad_init)
 *
 * 调用流程:
 *   Encoder_Init()            -- 初始化硬件
 *   Encoder_Get_Left()        -- 读左编码器脉冲 (读后自动清零)
 *   Encoder_Get_Right()       -- 读右编码器脉冲 (读后自动清零, 已取反)
 */

void    Encoder_Init(void);
int16_t Encoder_Get_Left(void);
int16_t Encoder_Get_Right(void);

#endif
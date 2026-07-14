/******************************************************************************
 * Encoder.c - TC264 编码器驱动 (2路正交编码器)
 * 左编码器: TIM2, P33_7(CH1) + P33_6(CH2)  使用 encoder_dir_init 方向编码器
 * 右编码器: TIM4, P02_8(CH1) + P00_9(CH2)  使用 encoder_quad_init 正交编码器
 *
 * 注意: 右编码器返回值取反 (机械安装方向差异)
 ******************************************************************************/
#include "Encoder.h"

/* ---------- 引脚定义 (按实际接线修改) ---------- */
#define ENC_LEFT   TIM6_ENCODER    /* 左编码器: TIM6 方向编码器模式 */
#define ENC_RIGHT  TIM4_ENCODER    /* 右编码器: TIM4 正交编码器模式 */

/*
 * Encoder_Init - 双路编码器初始化
 * 左: encoder_dir_init (方向+脉冲模式, 仅需CH1)
 * 右: encoder_quad_init (正交解码模式, 4倍频)
 */
void Encoder_Init(void)
{
    /* 左编码器: 方向编码器, 仅用CH1计脉冲, 方向由IO判断 */
    encoder_dir_init(ENC_LEFT,
        TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    /* 右编码器: 正交编码器, CH1+CH2 双相4倍频 */
    encoder_quad_init(ENC_RIGHT,
        TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
}

/*
 * Encoder_Get_Left - 获取左编码器计数并清零
 * 返回: 本次采样周期内的脉冲数 (有符号)
 */
int16_t Encoder_Get_Left(void)
{
    int16_t count = encoder_get_count(ENC_LEFT);
    encoder_clear_count(ENC_LEFT);
    return count;
}

/*
 * Encoder_Get_Right - 获取右编码器计数并清零
 * 返回: 本次采样周期内的脉冲数 (有符号, 取反以统一方向)
 */
int16_t Encoder_Get_Right(void)
{
    int16_t count = encoder_get_count(ENC_RIGHT);
    encoder_clear_count(ENC_RIGHT);
    return - count;   /* 取反: 右电机反向安装, 统一正方向 */
}
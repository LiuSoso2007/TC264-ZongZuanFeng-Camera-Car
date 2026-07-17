/******************************************************************************
 * Encoder.c - TC264 编码器驱动 (2路正交编码器)
 * 左: TIM2, P33_7(CH1) + P33_6(CH2)
 * 右: TIM4, P02_8(CH1) + P00_9(CH2)
 ******************************************************************************/
#include "Encoder.h"

/* ---------- 引脚定义 (按实际接线修改) ---------- */
#define ENC_LEFT   TIM6_ENCODER
#define ENC_RIGHT  TIM4_ENCODER

void Encoder_Init(void)
{
    // TIM6 ??? encoder_quad_init, ? encoder_dir_init ??
    encoder_dir_init(ENC_LEFT,
        TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    encoder_quad_init(ENC_RIGHT,
        TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
}

int16_t Encoder_Get_Left(void)
{
    int16_t count = encoder_get_count(ENC_LEFT);
    encoder_clear_count(ENC_LEFT);
    return count;
}

int16_t Encoder_Get_Right(void)
{
    int16_t count = encoder_get_count(ENC_RIGHT);
    encoder_clear_count(ENC_RIGHT);
    return - count;
}

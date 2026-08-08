/******************************************************************************
 * Key.c - TC264 按键驱动 (5键 + PIT扫描去抖)
 * 引脚: P10_7, P10_8, P10_9, P11_0, P11_1 (上拉输入，按下为低)
 * 扫描周期: 5ms (PIT), 去抖: 10ms
 ******************************************************************************/
#include "Key.h"

/* ---- 引脚定义 (按实际接线修改) ---- */
#define KEY1_PIN  P10_7
#define KEY2_PIN  P10_8
#define KEY3_PIN  P10_9
#define KEY4_PIN  P11_0
#define KEY5_PIN  P11_1

static uint8_t Key_Num;

void Key_Init(void)
{
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY5_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

uint8_t Key_GetNum(void)
{
    uint8_t temp;
    if (Key_Num)
    {
        temp = Key_Num;
        Key_Num = 0;
        return temp;
    }
    return 0;
}

/* 扫描按键，返回键值1~5，无按键返回0 */
static uint8_t Key_Scan(void)
{
    if (gpio_get_level(KEY1_PIN) == 0) return 1;
    if (gpio_get_level(KEY2_PIN) == 0) return 2;
    if (gpio_get_level(KEY3_PIN) == 0) return 3;
    if (gpio_get_level(KEY4_PIN) == 0) return 4;
    if (gpio_get_level(KEY5_PIN) == 0) return 5;
    return 0;
}

/* PIT每5ms调用一次，内部/2 = 10ms去抖 */
void Key_Tick(void)
{
    static uint8_t count;
    static uint8_t curr_state, prev_state;

    count++;
    if (count >= 2)
    {
        count = 0;
        prev_state = curr_state;
        curr_state = Key_Scan();

        /* 检测松开（下降沿）：按键释放时返回键值 */
        if (curr_state == 0 && prev_state != 0)
        {
            Key_Num = prev_state;
        }
    }
}

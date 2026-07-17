/**
 * cpu1_main.c  ---  CPU1: ????(IMU??PD + ????PI) + ????
 *
 * ????:
 *   CPU0: ??????? + OTSU??? + ???? -> Err ??
 *   CPU1: IMU660RB??? + ??PD(IMU?????) + ??? + ??PI????
 *         + IPS200???? + ????
 *
 * ????: 10ms (CCU61_CH0 PIT????, isr.c??PID_Flag)
 * ????: 5ms  (CCU60_CH1 PIT????, isr.c???Key_Tick)
 *
 * ????: Err (volatile, Shared.h??)
 *   Err: ???? (CPU0??, CPU1??, ????PD???PI)
 */

#include "zf_common_headfile.h"
#include "IPS200.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "imu.h"
#include "isr.h"

volatile uint8_t PID_Flag = 0;       /* PID?????? (isr.c 10ms????) */

#pragma section all "cpu1_dsram"

/* ---- CPU1???? ---- */
static int8_t   g_StraightSpeed = 50;    /* ?????? */
static int16_t  g_EncLeft       = 0;     /* ??????? (???) */
static int16_t  g_EncRight      = 0;     /* ??????? (???) */
static int16_t  EncCount        = 0;     /* ????????? */
static uint8_t  g_IMU_Ok        = 0;     /* IMU??????? */
static PI_t     s_PI_Left, s_PI_Right;   /* ????PI??? */

/* ---- LCD???? ---- */
#define LCD_DIV      10
#define LCD_LABEL_X  10
#define LCD_VALUE_X  100
#define LCD_Y_BASE   62
#define LCD_ROW_H    16

/* ---- ??PD?? (?????????) ---- */
#define PD_KP    2.5f       /* ??PD: Err->?????????? */
#define PD_KD    0.8f       /* ??PD: ?????????? */

float IMU_Yaw = 0.0f;        /* ??: IMU??Z???? (PID.c?PD_Update??) */

int core1_main(void)
{
    int16_t  enc_left, enc_right;
    int8_t   pwm_left,  pwm_right;
    float    position_err;
    static uint8_t lcd_cnt = 0, lcd_row = 0, lcd_dirty = 0;

    /* ---- CPU1??? ---- */
    disable_Watchdog();
    interrupt_global_enable(0);

    /* ---- ????? ---- */
    Encoder_Init();                      /* ???: TIM6(???)/TIM4(???) */
    Motor_Init();                        /* ????PWM: ATOM0_CH0/2(?) + ATOM1_CH1+ATOM0_CH3(?) */
    Servo_Init();                        /* ??50Hz PWM: ATOM0_CH1_P33_9 */
    Key_Init();                          /* 5?GPIO??: P10_7~P11_1 */
    IPS200_Init();                       /* IPS200 SPI???: 320x240 */

    /* ---- IMU660RB?????? ---- */
    g_IMU_Ok = !IMU_Init();
    if (g_IMU_Ok)
    {
        system_delay_ms(50);
        IMU_Calibrate(200);             /* ??200????????? */
    }
    /* IMU??????PD????P?? */

    /* ---- PI?????? ---- */
    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* ---- ???LCD???? ---- */
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*0, "L_Act:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*1, "L_Tar:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*2, "R_Act:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*3, "R_Tar:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*4, "Yaw:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*5, "Speed:");
    ips200_show_string(LCD_LABEL_X, LCD_Y_BASE+LCD_ROW_H*6, "Err:");

    /* ---- PIT?????? ---- */
    pit_ms_init(CCU60_CH1, 5);          /* ????: 5ms */
    pit_ms_init(CCU61_CH0, 10);         /* PID??:  10ms */

    /* ---- ??CPU0????? ---- */
    cpu_wait_event_ready();
    system_delay_ms(100);

    if (g_IMU_Ok)
    {
        IMU_Update();
        IMU_ClearAngleZ();              /* ??Z????? */
    }

    /* ---- ??? ---- */
    while (TRUE)
    {
        /* ---- ???? (?????, ??????) ---- */
        {
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;               /* ?????? */
        }

        /* ---- IMU??????? ---- */
        if (g_IMU_Ok)
        {
            IMU_Update();
        }

        /* ---- ??10ms???? ---- */
        if (!PID_Flag)
        {
            /* ????????LCD */
            if (++lcd_cnt >= LCD_DIV) { lcd_cnt = 0; lcd_row = 0; lcd_dirty = 1; }
            continue;
        }
        PID_Flag = 0;

        /* ---- IMU???? ---- */
        if (g_IMU_Ok)
        {
            IMU_Yaw = IMU_GetYawRate();   /* ??Z???? (?/s) */
            IMU_IntegrateZ(0.01f);        /* Z????? (dt=10ms) */
        }

        /* ---- ??????? (????, 8??????) ---- */
        EncCount++;
        if (EncCount >= 8)
        {
            EncCount = 0;
            enc_left  = Encoder_Get_Left();
            enc_right = Encoder_Get_Right();
        }
        g_EncLeft  = enc_left;
        g_EncRight = enc_right;

        /* ---- ?????? (CPU0??) ---- */
        position_err = Err;

        /* ---- ??PD?? (IMU?????, ??????) ---- */
        PD_Update(PD_KP, PD_KD);

        /* ---- ????: ???????????????? ---- */
        if      (position_err >   7.0f && position_err <  15.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 0.5f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.1f);
        }
        else if (position_err >= 15.0f && position_err <  25.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 1.1f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.5f);
        }
        else if (position_err >= 25.0f) {
            s_PI_Left.TargetBias  = (int16_t)(-g_StraightSpeed * 1.5f);
            s_PI_Right.TargetBias = (int16_t)( g_StraightSpeed * 0.8f);
        }
        else if (position_err <  -7.0f && position_err > -14.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.2f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.4f);
        }
        else if (position_err <= -14.0f && position_err > -20.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.3f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.55f);
        }
        else if (position_err <= -20.0f) {
            s_PI_Left.TargetBias  = (int16_t)( g_StraightSpeed * 0.3f);
            s_PI_Right.TargetBias = (int16_t)(-g_StraightSpeed * 0.65f);
        }
        else {
            s_PI_Left.TargetBias  = 0;
            s_PI_Right.TargetBias = 0;
        }

        /* ---- ??PI???? ---- */
        pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  g_StraightSpeed);
        pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, g_StraightSpeed);

        /* ---- ??PWM (PI_Update?????, Motor?????) ---- */
        Motor_SetLeftPWM(pwm_left);
        Motor_SetRightPWM(pwm_right);

        /* ---- LCD??????? ---- */
        if (++lcd_cnt >= LCD_DIV) { lcd_cnt = 0; lcd_row = 0; lcd_dirty = 1; }

        /* ---- LCD???? (???????????SPI??) ---- */
        if (lcd_dirty)
        {
            ips200_set_color(RGB565_BLUE, RGB565_WHITE);
            switch (lcd_row)
            {
                case 0: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*0, (int32)g_EncLeft,  5); break;
                case 1: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*1, (int32)s_PI_Left.TargetSpeed, 5); break;
                case 2: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*2, (int32)g_EncRight, 5); break;
                case 3: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*3, (int32)s_PI_Right.TargetSpeed, 5); break;
                case 4: ips200_show_float(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*4, IMU_Yaw, 3, 2); break;
                case 5: ips200_show_int(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*5, (int32)g_StraightSpeed, 5); break;
                case 6: ips200_show_float(LCD_VALUE_X, LCD_Y_BASE+LCD_ROW_H*6, Err, 3, 2); break;
            }
            if (++lcd_row >= 7) { lcd_row = 0; lcd_dirty = 0; }
        }
    }
}

#pragma section all restore

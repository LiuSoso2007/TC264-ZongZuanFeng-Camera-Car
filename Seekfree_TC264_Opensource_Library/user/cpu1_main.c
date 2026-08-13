/**
 * CPU1: 杩愬姩鎺у埗
 *
 * CPU0: 鍥惧儚閲囬泦涓庡鐞嗭紝杈撳嚭璧涢亾鍋忓樊Err涓庡厓绱犳爣蹇?
 * CPU1: 缂栫爜鍣ㄣ€佽埖鏈篜D銆佺數鏈篜I閫熷害鐜紝鎺у埗鍛ㄦ湡10ms
 * 鎺у埗瀹氭椂鍣? CCU61_CH0 PIT 10ms锛堜腑鏂湪isr.c涓級
 */

#include "zf_common_headfile.h"
#include "IPS200.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Servo.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

/* PID_Flag锛氱敱isr.c涓璫c61_pit_ch0涓柇缃?锛屾湰鍑芥暟澶勭悊鍚庢竻闆?*/
volatile uint8_t PID_Flag = 0;

/* CPU1鏈湴鍙橀噺锛欳PU0鍙鐢ㄤ簬鏄剧ず锛屾棤闇€浜掓枼閿?*/
volatile int16_t EncLeft  = 0;
volatile int16_t EncRight = 0;

#pragma section all "cpu1_dsram"   /* CPU1绉佹湁鍙橀噺鏀惧叆DSRAM娈?*/

/* CPU1鏈湴鍙傛暟锛堝悗缁彲鐢ㄦ寜閿?IMU璋冩暣锛?*/
static int8_t   StraightSpeed = 50;
static int16_t  EncCount        = 0;

/* 杩涚幆淇濈暀閫熷害鐧惧垎姣旓細60琛ㄧず淇濈暀鍘熼€熷害60%锛屾暟鍊艰秺澶ц秺蹇紝瓒婂皬瓒婃參銆?*/
#define RING_ENTRY_SPEED_PERCENT 60
#if RING_ENTRY_SPEED_PERCENT < 0 || RING_ENTRY_SPEED_PERCENT > 100
#error "RING_ENTRY_SPEED_PERCENT must be between 0 and 100"
#endif

/* PI鍙傛暟 */
#define PI_KP          0.4f
#define PI_KI          0.02f
#define CURVE_SPEED    0

/* PD鍙傛暟 */
#define PD_KP          0.78f
#define PD_KD          1.0f  /* 降低帧间Err跳变的随机微分冲击，保留快速入弯预判。 */

/* 宸﹀彸鐢垫満PI鎺у埗鍣?*/
static PI_t s_PI_Left, s_PI_Right;

/* CPU1鍏ュ彛鍑芥暟 */
int core1_main(void)
{
    /* CPU1鍒濆鍖栵細鍏抽棴鐪嬮棬鐙楀苟寮€鎬讳腑鏂?*/
    disable_Watchdog();
    interrupt_global_enable(0);

    int16_t  enc_left = 0, enc_right = 0;
    int8_t   pwm_left,  pwm_right;
    int16_t  motor_speed;
    float    position_err = 0.0f;
    float    new_position_err;
    uint8_t  ring_entry_slowdown = 0U;
    uint8_t  new_ring_entry_slowdown;

    /* CPU1澶栬鍒濆鍖?*/
    Key_Init();                          /* 鍥涢敭鎸夐敭锛堝姛鑳介鐣欙級 */
    Encoder_Init();                      /* 缂栫爜鍣細宸IM6/鍙砊IM4 */
    Motor_Init();                        /* 鐢垫満鍙屾瀬鎬WM(ATOM0) */
    Servo_Init();                        /* 鑸垫満50Hz PWM(ATOM0) */

    PI_Init(&s_PI_Left,  PI_KP, PI_KI, CURVE_SPEED);
    PI_Init(&s_PI_Right, PI_KP, PI_KI, CURVE_SPEED);

    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);

    /* 鎸夐敭鎵弿瀹氭椂鍣細5ms锛圕PU1 PIT锛?*/
    pit_ms_init(CCU60_CH1, 5);

    /* 鎺у埗鍛ㄦ湡瀹氭椂鍣細10ms锛屼腑鏂敱CPU1澶勭悊 */
    pit_ms_init(CCU61_CH0, 10);

    /* 绛夊緟CPU0灏辩华 */
    cpu_wait_event_ready();

    while (TRUE)
    {
        {
            /* 鎸夐敭鎵弿锛堟殏鏈粦瀹氬姛鑳斤級 */
            uint8_t KeyNum = Key_GetNum();
            (void)KeyNum;
        }

        /* 鏂癊rr鍒拌揪鍚庣珛鍗虫洿鏂拌埖鏈猴紝閬垮厤棰濆绛夊緟鏈€闀?0ms鎺у埗鍛ㄦ湡銆?*/
        if (ErrReady != 0U && StopRequest == 0U)
        {
            uint8_t has_new_err = 0U;
            if (Shared_TakeErr(&new_position_err, &new_ring_entry_slowdown))
            {
                position_err = new_position_err;
                ring_entry_slowdown = new_ring_entry_slowdown;
                has_new_err = 1U;
            }
            if (has_new_err != 0U && StopRequest == 0U)
            {
                PD_Update(PD_KP, PD_KD, position_err);
            }
        }

        /* 鐢垫満涓庣紪鐮佸櫒浠嶄繚鎸?0ms鎺у埗鍛ㄦ湡銆?*/
        if (!PID_Flag)
        {
            continue;
        }
        PID_Flag = 0;

        /* 缂栫爜鍣ㄨ鍙栵細姣?涓帶鍒跺懆鏈熼噰鏍蜂竴娆?*/
        EncCount ++;
        if(EncCount >= 8)
        {
             EncCount = 0;
             enc_left  = Encoder_Get_Left();
             enc_right = Encoder_Get_Right();
        }

        EncLeft  = enc_left;
        EncRight = enc_right;

        /* 璧涢亾璇樊宸插湪涓诲惊鐜叆鍙ｅ嵆鏃惰幏鍙栵紝鏃犳柊甯ф椂淇濇寔涓婁竴浠藉揩鐓с€?*/
        uint8_t Err_abs = 0U;
        if(position_err>0)Err_abs=position_err;
        if(position_err<0)Err_abs=-position_err;

        /* CPU0璇嗗埆鍒版枒椹嚎骞堕攣瀹氬悗锛屼緷娆＄疆闆禤WM鍜孭I鍋忕疆锛岀劧鍚庤缃埖鏈轰腑浣嶅仠杞︺€?*/
        if (StopRequest != 0U)
        {
            pwm_left = 0;
            pwm_right = 0;
            s_PI_Left.TargetBias = 0;
            s_PI_Right.TargetBias = 0;
            Motor_SetLeftPWM(0);
            Motor_SetRightPWM(0);
            Servo_SetAngleDeg(SERVO_CENTER_ANGLE);
            continue;
        }

        /* 閫熷害PI闂幆 */
        pwm_left  = PI_Update(&s_PI_Left,  position_err, enc_left,  StraightSpeed);
        pwm_right = PI_Update(&s_PI_Right, position_err, enc_right, StraightSpeed);

        /* 50FPS下按每像素1点主动降速，为连续弯和换向弯保留足够修正帧。 */
        motor_speed = (int16_t)((float)StraightSpeed - 1.0f * (float)Err_abs);
        if (motor_speed < CURVE_SPEED) motor_speed = CURVE_SPEED;
        /* 杩涘叆鍦嗙幆鏃舵寜淇濈暀姣斾緥闄嶉€?*/
        if (ring_entry_slowdown != 0U)
        {
            motor_speed = (int16_t)(motor_speed * RING_ENTRY_SPEED_PERCENT / 100);
        }

        /* 鍙屽悜杈撳嚭缁熶竴闄愬埗鍦?100~100锛岄槻姝㈣皟鍙傚悗瓒婅繃鐢垫満PWM杈圭晫銆?*/
        if (motor_speed > 100)  motor_speed = 100;
        if (motor_speed < -100) motor_speed = -100;
        Motor_SetLeftPWM((int8_t)motor_speed);
        Motor_SetRightPWM((int8_t)motor_speed);

    }
}

#pragma section all restore

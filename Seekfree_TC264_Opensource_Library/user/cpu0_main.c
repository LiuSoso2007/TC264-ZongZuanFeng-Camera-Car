/**
 * cpu0_main.c  ---  CPU0: ????? + ???? (???)
 *
 * ????:
 *   CPU0: ?????, Camera_Init (MT9V03X ???), ????
 *   CPU1: ?????, ?? PD, ?? PI ???? (????, 10ms)
 *         + LCD ?? + ???? (??) + IMU ???
 *
 * ?????? (??????, ????????):
 *   Err:     CPU0 ??????, CPU1 ????
 *   ImageStatus / ImageFlag: ??????? (??? CAMERA.h)
 *
 * CPU1 ???: cpu1_main.c
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

/* ----------------------------------------------------
 * ????: CPU0 -> CPU1 ????
 * ---------------------------------------------------- */
volatile float    Err             = 0.0f;

#pragma section all "cpu0_dsram"   /* ---- CPU0 ?????? ---- */

#define PRINT_DIV    20

int core0_main(void)
{
    static uint8_t print_cnt = 0, print_due = 0;

    /* ---- ????? ---- */
    clock_init();
    debug_init();
    system_delay_ms(100);

    /* ---- ?????? (??? MT9V03X) ---- */
    Camera_Init();

    /*
     * TODO: ???? Image_CompressInit() ???????
     * Image_CompressInit();
     */

    /* ---- ?? CPU1 ?? ---- */
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* ---- ?????? ---- */
        if (print_due)
        {
            print_due = 0;
            printf("Err=%.2f\r\n", (double)Err);
            /*
             * TODO: ???? CAMERA.h ???????:
             * printf("Err=%.2f  OffLine=%d  Bend=%d  Ring=%d\r\n",
             *        (double)Err,
             *        (int)ImageStatus.OFFLine,
             *        (int)ImageFlag.Bend_Road,
             *        (int)ImageFlag.image_element_rings);
             */
        }

        /* ---- ???? (???? CAMERA.c, ???) ---- */
        if (Camera_IsFrameReady())
        {
            /*
             * TODO: ???? Image_Process() ???:
             *
             * Image_Process() ????:
             *   1. Get_BinaryImage()   - ??????
             *   2. Get_BaseLine()      - ????
             *   3. Get_AllLine()       - ????
             *   4. Scan_Element()      - ???? (??/???/??/??)
             *   5. Element_Handle()    - ???? (??/??)
             *   ??: Err = ImageStatus.Det_True (??????)
             *        ImageFlag / ImageStatus ????? CPU1 ??
             *
             * Image_Process();
             */
        }

        if (++print_cnt >= PRINT_DIV) { print_cnt = 0; print_due = 1; }
    }
}

#pragma section all restore

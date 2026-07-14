/**
 * cpu0_main.c  ---  CPU0: Camera + IPS200 ????
 */

#include "zf_common_headfile.h"
#include "Camera.h"
#include "IPS200.h"
#include "PID.h"
#include "Shared.h"
#include "isr.h"

volatile float    Err             = 0.0f;

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    system_delay_ms(100);

    interrupt_global_enable(1);

    Camera_Init();
    Camera_CompressInit();       // ??????? (?????)

    cpu_wait_event_ready();

    while (TRUE)
    {
        if (Camera_IsFrameReady())
        {
            /*
             * ?????: ???? -> OTSU??? -> ???????? -> IPS200????
             * Camera_ShowDebug() ????:
             *   ??: ?????? 188x120
             *   ??: OTSU??????
             *   ??: ???? 94x60
             */
            Camera_GetBinaryImage();
            Camera_ShowDebug();
        }
    }
}

#pragma section all restore
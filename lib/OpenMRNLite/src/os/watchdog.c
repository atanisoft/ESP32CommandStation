/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file watchdog.c
 *
 * Implements a simple software watchdog that runs a thread which terminates if
 * the watchdog was not reset in the given period.
 *
 * @author Balazs Racz
 * @date 26 May 2013
 */

#define _DEFAULT_SOURCE

#include <unistd.h>

#include "os/watchdog.h"
#include "os/os.h"


/// What is the timeout of the watchdog in milliseconds.
static int watchdog_period_msec;
/// How many times have we seen the watchdog tick without being fed.
static int watchdog_ticks = 0;

/// Thread running a watchdog.
static void* watchdog_thread(void* arg)
{
    while (1)
    {
        usleep(((useconds_t)1000) * watchdog_period_msec);
        if (++watchdog_ticks > 1)
        {
#ifdef __FreeRTOS__
            diewith(BLINK_DIE_WATCHDOG);
#else
            abort();
#endif
        }
    }
    return NULL;
}

void start_watchdog(int period_msec)
{
    watchdog_period_msec = period_msec;
    reset_watchdog();
#ifdef __FreeRTOS__
    const int kStackSize = 256;
#else
    const int kStackSize = 2048;
#endif
    os_thread_create(NULL, "watchdog", 0, kStackSize,
                     &watchdog_thread, NULL);
}

void reset_watchdog(void)
{
    watchdog_ticks = 0;
}

#if 0
static long long watchdog_reset_timer(void* unused1, void* unused2)
{
    reset_watchdog();
    return OS_TIMER_RESTART;
}
#endif

void add_watchdog_reset_timer(int period_msec)
{
    //os_timer_start(os_timer_create(&watchdog_reset_timer, NULL, NULL),
    //               MSEC_TO_NSEC(period_msec));
}

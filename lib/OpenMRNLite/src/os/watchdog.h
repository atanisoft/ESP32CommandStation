/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file watchdog.h
 * Interface for a software watchdog that aborts if not reset periodically.
 *
 * @author Balazs Racz
 * @date 25 May 2013
 */

#ifndef _OS_WATCHDOG_H_
#define _OS_WATCHDOG_H_

#ifdef __cplusplus
extern "C" {
#endif

/// Starts a watchdog thread that must be reset more often than period_msec.
void start_watchdog(int period_msec);
/// Resets the watchdog.
void reset_watchdog(void);
/**
   Inserts a timer that will periodically reset the watchdog, thereby
   practically making the watchdog watch the timer thread. This makes it
   possible to add timers that watch other resources for not overflowing.

   @param period_msec tells how often to reset the watchdog.
*/
void add_watchdog_reset_timer(int period_msec);

#ifdef __cplusplus
}
#endif

#endif // _OS_WATCHDOG_H_

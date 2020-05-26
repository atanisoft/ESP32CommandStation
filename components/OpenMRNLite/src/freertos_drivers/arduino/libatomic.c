/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file libatomic.c
 *
 * A partial implementation of libatomic for Cortex-M0 for the necessary
 * operations in OpenMRN.
 *
 * @author Balazs Racz
 * @date 30 Dec 2019
 */

#include <stdint.h>

#if defined(STM32F0xx) || (!defined(ARDUINO) && !defined(ESP32))
// On Cortex-M0 the only way to do atomic operation is to disable interrupts.

/// Disables interrupts and saves the interrupt enable flag in a register.
#define ACQ_LOCK()                                                             \
    int _pastlock;                                                             \
    __asm volatile(" mrs %0, PRIMASK \n cpsid i\n" : "=r"(_pastlock));

/// Restores the interrupte enable flag from a register.
#define REL_LOCK() __asm volatile(" msr PRIMASK, %0\n " : : "r"(_pastlock));

uint16_t __atomic_fetch_sub_2(uint16_t *ptr, uint16_t val, int memorder)
{
    ACQ_LOCK();
    uint16_t ret = *ptr;
    *ptr -= val;
    REL_LOCK();
    return ret;
}

uint8_t __atomic_exchange_1(uint8_t *ptr, uint8_t val, int memorder)
{
    ACQ_LOCK();
    uint8_t ret = *ptr;
    *ptr = val;
    REL_LOCK();
    return ret;
}

#endif // guard for arduino compilation

/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * @file os/stack_malloc.c
 * This file provides a link time way to allocate RAM bubers.
 *
 * @author Balazs Racz
 * @date 24 June 2014
 */
#include <stdlib.h>

#if defined(__FreeRTOS__)
const void *__attribute__((weak)) stack_malloc(unsigned long length);

const void *stack_malloc(unsigned long length)
{
    /* We do a trick here to ensure that the compiler will output a stack frame
     * for this function. We want to avoid tail-chain optimization in this
     * function or else it disappears from the stack traces done for memory
     * tracing. */
    void *volatile v = malloc(length);
    return v;
}
#endif

void *buffer_malloc(size_t length) __attribute__((weak));

void *buffer_malloc(size_t length)
{
    /* We do a trick here to ensure that the compiler will output a stack frame
     * for this function. We want to avoid tail-chain optimization in this
     * function or else it disappears from the stack traces done for memory
     * tracing. */
    void *volatile v = malloc(length);
    return v;
}

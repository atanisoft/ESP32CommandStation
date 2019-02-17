/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file DeviceBuffer.hxx
 * This file provides a buffer class that is useful in the construction of
 * FreeRTOS device drivers.
 *
 * @author Stuart W. Baker
 * @date 2 March 2015
 */

#include "DeviceBuffer.hxx"

#ifndef ARDUINO

#include <sys/select.h>

/** Block until the wait condition is true.  The condition is defined by
 * the user of the buffer and could be that there is data in the buffer or
 * it could be that there is room in the buffer.  In any case, this method
 * should be called only when the buffer is locked within a critical
 * section.
 *
 * Internally the lock is released before blocking to prevent
 * deadlock.  The lock is grabbed once again before the method returns.
 * If multiple threads are waiting on the same condition, there is a race
 * between them as to who will consume the condition first.  Any thread(s)
 * loosing that race would typically make another call to
 * @ref block_until_condition() until another wakeup condition occurs.
 */
void DeviceBufferBase::block_until_condition(File *file, bool read)
{
    fd_set fds;
    FD_ZERO(&fds);
    int fd = Device::fd_lookup(file);
    FD_SET(fd, &fds);

    ::select(fd + 1, read ? &fds : NULL, read ? NULL : &fds, NULL, NULL);
}

#endif

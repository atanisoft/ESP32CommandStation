/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file MemoryConfig.cxx
 *
 * Implementation of the Memory Config Protocol server
 *
 * @author Balazs Racz
 * @date 23 Feb 2014
 */

#include "openlcb/MemoryConfig.hxx"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "utils/logging.h"
#ifdef __FreeRTOS__
#include "can_ioctl.h"
#endif

extern "C" {
/// Implement this function (usually in HwInit.cxx) to enter the
/// bootloader. Usual implementations write some magic value to RAM and the
/// reboot the MCU.
void enter_bootloader() __attribute__ ((weak));
void enter_bootloader()
{
}

#if !defined (__MACH__)
/// Implement this function (usually in HwInit.cxx) to reboot the MCU.
void reboot() __attribute__ ((weak));
void reboot()
{
}
#endif
}

namespace openlcb
{

FileMemorySpace::FileMemorySpace(int fd, address_t len)
    : fileSize_(len)
    , name_(nullptr)
    , fd_(fd)
{
    HASSERT(fd_ >= 0);
}

FileMemorySpace::FileMemorySpace(const char *name, address_t len)
    : fileSize_(len)
    , name_(name)
    , fd_(-1)
{
    HASSERT(name_);
}

void FileMemorySpace::ensure_file_open()
{
    if (fd_ < 0)
    {
        int opts = 0;
        if (read_only()) {
            opts = O_RDONLY;
        } else {
            opts = O_RDWR;
        }
#ifdef __FreeRTOS__
        opts |= O_NONBLOCK;
#endif
        fd_ = open(name_, opts);
        if (fd_ < 0)
        {
            LOG(WARNING, "Error opening file %s : %s", name_, strerror(errno));
            return;
        }
        HASSERT(fd_ >= 0);
    }
    if (fileSize_ == AUTO_LEN)
    {
        struct stat buf;
        HASSERT(fstat(fd_, &buf) >= 0);
        fileSize_ = buf.st_size;
    }
}

size_t FileMemorySpace::write(address_t destination, const uint8_t *data,
                              size_t len, errorcode_t *error, Notifiable *again)
{
    ensure_file_open();
    if (fd_ < 0)
    {
        *error = Defs::ERROR_PERMANENT;
        return 0;
    }
    off_t actual_position = lseek(fd_, destination, SEEK_SET);
    if ((address_t)actual_position != destination)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        return 0;
    }
    ssize_t ret = ::write(fd_, data, len);
    if (ret < 0)
    {
        LOG(INFO, "Error writing to fd %d: %s", fd_, strerror(errno));
        *error = Defs::ERROR_PERMANENT;
        return 0;
    }
    else if ((size_t)ret < len)
    {
#ifdef __FreeRTOS__
        *error = ERROR_AGAIN;
        HASSERT(ioctl(fd_, CAN_IOC_WRITE_ACTIVE, again) == 0);
#endif
        return ret;
    }
    else
    {
        return ret;
    }
}

size_t FileMemorySpace::read(address_t destination, uint8_t *dst, size_t len,
                             errorcode_t *error, Notifiable *again)
{
    ensure_file_open();
    if (fd_ < 0)
    {
        *error = Defs::ERROR_PERMANENT;
        return 0;
    }
    off_t actual_position = lseek(fd_, destination, SEEK_SET);
    if ((address_t)actual_position != destination)
    {
        *error = Defs::ERROR_PERMANENT;
        return 0;
    }
    if (destination >= fileSize_)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        return 0;
    }
    if (destination + len > fileSize_)
    {
        len = fileSize_ - destination;
    }
    ssize_t ret = ::read(fd_, dst, len);
    if (ret < 0)
    {
        LOG(INFO, "Error reading from fd %d: %s", fd_, strerror(errno));
        *error = Defs::ERROR_PERMANENT;
        return 0;
    }
    else if ((size_t)ret < len)
    {
#ifdef __FreeRTOS__
        *error = ERROR_AGAIN;
        HASSERT(ioctl(fd_, CAN_IOC_READ_ACTIVE, again) == 0);
#endif
        return ret;
    }
    else
    {
        return ret;
    }
}

} // namespace openlcb

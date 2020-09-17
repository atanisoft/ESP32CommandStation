/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file ArduinoFs.hxx
 *
 * Single-file POSIX compatible filesystem using Arduino's EEPROM
 * implementation for STM32.
 *
 * This file needs to be included into the .ino of the arduino sketch. It may
 * only be compiled once as it has definitions of C library functions.
 *
 * @author Balazs Racz
 * @date 18 July 2020
 */

#ifndef _FREERTOS_DRIVERS_ARDUINO_ARDUINOFS_HXX_
#define _FREERTOS_DRIVERS_ARDUINO_ARDUINOFS_HXX_

#ifndef ARDUINO_ARCH_STM32
#error This module only works for STM32 boards.
#endif

#ifndef OPENMRN_HAVE_POSIX_FD
#error You must add a file build_opt.h to the sketch directory and add -DOPENMRN_HAVE_POSIX_FD to it to make use of this module.
#endif

#include "stm32_eeprom.h"
#include <errno.h>

#define EEPROM_FILENAME "/dev/eeprom"

/// Class that holds information and code for the single-file filesystem for
/// emulating eeprom.
class FsStatic
{
public:
    /// Number of possible open file desriptors.
    static constexpr unsigned MAX_FD = 8;
    /// Offset markng that a file descriptor is not in use.
    static constexpr off_t UNUSED_FILE = (off_t)-1;

    /// We have one of these for each open file descriptor.
    struct FileInfo
    {
        /// POSIX file offset.
        off_t offset = UNUSED_FILE;

        /// @return true if this file descriptor is in use.
        bool in_use()
        {
            return offset != UNUSED_FILE;
        }

        /// Marks the file descriptor to be in use.
        void open()
        {
            offset = 0;
        }

        /// Marks the file descriptor to be not in use.
        void close()
        {
            offset = UNUSED_FILE;
        }
    };

    /// Stores all file descriptors.
    static FileInfo fds[MAX_FD];

    /// Lookup a file descriptor.
    /// @param fd the file descriptor.
    /// @return nullptr if fd is an invalid file descriptor (in which case also
    /// sets errno), otherwise the file descriptor structure.
    static FileInfo *get_file(int fd)
    {
        if (fd >= MAX_FD || !fds[fd].in_use())
        {
            errno = EBADF;
            return nullptr;
        }
        return &fds[fd];
    }

    /// Allocates a new file descriptor.
    /// @return new fd. If there is no free file descriptor, returns -1 and
    /// sets errno.
    static int new_fd()
    {
        for (int fd = 0; fd < MAX_FD; ++fd)
        {
            if (!fds[fd].in_use())
            {
                fds[fd].open();
                return fd;
            }
        }
        errno = ENFILE;
        return -1;
    }

    /// If there is unflushed writes, performs the flash write.
    static void flush_if_dirty()
    {
        if (dirty_)
        {
            eeprom_buffer_flush();
            dirty_ = 0;
        }
    }

    /// 1 if we have filled the eeprom buffer at least once since startup.
    static uint8_t loaded_;
    /// 1 if we have unflushed written data in the eeprom buffer.
    static uint8_t dirty_;
};

FsStatic::FileInfo FsStatic::fds[FsStatic::MAX_FD];
uint8_t FsStatic::loaded_ = 0;
uint8_t FsStatic::dirty_ = 0;

extern "C"
{

int _open_r(struct _reent *reent, const char *path, int flags, int mode)
{
    if (strcmp(path, EEPROM_FILENAME) != 0)
    {
        errno = ENOENT;
        return -1;
    }
    if (!FsStatic::loaded_)
    {
        eeprom_buffer_fill();
        FsStatic::loaded_ = 1;
    }
    return FsStatic::new_fd();
}

int _close_r(struct _reent *reent, int fd)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    finfo->close();
    FsStatic::flush_if_dirty();
    return 0;
}

ssize_t _read_r(struct _reent *reent, int fd, void *buf, size_t count)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    ssize_t ret = 0;
    uint8_t *dst = (uint8_t *)buf;
    int left = (int)E2END - (int)finfo->offset;
    if (left < 0)
    {
        left = 0;
    }
    if (left < count)
    {
        count = left;
    }
    while (count > 0)
    {
        *dst = eeprom_buffered_read_byte(finfo->offset);
        ++dst;
        ++finfo->offset;
        --count;
        ++ret;
    }
    return ret;
}

ssize_t _write_r(struct _reent *reent, int fd, const void *buf, size_t count)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    ssize_t ret = 0;
    const uint8_t *src = (const uint8_t *)buf;
    int left = (int)E2END - (int)finfo->offset;
    if (left < 0)
    {
        left = 0;
    }
    if (left < count)
    {
        count = left;
    }
    if (count)
    {
        FsStatic::dirty_ = 1;
    }
    while (count > 0)
    {
        eeprom_buffered_write_byte(finfo->offset, *src);
        ++src;
        ++finfo->offset;
        --count;
        ++ret;
    }
    return ret;
}

int fsync(int fd)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    FsStatic::flush_if_dirty();
    return 0;
}

int _stat_r(struct _reent *reent, const char *path, struct stat *stat)
{
    if (strcmp(path, EEPROM_FILENAME) != 0)
    {
        errno = ENOENT;
        return -1;
    }
    memset(stat, 0, sizeof(*stat));
    stat->st_size = E2END;
    return 0;
}

int _fstat_r(struct _reent *reent, int fd, struct stat *stat)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    memset(stat, 0, sizeof(*stat));
    stat->st_size = E2END;
    return 0;
}

_off_t _lseek_r(struct _reent *reent, int fd, _off_t offset, int whence)
{
    FsStatic::FileInfo *finfo = FsStatic::get_file(fd);
    if (!finfo)
    {
        return -1;
    }
    off_t new_offset = finfo->offset;
    switch (whence)
    {
        case SEEK_SET:
            new_offset = offset;
            break;
        case SEEK_CUR:
            new_offset += offset;
            break;
        case SEEK_END:
            new_offset = E2END + offset;
            break;
        default:
            new_offset = E2END + 1;
    }
    if (new_offset > E2END)
    {
        errno = EINVAL;
        return -1;
    }
    finfo->offset = new_offset;
    return new_offset;
}

} // extern "C"

#endif // _FREERTOS_DRIVERS_ARDUINO_ARDUINOFS_HXX_

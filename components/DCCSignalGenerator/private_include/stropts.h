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
 * \file stropts.h
 * This file implements ioctl() prototypes and defines.
 *
 * @author Stuart W. Baker
 * @date 2 November 2013
 */

#ifndef _stropts_h_
#define _stropts_h_

#if defined (__cplusplus)
extern "C" {
#endif

#if defined(ESP32)
#include <sys/ioctl.h>
#else
/** Request and ioctl transaction
 * @param fd file descriptor
 * @param key ioctl key
 * @param ... key data (as a pointer or unsigned long type)
 */
int ioctl(int fd, unsigned long int key, ...);
#endif // ESP32

/** ioctl key value for operation (not read or write) */
#define IOC_NONE 0U

/** ioctl key write bit */
#define IOC_WRITE 1U

/** ioctl key read bit */
#define IOC_READ 2U

/** create an ioctl.
 * @param _dir direction
 * @param _type device driver unique number
 * @param _num ioctl index number
 * @param _size size of ioctl data in bytes
 */
#define IOC(_dir, _type, _num, _size) \
    (((_dir) << 30) | ((_type) << 8) | ((_num)) | ((_size) << 16))

/** create an operation ioctl
 * @param _type device driver unique number
 * @param _num ioctl index number
 */
#define IO(_type, _num) IOC(IOC_NONE, (_type), (_num), 0)

/** create an operation ioctl
 * @param _type device driver unique number
 * @param _num ioctl index number
 * @param _size size of ioctl data in bytes
 */
#define IOR(_type, _num, _size) IOC(IOC_READ, (_type), (_num), (_size))

/** create an operation ioctl
 * @param _type device driver unique number
 * @param _num ioctl index number
 * @param _size size of ioctl data in bytes
 */
#define IOW(_type, _num, _size) IOC(IOC_WRITE, (_type), (_num), (_size))

/** create an operation ioctl
 * @param _type device driver unique number
 * @param _num ioctl index number
 * @param _size size of ioctl data in bytes
 */
#define IOWR(_type, _num, _size) IOC(IOC_WRITE | IOC_READ, (_type), (_num), (_size))

/** Decode ioctl number direction.
 * @param _num encoded ioctl value
 * @return IOC_NONE, IOC_WRITE, IOC_READ, or IOC_WRITE | IOC_READ
 */
#define IOC_DIR(_num) (((_num) >> 30) & 0x00000003)

/** Decode ioctl type.
 * @param _num encoded ioctl value
 * @return device driver unique number
 */
#define IOC_TYPE(_num) (((_num) >> 8) & 0x000000FF)

/** Decode ioctl number.
 * @param _num encoded ioctl value
 * @return ioctl index number
 */
#define IOC_NR(_num) (((_num) >> 0) & 0x000000FF)

/** Decode ioctl size.
 * @param _num encoded ioctl value
 * @return size of ioctl data in bytes
 */
#define IOC_SIZE(_num) (((_num) >> 16) & 0x00003FFF)

/** Number of bits that make up the size field */
#define IOC_SIZEBITS 14

#if defined (__cplusplus)
}
#endif

#endif /* _stropts_h_ */

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
 * \file can_ioctl.h
 * This file implements can specific ioctl() keys.
 *
 * @author Stuart W. Baker
 * @date 2 November 2013
 */

#ifndef _FREERTOS_CAN_IOCTL_H_
#define _FREERTOS_CAN_IOCTL_H_

#include <stdint.h>
#ifdef ESP32
#include "stropts.h"
#else
#include "freertos/stropts.h"
#endif

#if defined (__cplusplus)
extern "C" {
#endif

/** Magic number for this driver's ioctl calls */
#define CAN_IOC_MAGIC ('c')

/// ioctl minor type used for the read/write active notifiable integration.
#define NOTIFIABLE_TYPE 13

/** read active ioctl. Argument is a literal pointer to a Notifiable. */
#define CAN_IOC_READ_ACTIVE IOW(CAN_IOC_MAGIC, 1, NOTIFIABLE_TYPE)

/** write active ioctl. Argument is a literal pointer to a Notifiable. */
#define CAN_IOC_WRITE_ACTIVE IOW(CAN_IOC_MAGIC, 2, NOTIFIABLE_TYPE)

/** CAN state type */
typedef uint32_t can_state_t;

/** Read the CAN state */
#define SIOCGCANSTATE IOR(CAN_IOC_MAGIC, 3, sizeof(can_state_t))

/** CAN bus active */
#define CAN_STATE_ACTIVE            0

/** CAN bus error warning */
#define CAN_STATE_BUS_WARNING       1

/** CAN bus error passive */
#define CAN_STATE_BUS_PASSIVE       2

/** CAN bus off */
#define CAN_STATE_BUS_OFF           3

/** CAN bus scanning baud rate (CANFD) */
#define CAN_STATE_SCANNING_BAUDRATE 4

/** CAN bus stopped */
#define CAN_STATE_STOPPED           5

/** CAN bus sleeping */
#define CAN_STATE_SLEEPING          6

#if defined (__cplusplus)
}
#endif

#endif /* _FREERTOS_CAN_IOCTL_H_ */

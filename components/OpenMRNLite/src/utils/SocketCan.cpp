/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file SocketCan.cxx
 *
 * Helper functions to connect to CAN devices via SocketCan.
 *
 * @author Balazs Racz
 * @date 1 Sep 2020
 */

#include "utils/SocketCan.hxx"
#include "can_frame.h"
#include <string.h>

#if defined(__linux__)

#include <errno.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <stdio.h>
#include <sys/ioctl.h>

/// This macro executes an OS call, and if it returns negative result, then
/// prints the errno to stderr, and terminates the current function with -1
/// return value.
/// @param where textual description of what function was called
/// (e.g. "socket")
/// @param x... the function call.
#define ERRNOLOG(where, x...)                                                  \
    do                                                                         \
    {                                                                          \
        if ((x) < 0)                                                           \
        {                                                                      \
            perror(where);                                                     \
            return -1;                                                         \
        }                                                                      \
    } while (0)

int socketcan_open(const char *device, int loopback)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ERRNOLOG("socket", s);

    // Set the blocking limit to the minimum allowed, typically 1024 in Linux
    int sndbuf = 0;
    ERRNOLOG("setsockopt(sndbuf)",
        setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)));

    // turn on/off loopback
    ERRNOLOG("setsockopt(loopback)",
        setsockopt(
            s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)));

    // setup error notifications
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB |
        CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK |
        CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    ERRNOLOG("setsockopt(filter)",
        setsockopt(
            s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)));
    strcpy(ifr.ifr_name, device);

    ERRNOLOG("interface set", ::ioctl(s, SIOCGIFINDEX, &ifr));

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    ERRNOLOG("bind", bind(s, (struct sockaddr *)&addr, sizeof(addr)));

    return s;
}

#endif

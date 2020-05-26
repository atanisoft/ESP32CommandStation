/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file ifaddr.h
 * This file implements Linux/BSD compatible ifaddr.h prototypes.
 *
 * @author Stuart W. Baker
 * @date 2 September 2018
 */

#ifndef _IFADDRS_H_
#define _IFADDRS_H_

#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

/// network interface address list member
struct ifaddrs
{
    struct ifaddrs  *ifa_next;    ///< next item in list
    char            *ifa_name;    ///< name of interface.
    unsigned int    *ifa_flags;   ///< flags from SIOCGIFFLAGS
    struct sockaddr *ifa_addr;    ///< address of interface
    struct sockaddr *ifa_netmask; ///< netmask of interface
    union
    {
        struct sockaddr *ifu_broadaddr; ///< broadcast address of interface
        struct sockaddr *ifu_dstadr;    ///< point-to-point destination address
    } ifa_ifu;
    void            *ifa_data;    ///< address-specific data
};

/// Create a linked list of structures describing the network interfaces of the
/// local system.
///
/// @param ifap the first item in the list is in *ifap
/// @return 0 upon success, else -1 with errno set appropriately
int getifaddrs(struct ifaddrs **ifap);

/// Free a previously generated linked list of structures describing the network
/// interfaces of the local system.
///
/// @param ifa pointer to the list that will be freed
void freeifaddrs(struct ifaddrs *ifa);

#ifdef __cplusplus
}
#endif

#endif /* _IFADDRS_H_ */

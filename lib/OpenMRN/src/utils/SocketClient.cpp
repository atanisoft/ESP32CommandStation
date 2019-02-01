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
 * \file socket_client.cxx
 *
 * Connects to a remote TCP socket.
 *
 * @author Balazs Racz
 * @date 28 Dec 2013
 */

#if defined(__linux__) || defined (__MACH__) || defined (__FreeRTOS__)

#define LOGLEVEL INFO

#include "utils/SocketClient.hxx"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#include <memory>

#include "utils/socket_listener.hxx"

#include "utils/format_utils.hxx"
#include "utils/macros.h"
#include "utils/logging.h"

int ConnectSocket(const char *host, int port)
{
    return SocketClient::connect(host, port);
}

int ConnectSocket(const char *host, const char* port_str)
{
    return SocketClient::connect(host, port_str);
}

int SocketClient::connect(const char *host, int port)
{
#ifdef __linux__
    // We expect write failures to occur but we want to handle them where 
    // the error occurs rather than in a SIGPIPE handler.
    signal(SIGPIPE, SIG_IGN);
#endif

    char port_str[30];
    integer_to_buffer(port, port_str);

    return SocketClient::connect(host, port_str);
}

int SocketClient::connect(const char *host, const char* port_str)
{
    struct addrinfo *addr;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;

    if (int ai_ret = getaddrinfo(host, port_str, &hints, &addr) != 0 || !addr)
    {
        LOG_ERROR("getaddrinfo failed for '%s': %s", host,
                  gai_strerror(ai_ret));
        return -1;
    }

    struct AddrInfoDeleter
    {
        void operator()(struct addrinfo *s)
        {
            if (s)
            {
                freeaddrinfo(s);
            }
        }
    };
    std::unique_ptr<struct addrinfo, AddrInfoDeleter> ai_deleter(addr);

    int fd = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (fd < 0)
    {
        LOG_ERROR("socket: %s", strerror(errno));
        return -1;
    }

    int ret = ::connect(fd, addr->ai_addr, addr->ai_addrlen);
    if (ret < 0)
    {
        LOG_ERROR("connect: %s", strerror(errno));
        close(fd);
        return -1;
    }

    int val = 1;
    ERRNOCHECK("setsockopt(nodelay)",
               ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val)));

    LOG(INFO, "Connected to %s:%s. fd=%d", host ? host : "mDNS", port_str, fd);
    return fd;
}

/*
 * SocketClient::local_test()
 */
bool SocketClient::local_test(struct addrinfo *addr)
{
    bool local = false;
    struct ifaddrs *ifa;
    int result = getifaddrs(&ifa);
    if (result == 0)
    {
        struct ifaddrs *ifa_free = ifa;
        while (ifa)
        {
            if (ifa->ifa_addr)
            {
                /* ifa_addr pointer valid */
                if (ifa->ifa_addr->sa_family == AF_INET)
                {
                    /* have a valid IPv4 address */
                    struct sockaddr_in *ai_addr_in =
                        (struct sockaddr_in*)addr->ai_addr;
                    struct sockaddr_in *if_addr_in =
                        (struct sockaddr_in*)ifa->ifa_addr;
                    if (ai_addr_in->sin_addr.s_addr ==
                        if_addr_in->sin_addr.s_addr)
                    {
                        /* trying to connected to myself */
                        local = true;
                        break;
                    }
                }
            }
            ifa = ifa->ifa_next;
        }
        freeifaddrs(ifa_free);
    }

    return local;
}

#endif // __linux__

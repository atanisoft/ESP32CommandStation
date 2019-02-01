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
 * \file socket_listener.cxx
 *
 * Listens to a socket and calls a callback for every incoming connection.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#if defined (__linux__) || defined (__MACH__) || defined(__FreeRTOS__)

#define _DEFAULT_SOURCE

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>

#include "utils/socket_listener.hxx"

#include "utils/macros.h"
#include "utils/logging.h"


static void* accept_thread_start(void* arg) {
  SocketListener* l = static_cast<SocketListener*>(arg);
  l->AcceptThreadBody();
  return NULL;
}

SocketListener::SocketListener(int port, connection_callback_t callback)
    : startupComplete_(0),
      shutdownRequested_(0),
      shutdownComplete_(0),
      port_(port),
      callback_(callback),
      accept_thread_("accept_thread", 0, 1000, accept_thread_start, this) {
#ifdef __linux__
    // We expect write failures to occur but we want to handle them where the
    // error occurs rather than in a SIGPIPE handler.
    signal(SIGPIPE, SIG_IGN);
#endif
}

SocketListener::~SocketListener() {
    if (!shutdownComplete_) {
        shutdown();
    }
}

void SocketListener::shutdown()
{
    shutdownRequested_ = 1;
    while (!shutdownComplete_) {
        usleep(1000);
    }
}

void SocketListener::AcceptThreadBody() {
  socklen_t namelen;
  struct sockaddr_in addr;
  int listenfd;

  ERRNOCHECK("socket", listenfd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP));

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);
  int val = 1;
  ERRNOCHECK("setsockopt_reuseaddr",
             setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)));

  ERRNOCHECK("bind",
             ::bind(listenfd, (struct sockaddr *) &addr, sizeof(addr)));

#ifndef __FreeRTOS__  // no getsockname support
  namelen = sizeof(addr);
  ERRNOCHECK("getsockname",
             getsockname(listenfd, (struct sockaddr *) &addr, &namelen));

  // This is the actual port that got opened. We could check it against the
  // requested port. listenport = ;
#endif

  // FreeRTOS+TCP uses the parameter to listen to set the maximum number of
  // connections to the given socket, so allow some room
  ERRNOCHECK("listen", listen(listenfd, 5));

  LOG(INFO, "Listening on port %d, fd %d", ntohs(addr.sin_port), listenfd);

#ifndef __FreeRTOS__
  {
      struct timeval tm;
      tm.tv_sec = 0;
      tm.tv_usec = MSEC_TO_USEC(100);
      ERRNOCHECK("setsockopt_timeout",
          setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm)));
  }
#endif

  int connfd;

  startupComplete_ = 1;

  while (!shutdownRequested_) {
    namelen = sizeof(addr);
    connfd = accept(listenfd,
                    (struct sockaddr *)&addr,
                    &namelen);
    if (connfd < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EMFILE) {
        continue;
      }
      else if (errno == ECONNABORTED) {
        break;
      }
      print_errno_and_exit("accept");
      return;
    }
    int val = 1;
    ERRNOCHECK("setsockopt(nodelay)",
               setsockopt(connfd, IPPROTO_TCP, TCP_NODELAY,
                          &val, sizeof(val)));

    LOG(INFO, "Incoming connection from %s, fd %d.", inet_ntoa(addr.sin_addr),
        connfd);

    callback_(connfd);
  }
  close(listenfd);
  LOG(INFO, "Shutdown listening socket %d.", port_);
  shutdownComplete_ = 1;
}




#endif // __linux__

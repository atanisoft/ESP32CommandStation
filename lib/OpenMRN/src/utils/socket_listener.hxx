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
 * \file socket_listener.hxx
 *
 * Interface for listening to a socket and delivering a callback for each
 * incoming connection.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#ifndef _UTILS_SOCKET_LISTENER_HXX_
#define _UTILS_SOCKET_LISTENER_HXX_

#include <functional>

#include "os/OS.hxx"

/// TCP server. Listends to a specific TCP port, accepts every
/// incomingconnection, and calls an application-specific callback with the new
/// file descriptor.
///
/// usage: Instantiate SocketListener and pass in a function object (by either
/// std::bind or a C++11 lambda function) to decide what to do with the new
/// incoming connections.
///
/// Starts a separate thread for the accept loop.
class SocketListener
{
public:
    /// Callback type that will be called when a new incoming connection is
    /// established. The argument is the filedescriptor of the new connection.
    typedef std::function<void(int)> connection_callback_t;

    /// Constructor. Startsthe listener.
    ///
    /// @param port which TCP port number to listen upon.
    /// @param callback will be called on each incoming connection.
    SocketListener(int port, connection_callback_t callback);
    ~SocketListener();

    /// Implementation of the accept thread.
    void AcceptThreadBody();

    /** Shuts down the socket listener. Blocks until the accept thread is
     * stopped. This call can take up to 100 msec as of now. It is safe to
     * delete *this after this call is returned. */
    void shutdown();

    /// @return true if the accept thread is already listening.
    bool is_started()
    {
        return startupComplete_;
    }

private:
    /// 1 if we have completed bind.
    volatile unsigned startupComplete_ : 1;
    /// 1 if shutting down.
    volatile unsigned shutdownRequested_ : 1;
    /// 1 if accept thread is exited.
    volatile unsigned shutdownComplete_ : 1;
    /// Port to listen on.
    int port_;
    /// Callback to call with each incoming conneciton.
    connection_callback_t callback_;
    /// Thread handle / instance for running the listen/accept loop.
    OSThread accept_thread_;
};

/// Connects a tcp socket to the specified remote host:port. Returns -1 if
/// unsuccessful; returns the fd if successful.
///
/// @param host hostname to connect to
/// @param port TCP port number to connect to
///
/// @return fd of the connected socket.
///
int ConnectSocket(const char *host, int port);

/// Connects a tcp socket to the specified remote host:port. Returns -1 if
/// unsuccessful; returns the fd if successful.
///
/// @param host hostname to connect to. Shall be null for mDNS target.
/// @param port TCP port number or mDNS hostname to connect to.
///
/// @return fd of the connected socket.
///
int ConnectSocket(const char *host, const char* port_str);

#endif //_UTILS_SOCKET_LISTENER_HXX_

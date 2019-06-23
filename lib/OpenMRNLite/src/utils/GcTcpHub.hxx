/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file GcTcpHub.hxx
 * A component that starts a gridconnect-protocol HUB listening on a TCP port.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_GCTCPHUB_HXX_
#define _UTILS_GCTCPHUB_HXX_

#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"

class ExecutorBase;

/** This class runs a CAN-bus HUB listening on TCP socket using the gridconnect
 * format. Any new incoming connection will be wired into the same virtual CAN
 * hub. All packets will be forwarded to every participant, without
 * loopback. */
class GcTcpHub
{
public:
    /// Constructor.
    ///
    /// @param can_hub Which CAN-hub should we attach the TCP gridconnect hub
    /// onto.
    /// @param port TCp port number to listen on.
    GcTcpHub(CanHubFlow *can_hub, int port);
    ~GcTcpHub();

    /// @return true of the listener is ready to accept incoming connections.
    bool is_started()
    {
        return tcpListener_.is_started();
    }

private:
    /// Callback when a new connection arrives.
    ///
    /// @param fd filedes of the freshly established incoming connection.
    ///
    void OnNewConnection(int fd);

    /// @param can_hub Which CAN-hub should we attach the TCP gridconnect hub
    /// onto.
    CanHubFlow *canHub_;
    /// Helper object representing the listening on the socket.
    SocketListener tcpListener_;
};

#endif // _UTILS_GCTCPHUB_HXX_

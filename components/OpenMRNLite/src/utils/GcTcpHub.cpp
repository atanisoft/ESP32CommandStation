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
 * \file GcTcpHub.cxx
 * A component that starts a gridconnect-protocol HUB listening on a TCP port.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#include <memory>

#include "utils/GcTcpHub.hxx"

#include "nmranet_config.h"
#include "utils/GridConnectHub.hxx"

void GcTcpHub::on_new_connection(int fd)
{
    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);
    {
        AtomicHolder h(this);
        numClients_++;
    }
    create_gc_port_for_can_hub(canHub_, fd, this, use_select);
}

void GcTcpHub::notify()
{
    AtomicHolder h(this);
    if (numClients_)
    {
        numClients_--;
    }
}

GcTcpHub::GcTcpHub(CanHubFlow *can_hub, int port)
    : canHub_(can_hub)
    , tcpListener_(port,
          std::bind(&GcTcpHub::on_new_connection, this, std::placeholders::_1),
          "GcTcpHub")
{
}

GcTcpHub::~GcTcpHub()
{
    tcpListener_.shutdown();
}

/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file DatagramTcp.cxx
 *
 * TCP-If datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 24 March 2019
 */

#include "openlcb/DatagramTcp.hxx"

#include "openlcb/DatagramImpl.hxx"

namespace openlcb
{

TcpDatagramService::TcpDatagramService(
    IfTcp *iface, int num_registry_entries, int num_clients)
    : DatagramService(iface, num_registry_entries)
{
    auto *dg_send = if_tcp()->addressed_message_write_flow();
    for (int i = 0; i < num_clients; ++i)
    {
        auto *client_flow = new DatagramClientImpl(if_tcp(), dg_send);
        if_tcp()->add_owned_flow(client_flow);
        client_allocator()->insert(static_cast<DatagramClient *>(client_flow));
    }
}

TcpDatagramService::~TcpDatagramService()
{
}

} // namespace openlcb

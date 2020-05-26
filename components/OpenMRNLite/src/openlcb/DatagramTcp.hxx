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
 * \file DatagramTcp.hxx
 *
 * TCP-If datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 24 March 2019
 */

#ifndef _OPENLCB_DATAGRAMTCP_HXX_
#define _OPENLCB_DATAGRAMTCP_HXX_

#include "openlcb/Datagram.hxx"
#include "openlcb/IfTcp.hxx"

namespace openlcb
{

/// Implementation of the DatagramService on TCP transfer. This class is also
/// responsible for instantiating the correct DatagramClient objects.
class TcpDatagramService : public DatagramService
{
public:
    /// @param iface is the TCP interface.
    /// @param num_registry_entries is the size of the registry map (how many
    /// datagram handlers can be registered)
    /// @param num_clients how many datagram clients to create. These are
    /// allocated and freed on demand by flows sending datagrams.
    TcpDatagramService(IfTcp *iface, int num_registry_entries, int num_clients);

    ~TcpDatagramService();

    IfTcp *if_tcp()
    {
        return static_cast<IfTcp *>(iface());
    }
};

} // namespace openlcb

#endif // _OPENLCB_DATAGRAMTCP_HXX_

/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file DatagramCan.hxx
 *
 * CANbus datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 25 Jan 2014
 */

#ifndef _OPENLCB_DATAGRAMCAN_HXX_
#define _OPENLCB_DATAGRAMCAN_HXX_

#include "openlcb/IfCan.hxx"
#include "openlcb/Datagram.hxx"

namespace openlcb
{

/// Implementation of the DatagramService with the CANbus-specific OpenLCB
/// datagram protocol. This service is responsible for fragmenting outgoing
/// datagram messages to the CANbus, assembling incoming datagram frames into
/// messages and managing the necessary temporary buffers. This class is also
/// responsible for instantiating the correct DatagramClient objects.
class CanDatagramService : public DatagramService
{
public:
    /*
     * @param num_registry_entries is the size of the registry map (how
     * many datagram handlers can be registered)*/
    CanDatagramService(IfCan *iface, int num_registry_entries,
                       int num_clients);

    ~CanDatagramService();

    IfCan *if_can()
    {
        return static_cast<IfCan *>(iface());
    }
};

/// Creates a CAN datagram parser flow. Exposed for testing only.
Executable *TEST_CreateCanDatagramParser(IfCan *if_can);

} // namespace openlcb

#endif // _OPENLCB_DATAGRAMCAN_HXX_

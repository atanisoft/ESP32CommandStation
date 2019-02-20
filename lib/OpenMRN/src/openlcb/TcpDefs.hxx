/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file TcpDefs.hxx
 *
 * Static declarations, enums and helper functions for the OpenLCB TCP
 * interfaces.
 *
 * @author Balazs Racz
 * @date 12 September 2017
 */

#ifndef _OPENLCB_TCPDEFS_HXX_
#define _OPENLCB_TCPDEFS_HXX_

#include "utils/macros.h"

namespace openlcb {

class TcpDefs {
public:
    /// Protocol to be used for mDNS broadcast
    static const char MDNS_PROTOCOL_TCP[];
    /// base name of the mDNS Service Name for mDNS broadcast as a hub
    static const char MDNS_SERVICE_NAME_HUB[];
    /// complete mDNS broadcast name for a TCP hub
    static const char MDNS_SERVICE_NAME_HUB_TCP[];
    /// base name of the mDNS Service Name for mDNS broadcast as a client
    static const char MDNS_SERVICE_NAME_GRIDCONNECT_CAN[];
    /// complete mDNS broadcast name for a TCP GridConnect protocol client
    static const char MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP[];

private:
    /// Nobody can construct this class.
    TcpDefs();
    DISALLOW_COPY_AND_ASSIGN(TcpDefs);
};  // class TcpDefs

}  // namespace openlcb

#endif // _OPENLCB_TCPDEFS_HXX_

/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file RailcomHub.hxx
 *
 * Declarations for listening to incoming railcom packets.
 *
 * @author Balazs Racz
 * @date 16 May 2015
 */

#ifndef _DCC_RAILCOMHUB_HXX_
#define _DCC_RAILCOMHUB_HXX_

#include "utils/Hub.hxx"
#include "dcc/RailCom.hxx"

namespace dcc {

/// Data payload sent in the buffers for Railcom dispatchers and hubs.
typedef HubContainer<StructContainer<dcc::Feedback> > RailcomHubData;
/// Interface class for consumers of railcom data.
typedef FlowInterface<Buffer<RailcomHubData> > RailcomHubPortInterface;
/// Base class for consumers of railcom data that are implemented as state
/// flows.
typedef StateFlow<Buffer<RailcomHubData>, QList<1>> RailcomHubPort;
/// The hub flow that sends a copy of each packet to each listener port
/// registered.
typedef GenericHubFlow<RailcomHubData> RailcomHubFlow;

}  // namespace dcc

#endif // _DCC_RAILCOMHUB_HXX_

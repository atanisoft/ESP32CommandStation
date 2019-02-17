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
 * \file EventHandlerMock.hxx
 *
 * Helper utilities for testing event handlers.
 *
 * This file must only ever be included in unittests.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#ifndef _OPENLCB_EVENTHANDLERMOCK_HXX_
#define _OPENLCB_EVENTHANDLERMOCK_HXX_

#include "gmock/gmock.h"
#include "openlcb/EventHandler.hxx"

namespace openlcb {

/// Test handler for receiving incoming event related messages via the
/// EventService. Incoming messages need GoogleMock expectations.
class MockEventHandler : public EventHandler
{
public:
/// Proxies an event handler function to a gmock function.
///
/// @param FN name of the function to proxy.
#define DEFPROXYFN(FN)                                                         \
    MOCK_METHOD3(FN, void(const EventRegistryEntry &, EventReport *event,      \
                          BarrierNotifiable *done))

    DEFPROXYFN(handle_event_report);
    DEFPROXYFN(handle_consumer_identified);
    DEFPROXYFN(handle_consumer_range_identified);
    DEFPROXYFN(handle_producer_identified);
    DEFPROXYFN(handle_producer_range_identified);
    DEFPROXYFN(handle_identify_global);
    DEFPROXYFN(handle_identify_consumer);
    DEFPROXYFN(handle_identify_producer);

#undef DEFPROXYFN
};

}  // namespace openlcb

#endif // _OPENLCB_EVENTHANDLERMOCK_HXX_



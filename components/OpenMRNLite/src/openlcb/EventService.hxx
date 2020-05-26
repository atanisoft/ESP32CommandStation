/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file EventService.hxx
 *
 * A service that takes incoming event messages, owns the event registry, and
 * dispatches the incoming event messages to the registered handlers. This
 * service can be registered to multiple NMRAnet::If interfaces at the same
 * time.
 *
 * @author Balazs Racz
 * @date 22 May 2014
 */

#ifndef _OPENLCB_EVENTSERVICE_HXX_
#define _OPENLCB_EVENTSERVICE_HXX_

// This is a workaround for missing shared_ptr.h causing compilation errors. We
// do not use shared_ptr.
#ifndef __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#define __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#endif

#define DEBUG_EVENT_PERFORMANCE

#include <memory>

#include "utils/macros.h"
#include "executor/Service.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/If.hxx"

namespace openlcb
{

class Node;

class EventIteratorFlow;

/// Global Event Service. Registers itself with a specific interface to receive
/// all incoming messages related to the OpenLCB Event Protocol, maintains the
/// registry of event handlers, and routes the incoming messages to the event
/// handlers based on the registration arguments.
class EventService : public Service
{
public:
    /** Creates a global event service with no interfaces registered. */
    EventService(ExecutorBase *e);
    /** Creates a global event service that runs on an interface's thread and
     * registers the interface. */
    EventService(If *iface);
    ~EventService();

    /** Registers this global event handler with an interface. This operation
     * will be undone in the destructor. */
    void register_interface(If *iface);

    class Impl;
    Impl *impl()
    {
        return impl_.get();
    }

    /** Returns true if there are outstanding events that are not yet
     * handled. */
    bool event_processing_pending();

    static EventService *instance;

private:
    std::unique_ptr<Impl> impl_;
};

}; /* namespace openlcb */

#endif // _OPENLCB_EVENTSERVICE_HXX_

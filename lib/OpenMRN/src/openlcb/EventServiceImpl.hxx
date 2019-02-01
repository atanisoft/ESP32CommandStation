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
 * \file EventServiceImpl.hxx
 *
 * Implementation headers shared between components of the global event
 * handler's various flows. An end-user will typically not need to include this
 * header.
 *
 * @author Balazs Racz
 * @date 20 April 2014
 */

#ifndef _OPENLCB_EVENTSERVICEIMPL_HXX_
#define _OPENLCB_EVENTSERVICEIMPL_HXX_

#include <memory>
#include <vector>

#include "openlcb/EventService.hxx"
#include "openlcb/EventHandler.hxx"

namespace openlcb
{

class IncomingEventFlow;
class GlobalIdentifyFlow;
class EventHandler;

/// Arguments structure for the EventCallerFlow. Each such buffer sent to @ref
/// EventCallerFlow means calling one event handler's one specific function
/// with a given argument.
struct EventHandlerCall
{
    const EventRegistryEntry *registry_entry;
    EventReport *rep;
    EventHandlerFunction fn;
    unsigned epoch;
    void reset(const EventRegistryEntry *entry, unsigned epoch,
        EventReport *rep, EventHandlerFunction fn)
    {
        this->registry_entry = entry;
        this->rep = rep;
        this->fn = fn;
        this->epoch = epoch;
    }
};

/// Control flow that calls individual event handlers one at a time and waits
/// until the done callback is invoked before calling the next event
/// handler. In essence this control flow behaves as a global lock for the
/// event handlers being called. This global lock is necessary, because the
/// event handlers are using global buffers for holding the outgoing packets.
class EventCallerFlow : public StateFlow<Buffer<EventHandlerCall>, QList<5>>
{
public:
    EventCallerFlow(Service *service)
        : StateFlow<Buffer<EventHandlerCall>, QList<5>>(service) {};

private:
    virtual Action entry() OVERRIDE;
    Action call_done();

    BarrierNotifiable n_;
};

/// PImpl class for the EventService. This class creates and owns all
/// components necessary to the correct operation of the EventService but does
/// not need to appear on the application-facing API.
class EventService::Impl
{
public:
    Impl(EventService *service);
    ~Impl();

    /// The implementation of the event registry.
    std::unique_ptr<EventRegistry> registry;

    /// Flows that we own. There will be a few entries for each interface
    /// registered.
    std::vector<std::unique_ptr<StateFlowWithQueue>> ownedFlows_;

    /// This flow will serialize calls to NMRAnetEventHandler objects. All such
    /// calls need to be sent to this flow.
    EventCallerFlow callerFlow_;

    enum
    {
        // These address/mask should match all the messages carrying an event
        // id.
        MTI_VALUE_EVENT = Defs::MTI_EVENT_MASK,
        MTI_MASK_EVENT = Defs::MTI_EVENT_MASK,
        MTI_VALUE_GLOBAL = Defs::MTI_EVENTS_IDENTIFY_GLOBAL,
        MTI_MASK_GLOBAL = 0xffff,
        MTI_VALUE_ADDRESSED_ALL = Defs::MTI_EVENTS_IDENTIFY_ADDRESSED,
        MTI_MASK_ADDRESSED_ALL = 0xffff,
    };
};

/** Flow to receive incoming messages of event protocol, and dispatch them to
 * the registered event handler. This flow runs on the executor of the event
 * service (and not necessarily the interface). Its main job is to iterate
 * through the matching event handler and call each of them for that report. */
class EventIteratorFlow : public IncomingMessageStateFlow
{
public:
    EventIteratorFlow(If *iface, EventService *event_service,
                      unsigned mti_value, unsigned mti_mask);
    ~EventIteratorFlow();

protected:
    Action entry() OVERRIDE;
    Action iterate_next();

private:
    virtual Action dispatch_event(const EventRegistryEntry *entry);

protected:
    EventService *eventService_;

    /// Statically allocated structure for calling the event handlers from the
    /// main event queue.
    EventReport eventReport_;

    /** Iterator for generating the event handlers from the registry. */
    EventIterator *iterator_;
    /** This done notifiable holds a reference to the incoming message
     * buffer. We must not release this notifiable until we have completed
     * processing and freed all the buffers related to this iteration. */
    Notifiable *incomingDone_;
    /// The epoch of the event registry at the start of the iteration. Used to
    /// recognize when the iterators are invalidated.
    unsigned eventRegistryEpoch_;

    BarrierNotifiable n_;
    EventHandlerFunction fn_;

#ifdef DEBUG_EVENT_PERFORMANCE
    static const int REPORT_COUNT = 100;
    /// How many events' cost are accumulated so far.
    uint8_t countEvents_{0};
    uint16_t mtiValue_;
    /// Accumulator of how many msec processingthe events took.
    long long numProcessNsec_{0};
    /// When the processing of the current event started.
    long long currentProcessStart_{0};
#endif
};

/** Flow to receive incoming messages of event protocol, and dispatch them to
 * the registered event handler. This flow runs on the executor of the event
 * service (and not necessarily the interface). Its main job is to iterate
 * through the matching event handler and call each of them for that report. */
class InlineEventIteratorFlow : public EventIteratorFlow
{
public:
    InlineEventIteratorFlow(If *iface, EventService *event_service,
                            unsigned mti_value, unsigned mti_mask)
        : EventIteratorFlow(iface, event_service, mti_value, mti_mask)
    {
    }

private:
    Action dispatch_event(const EventRegistryEntry *entry) OVERRIDE;

    /// The handler we need to call.
    const EventRegistryEntry *currentEntry_{nullptr};
};

} // namespace openlcb

#endif // _OPENLCB_EVENTSERVICEIMPL_HXX_

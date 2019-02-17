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
 * \file EventHandler.hxx
 * Interface for handling NMRAnet events.
 *
 * @author Balazs Racz
 * @date 29 September 2013
 */

#ifndef _OPENLCB_EVENTHANDLER_HXX_
#define _OPENLCB_EVENTHANDLER_HXX_

#include <stdint.h>

#include "executor/Notifiable.hxx"
#include "utils/AsyncMutex.hxx"
#include "utils/macros.h"
#include "utils/Singleton.hxx"
#include "openlcb/WriteHelper.hxx"

namespace openlcb
{

typedef uint64_t EventId;
class Node;
class EventHandler;

/*enum EventMask {
  EVENT_EXACT_MASK = 1,
  EVENT_ALL_MASK = 0xffffffffffffffffULL
  };*/

enum TestingEnum
{
    FOR_TESTING
};

/// Shared notification structure that is assembled for each incoming
/// event-related message, and passed around to all event handlers.
struct EventReport
{
    /// The event ID from the incoming message.
    EventId event;
    /// Specifies the mask in case the request is for an event range. The low
    /// bits are set to one, the high bits are set to zero. Ranges of size 1
    /// have
    /// mask==0, a range that covers all events has mask==0xffff...f.
    EventId mask;
    /// Information about the sender of the incoming event-related OpenLCB
    /// message. It is not specified whether the node_id or the alias is
    /// specified, but they are not both zero.
    NodeHandle src_node;
    /// nullptr for global messages; points to the specific virtual node for
    /// addressed events identify message.
    Node *dst_node;
    /// For producer/consumer identified messages, specifies the state of the
    /// producer/consumer as the sender of the message
    /// (valid/invalid/unknown/reserved).
    EventState state;

    /// These allow event handlers to produce up to four messages per
    /// invocation. They are always available at the entry to an event handler
    /// function.
    template <int N> WriteHelper *event_write_helper()
    {
        static_assert(1 <= N && N <= 4, "WriteHelper out of range.");
        return write_helpers + (N - 1);
    }

    /// Public constructor for use in tests only.
    EventReport(TestingEnum)
    {
    }

private:
    /// Constrained access to the constructors. We do this because the
    /// EventReport structure is pretty expensive due to the statically
    /// allocated memory of the write helpers. Only the EventIteratorFlow
    /// should have objects of this type.
    EventReport() {}
    friend class EventIteratorFlow;
    friend class DecoderRangeTest;

    /// Static objects usable by all event handler implementations.
    WriteHelper write_helpers[4];
};

/// Structure used in registering event handlers.
class EventRegistryEntry
{
public:
    /// Stores the event ID or beginning of range for which to register the
    /// given handler.
    EventId event;
    /// Pointer to the handler.
    EventHandler *handler;
    /// Opaque user argument. The event handlers may use this to store
    /// arbitrary data.
    uint32_t user_arg;
    EventRegistryEntry(EventHandler *_handler, EventId _event)
        : event(_event)
        , handler(_handler)
        , user_arg(0)
    {
    }
    EventRegistryEntry(EventHandler *_handler, EventId _event,
                       unsigned _user_arg)
        : event(_event)
        , handler(_handler)
        , user_arg(_user_arg)
    {
    }
};

/// Abstract base class for all event handlers. Instances of this class can
/// get registered with the event service to receive notifications of incoming
/// event messages from the bus.
class EventHandler
{
public:
    using EventReport = openlcb::EventReport;
    using EventRegistryEntry = openlcb::EventRegistryEntry;
    using EventId = openlcb::EventId;

    virtual ~EventHandler()
    {
    }

    /// Called on incoming EventReport messages. @param event stores
    /// information about the incoming message. Filled: src_node, event. Mask
    /// is always 1 (filled in). state is not filled in. @param registry_entry
    /// gives the registry entry for which the current handler is being
    /// called. @param done must be notified when the processing is done.
    virtual void handle_event_report(const EventRegistryEntry &registry_entry,
                                   EventReport *event,
                                   BarrierNotifiable *done) = 0;

    /// Called on another node sending ConsumerIdentified for this event.
    /// @param event stores information about the incoming message. Filled:
    /// event_id, mask=1, src_node, state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// done must be notified when the processing is done.
    virtual void
    handle_consumer_identified(const EventRegistryEntry &registry_entry,
                             EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    };

    /// Called on another node sending ConsumerRangeIdentified. @param event
    /// stores information about the incoming message. Filled: event id, mask
    /// (!= 1), src_node. Not filled: state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// done must be notified when the processing is done.
    virtual void
    handle_consumer_range_identified(const EventRegistryEntry &registry_entry,
                                  EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on another node sending ProducerIdentified for this event.
    /// @param event stores information about the incoming message. Filled:
    /// event_id, mask=1, src_node, state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// user_arg is an opaque argument passed in from the registration. @param
    /// done must be notified when the processing is done.
    virtual void
    handle_producer_identified(const EventRegistryEntry &registry_entry,
                             EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on another node sending ProducerRangeIdentified for this
    /// event. @param event stores information about the incoming
    /// message. Filled: event id, mask (!= 1), src_node. Not filled: state.
    /// @param registry_entry gives the registry entry for which the current
    /// handler is being called. @param done must be notified when the
    /// processing is done.
    virtual void
    handle_producer_range_identified(const EventRegistryEntry &registry_entry,
                                  EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on the need of sending out identification messages. @param event
    /// is NULL. This happens on startup, or when a global or addressed
    /// IdentifyGlobal message arrives. Might have destination node id! @param
    /// registry_entry gives the registry entry for which the current handler
    /// is being called. @param done must be notified when the processing is
    /// done.
    virtual void handle_identify_global(const EventRegistryEntry &registry_entry,
                                      EventReport *event,
                                      BarrierNotifiable *done) = 0;

    /// Called on another node sending IdentifyConsumer. @param event stores
    /// information about the incoming message. Filled: src_node, event,
    /// mask=1. Not filled: state. @param registry_entry gives the registry
    /// entry for which the current handler is being called. @param done must
    /// be notified when the processing is done.
    virtual void
    handle_identify_consumer(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) = 0;

    /// Called on another node sending IdentifyProducer. @param event stores
    /// information about the incoming message. Filled: src_node, event,
    /// mask=1. Not filled: state.  @param registry_entry gives the registry
    /// entry for which the current handler is being called. @param done must
    /// be notified when the processing is done.
    virtual void
    handle_identify_producer(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) = 0;
};

typedef void (EventHandler::*EventHandlerFunction)(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done);

class EventIterator;

/// Global static object for registering event handlers.
///
/// Usage: create one of the implementation classes depending on the resource
/// requirements of your binary. In the event handlers constructor, register
/// the event handler via the singleton pointer.
///
class EventRegistry : public Singleton<EventRegistry>
{
public:
    virtual ~EventRegistry();

    /** Computes the alignment mask for registering an event range. Updates the
     * event by rounding and returns the mask value to be sent to the
     * register_handler function.
     * @param event is the event id to be registered. Will be modified.
     * @param size is the number of events to register from that offset. [event,
     * event+size) will be the registration range.
     */
    static unsigned align_mask(EventId *event, unsigned size);

    /// Adds a new event handler to the registry.
    virtual void register_handler(const EventRegistryEntry &entry,
                                  unsigned mask) = 0;
    /// Removes all registered instances of a given event handler pointer.
    virtual void unregister_handler(EventHandler *handler) = 0;

    /// Creates a new event iterator. Caller takes ownership of object.
    virtual EventIterator *create_iterator() = 0;

    /// Returns a monotonically increasing number that will change every time
    /// the set of registered event handlers change. Whenever this number
    /// changes, the iterators are invalidated and must be cleared.
    unsigned get_epoch()
    {
        return dirtyCounter_;
    }

protected:
    EventRegistry();

    /// Implementations must call this function from register and unregister
    /// handler to mark iterators being invalidated.
    void set_dirty()
    {
        ++dirtyCounter_;
    }

private:
    static EventRegistry *instance_;

    /// This counter will be incremented every time the set of event handlers
    /// change (and thus the event iterators are invalidated).
    unsigned dirtyCounter_ = 0;

    DISALLOW_COPY_AND_ASSIGN(EventRegistry);
};

}; /* namespace openlcb */

#endif // _OPENLCB_EVENTHANDLER_HXX_

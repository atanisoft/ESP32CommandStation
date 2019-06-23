/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file NonAthoritativeEventProducer.hxx
 *
 * Defines implementations of Event Producers that are uathoratative, meaning
 * that they do not represent the actual state of an event, but can still
 * query that state (presumably from a consumer) and produce a new state.
 *
 * @author Stuart Baker
 * @date 17 June 2017
 */

#ifndef _OPENLCB_NONAUTHORITATIVEEVENTPRODUCER_HXX_
#define _OPENLCB_NONAUTHORITATIVEEVENTPRODUCER_HXX_

#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Event producer for range of bits (event pairs) that is non-autoritative.
class BitRangeNonAuthoritativeEventP : public SimpleEventHandler
{
public:
    /// Constructor.  Creates a new bit range producer.
    ///
    /// @param node the node that the producer will be bound to
    /// @param event_base the starting event ID for the event range
    /// @param size The total event range size in "pairs" of events.  Each
    ///             pair of sequential events represents a single "bit" with
    ///             a binary state.
    /// @param state_callback Callback method for delivering the results of a
    ///                       consumer identified.  The first unsigned parameter
    ///                       represents the bit offset for the range and
    ///                       the second bool parameter indicates the state as
    ///                       true for valid and false for invalid
    BitRangeNonAuthoritativeEventP(Node *node, uint64_t event_base,
                  uint32_t size,
                  std::function<void(unsigned, bool)> state_callback = nullptr)
        : node_(node)
        , eventBase_(event_base)
        , eventBaseOff_(0)
        , size_(size)
        , stateCallback_(state_callback)
    {
        unsigned mask = EventRegistry::align_mask(&event_base, size * 2);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, event_base, EVENT_BASE), mask);
    }

    /// Constructor.  Creates a new bit range producer.
    ///
    /// @param node the node that the producer will be bound to
    /// @param event_base_on the starting event ID for the "on" event range
    /// @param event_base_off the starting event ID for the "off" event range
    /// @param size The total event range size in "pairs" of events.  Each
    ///             pair of sequential events represents a single "bit" with
    ///             a binary state.
    /// @param state_callback Callback method for delivering the results of a
    ///                       consumer identified or event report.  The first
    ///                       unsigned parameter represents the bit offset for
    ///                       the range and the second bool parameter indicates
    ///                       the state as true for valid and false for invalid.
    BitRangeNonAuthoritativeEventP(Node *node, uint64_t event_base_on,
                  uint64_t event_base_off, uint32_t size,
                  std::function<void(unsigned, bool)> state_callback = nullptr)
        : node_(node)
        , eventBaseOn_(event_base_on)
        , eventBaseOff_(event_base_off)
        , size_(size)
        , stateCallback_(state_callback)
    {
        unsigned mask;

        mask = EventRegistry::align_mask(&event_base_on, size);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, event_base_on, EVENT_BASE_ON), mask);

        mask = EventRegistry::align_mask(&event_base_off, size);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, event_base_off, EVENT_BASE_OFF), mask);
    }

    /// Destructor.
    ~BitRangeNonAuthoritativeEventP()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Queries consumer and acquires the current state of the bit.
    ///
    /// @param bit bit pair offset from the base event ID of the range,
    ///            (0 <= bit < size)
    /// @param writer object that will assist in the write transaction
    /// @param done notifible to wakup when finished
    void send_query_consumer(unsigned bit, WriteHelper *writer,
                             BarrierNotifiable *done);

    /// Requests the event associated with the current value of the bit to be
    /// produced (unconditionally).
    ///
    /// @param bit is the offset of the bit to set (0 <= bit < size)
    /// @param new_value is the new value of the bit
    /// @param writer is the output flow to be used
    /// @param done notifible to wakup when query is sent on the bus
    void set(unsigned bit, bool new_value, WriteHelper *writer,
             BarrierNotifiable *done);

    /// Handle an incoming event.
    ///
    /// @param entry reference to this entry in the event registry
    /// @param event event metadata
    /// @param done notifible to wakup when finished with the report processing
    void handle_event_report(const EventRegistryEntry &entry,
                             EventReport *event,
                             BarrierNotifiable *done) override;

    /// Handle an incoming consumer identified message.
    ///
    /// @param entry reference to this entry in the event registry
    /// @param event event metadata
    /// @param done notifible to wakup when finished with the report processing
    void handle_consumer_identified(const EventRegistryEntry &entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override;

    /// Handle an incoming identify global or addressed message.
    ///
    /// @param entry reference to this entry in the event registry
    /// @param event event metadata
    /// @param done notifible to wakup when finished with the report processing
    void handle_identify_global(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;

    /// Handle an incoming identify producer message.
    ///
    /// @param entry reference to this entry in the event registry
    /// @param event event metadata
    /// @param done notifible to wakup when finished with the report processing
    void handle_identify_producer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override;

private:
    /// Identifiers for the types of event range registration.
    enum EventBaseType : unsigned
    {
        // start counting at 1 as 0 is reserved or unused
        EVENT_BASE = 1, /// one unified range starting at @ref eventBase_
        EVENT_BASE_ON, /// on range starting at @ref eventBaseOn_
        EVENT_BASE_OFF, /// on range starting at @ref eventBaseOff_
    };

    Node *node_; ///< Node ID that this producer is attached to
    union
    {
        uint64_t eventBase_; ///< base event ID of the full range
        uint64_t eventBaseOn_; ///< base event ID for "on" range
    };
    uint64_t eventBaseOff_; ///< base event ID for "off" range
    unsigned size_; ///< number of bits stored

    /// Callback method that will be invoked when a consumer identified
    /// message is received with a known state.
    std::function<void(unsigned, bool)> stateCallback_;

    DISALLOW_COPY_AND_ASSIGN(BitRangeNonAuthoritativeEventP);
};

} // namespace openlcb

#endif // _OPENLCB_AUTHORITATIVEEVENTPRODUCER_HXX_


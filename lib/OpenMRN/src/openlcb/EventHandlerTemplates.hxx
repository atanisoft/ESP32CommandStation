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
 * \file EventHandlerTemplates.hxx
 *
 * Defines partial implementations for event handlers that are usable for
 * multiple event handler types.
 *
 * @author Balazs Racz
 * @date 19 October 2013
 */

#ifndef _OPENLCB_EVENTHANDLERTEMPLATES_HXX_
#define _OPENLCB_EVENTHANDLERTEMPLATES_HXX_

#include "openlcb/EventHandler.hxx"
#include "openlcb/WriteHelper.hxx"
#include "os/Gpio.hxx"

namespace openlcb
{

/// Creates a single encoded event range from the beginning of the range and
/// the number fo events to cover. There is no restriction on begin and size
/// (e.g. need ot be power-of-two aligned etc.), the range will be expanded so
/// long as it covers begin + size - 1.
uint64_t EncodeRange(uint64_t begin, unsigned size);

/// A proxy event handler has a single helper function that gets every event
/// handler call with an indication of which call it is. It is helpful to
/// create event containers that proxy calls to many event handler instances.
class ProxyEventHandler : public EventHandler
{
public:
    virtual ~ProxyEventHandler()
    {
    }

    /// This function will be called for any other incoming event handler
    /// function.
    virtual void HandlerFn(EventHandlerFunction fn,
                           const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) = 0;

/// Defines an event handler function as a proxy implementation to the single
/// HandlerFn.
///
/// @param FN C++ name of the function to proxy.
#define DEFPROXYFN(FN)                                                         \
    virtual void FN(const EventRegistryEntry &registry_entry,                  \
                    EventReport *event, BarrierNotifiable *done) override      \
    {                                                                          \
        HandlerFn(&EventHandler::FN, registry_entry, event, done);             \
    }

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

/// SimpleEventHandler ignores all non-essential callbacks. This is used as as
/// a base class for other event handler implementations to avoid duplicate
/// code.
class SimpleEventHandler : public EventHandler
{
public:
/// Defines an event handler function as ignored.
///
/// @param FN name of the function to define.
#define IGNOREFN(FN)                                                           \
    virtual void FN(const EventRegistryEntry &registry_entry,                  \
                    EventReport *event, BarrierNotifiable *done) override      \
    {                                                                          \
        done->notify();                                                        \
    }

    IGNOREFN(handle_event_report);
    IGNOREFN(handle_consumer_identified);
    IGNOREFN(handle_consumer_range_identified);
    IGNOREFN(handle_producer_identified);
    IGNOREFN(handle_producer_range_identified);
    IGNOREFN(handle_identify_consumer);
    IGNOREFN(handle_identify_producer);

#undef IGNOREFN
};

/// Class that advertises an event ID to be produced. It does not do anything
/// else. This feature is used by certain protocols to find nodes supporting a
/// given feature, such as IsTrain and IsCommandStation in the traction
/// protocol.
///
///  usage:
///
///  static const uint64_t IS_TRAIN_EVENT_ID = 0x0101000000000303ULL;
///  FixedEventProducer<IS_TRAIN_EVENT_ID> train_event_producer(&train_node);
///
///  The eventID constant must be available at compile time.
template <uint64_t EVENT_ID>
class FixedEventProducer : public SimpleEventHandler
{
public:
    FixedEventProducer(Node *node) : node_(node)
    {
        /// TODO (balazs.racz) move the event ID argument from the templates to
        /// a constructor argument; use the event registry entry value to send
        /// out messages to the bus.
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, EVENT_ID), 0);
    }

    ~FixedEventProducer()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry, EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        if (event->dst_node && event->dst_node != node_)
        {
            return done->notify();
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            openlcb::Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN,
            WriteHelper::global(), openlcb::eventid_to_buffer(EVENT_ID), done);
    }

    void handle_identify_producer(const EventRegistryEntry &registry_entry, EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        return handle_identify_global(registry_entry, event, done);
    }

private:
    Node *node_;
};

/// Represents a bit of state using two events.
///
/// This class is used as an implementation plugin to various event producers
/// and consumers. It encapsulates application-specific information about the
/// event bit:
///
///  - which OpenLCB node this bit is represented on;
///  - the event identifiers for ON and OFF events;
///  - how to query the hardware for the current event state;
///  - how to set the hardware to a new event state.
///
/// See @ref BitEventProducer, @ref BitEventConsumer, @ref BitEventPC.
class BitEventInterface
{
public:
    BitEventInterface(uint64_t event_on, uint64_t event_off)
        : event_on_(event_on)
        , event_off_(event_off)
    {
    }

    /// returns the current hardware state: true for ON, false for OFF.
    virtual EventState get_current_state() = 0;

    /// Get the requested state. @TODO(stbaker): document the difference
    /// between requested state and current state.
    /// @return requested state
    virtual EventState get_requested_state()
    {
        return get_current_state();
    }

    /// Updates the hardware for the new event state.
    ///
    /// @param new_value is true for state ON, false for state OFF.
    virtual void set_state(bool new_value) = 0;

    /// returns the event ID for representing the state transition OFF->ON.
    uint64_t event_on()
    {
        return event_on_;
    }

    /// returns the event ID for representing the state transition ON->OFF.
    uint64_t event_off()
    {
        return event_off_;
    }

    /// returns the OpenLCB virtual node from which to send the respective
    /// events when the bit changes.
    virtual Node *node() = 0;

private:
    uint64_t event_on_;
    uint64_t event_off_;

    DISALLOW_COPY_AND_ASSIGN(BitEventInterface);
};

/// Implementation of the BitEventInterface that has accessors to the desired
/// and actual state, but does not perform callback on change.
class DistributedBit : public BitEventInterface
{
public:
    DistributedBit(Node *node, uint64_t event_on, uint64_t event_off)
        : BitEventInterface(event_on, event_off)
        , node_(node)
        , state_(EventState::UNKNOWN)
        , requested_(EventState::INVALID)
    {
    }

    /// Destructor.
    ~DistributedBit()
    {
    }

    /// Get a refference to the owning Node.
    /// @return Node reference
    Node *node() override
    {
        return node_;
    }

    /// Get the current state.
    /// @return current state
    EventState get_current_state() override
    {
        return state_;
    }

    /// Get the requested state.
    /// @return requested state
    EventState get_requested_state() override
    {
        return requested_;
    }

    /// Set the current state.
    /// @param new state value
    void set_state(bool new_value) override
    {
        state_ = new_value ? EventState::VALID : EventState::INVALID;
    }

    /// Set the requested state.
    /// @param new requested value
    void set_requested_state(bool new_value)
    {
        requested_ = new_value ? EventState::VALID : EventState::INVALID;
    }

    /// Invert the requested state from the current state.
    void toggle_state()
    {
        requested_ = state_ == EventState::UNKNOWN ? EventState::VALID :
                                                     invert_event_state(state_);
    }

private:
    /// node that this interface is bound to
    Node *node_;

    /// Event state.
    EventState state_;

    /// Event state reauested
    EventState requested_;

    DISALLOW_COPY_AND_ASSIGN(DistributedBit);
};

/// A network-initialized bit is a tri-state bit implementation that starts up
/// in the UNKNOWN state until either a local state set or a remote state set
/// occurs. After that transient passes the state will always be definite. The
/// network state set typically comes from query responses or other nodes
/// setting the state. The owner of this bit will typically want to actively
/// send out a query upon boot or network reconnect.
class NetworkInitializedBit : public BitEventInterface {
public:
    /// Constructs a NetworkInitializedBit.
    ///
    /// @param node the virtual node who exposes this bit.
    /// @param event_on event ID to set the state to true
    /// @param event_off event ID to set the state to false
    /// @param default_local_state Until there is a definite network state we
    /// return this state for a local query. Also determines what state a first
    /// local toggle() call will set to.
    NetworkInitializedBit(Node *node, uint64_t event_on, uint64_t event_off,
        bool default_local_state)
        : BitEventInterface(event_on, event_off)
        , node_(node)
        , isKnown_(0)
        , localState_(default_local_state ? 1 : 0)
    {
    }

    /// Get a reference to the owning Node.
    /// @return Node reference
    Node *node() override
    {
        return node_;
    }

    /// Accessor from the network stack to return the current state.
    /// @return tri-stated value
    EventState get_current_state() override
    {
        if (!isKnown_) return EventState::UNKNOWN;
        if (localState_) return EventState::VALID;
        return EventState::INVALID;
    }

    /// @return the local state, which defaults to default_local_state if the
    /// network state is unknown.
    bool get_local_state() {
        return localState_;
    }

    /// @return true if the network state is known; i.e. when we are sure that
    /// the network state matches the local state.
    bool is_network_state_known() {
        return isKnown_;
    }
    
    /// Call from the network stack (or the client before notifying the network
    /// stack) to set the state. Always sets the state to definite.
    /// @param new state value
    ///
    /// NOTE: this does not send any messages. The caller must use the
    /// EventHandler object after this function to send out an event.
    void set_state(bool new_value) override
    {
        isKnown_ = 1;
        localState_ = new_value ? 1 : 0;
    }

    /// Invert the current state. This also works when the network state is not
    /// yet known: in that case it sets the state to the opposite of the
    /// default state.
    ///
    /// NOTE: this does not send any messages. The caller must use the
    /// EventHandler object after this function to send out an event.
    void toggle_state()
    {
        set_state(!get_local_state());
    }
    
protected:
    Node* node_;
    /// true when we knowthe network state
    uint8_t isKnown_ : 1;
    /// local state; either matches the network state or is the
    /// constructor-default local state.
    uint8_t localState_ : 1;
};

/// Speciallization of NetworkInitializedBit that adds callback support when
/// the state changes.
class CallbackNetworkInitializedBit : public NetworkInitializedBit
{
public:
    /// Constructor.
    ///
    /// @param node the virtual node who exposes this bit.
    /// @param event_on event ID to set the state to true
    /// @param event_off event ID to set the state to false
    /// @param default_local_state Until there is a definite network state we
    ///        return this state for a local query. Also determines what state
    ///        a first local toggle() call will set to.
    CallbackNetworkInitializedBit(openlcb::Node *node, uint64_t event_on,
                                  uint64_t event_off, bool default_local_state)
        : NetworkInitializedBit(node, event_on, event_off, default_local_state)
    {
    }

    /// Specifies the change notifier.
    /// @param cb will be invoked every time the state is changed
    ///        (both from local calls as well as from the network stack).
    void set_change_callback(std::function<void()> cb)
    {
        callback_ = std::move(cb);
    }

    /// Call from the network stack (or the client before notifying the network
    /// stack) to set the state. Always sets the state to definite.
    /// NOTE: this does not send any messages. The caller must use the
    /// EventHandler object after this function to send out an event.
    ///
    /// @param new state value
    void set_state(bool new_value) override
    {
        openlcb::NetworkInitializedBit::set_state(new_value);
        if (callback_)
        {
            callback_();
        }
    }

    /// Call this function in order to reset the network state to unknown.
    void reset()
    {
        isKnown_ = 0;
    }
    
private:
    /// This function is invoked when the state of the bit changes.
    std::function<void()> callback_;
};


/// Simple implementation of a BitEventInterface when the true state ofthe
/// variable is mapped in memory (e.g. mmap-ed gpio, or if there is no real
/// hardware but a bit in RAM).
///
/// The template argument is the C++ type of the raw pointer, usually uint32_t
/// or uint8_t.
template <class T> class MemoryBit : public BitEventInterface
{
public:
    /// @param ptr defines the memory address of the bit where the hardware
    /// state
    /// is located in the address space. @param mask defines which bit at that
    /// address. If there are multiple bits set in mask, they will all be
    /// set/cleared for output purposes, and if any of them is set, the input
    /// will be considered on.
    MemoryBit(Node *node, uint64_t event_on, uint64_t event_off, T *ptr, T mask)
        : BitEventInterface(event_on, event_off)
        , node_(node)
        , ptr_(ptr)
        , mask_(mask)
    {
    }

    Node *node() override
    {
        return node_;
    }
    EventState get_current_state() override
    {
        return ((*ptr_) & mask_) ? EventState::VALID : EventState::INVALID;
    }
    void set_state(bool new_value) override
    {
        if (new_value)
        {
            *ptr_ |= mask_;
        }
        else
        {
            *ptr_ &= ~mask_;
        }
    }

private:
    Node *node_;
    T *ptr_;
    T mask_;

    DISALLOW_COPY_AND_ASSIGN(MemoryBit);
};

/// Simple implementation of the BitEventInterface for going through GPIO
/// ports. The port getter and setter are taken as a GPIO* pointer.
class GPIOBit : public BitEventInterface
{
public:
    GPIOBit(Node *node, EventId event_on, EventId event_off, const Gpio *gpio)
        : BitEventInterface(event_on, event_off)
        , node_(node)
        , gpio_(gpio)
    {
    }

    template <class HW>
    GPIOBit(Node *node, EventId event_on, EventId event_off, const HW &, const Gpio* g = HW::instance(), decltype(HW::instance)* = 0)
        : GPIOBit(node, event_on, event_off, g)
    {
    }

    EventState get_current_state() OVERRIDE
    {
        return gpio_->is_set() ? EventState::VALID : EventState::INVALID;
    }
    void set_state(bool new_value) OVERRIDE
    {
        gpio_->write(new_value);
    }
    Node *node() OVERRIDE
    {
        return node_;
    }

public:
    Node *node_;
    const Gpio *gpio_;
};

/// Base class for single-bit producer and consumer objects.
///
/// Contains helper functions for operations shared by event handlers.
class BitEventHandler : public SimpleEventHandler
{
public:
    BitEventHandler(BitEventInterface *bit);

    /// Requests the event associated with the current value of the bit to be
    /// produced (unconditionally): sends an event report packet ot the bus.
    ///
    /// @param writer is the output flow to be used.
    ///
    /// @param done is the notification callback. If it is NULL, the writer will
    /// be invoked inline and potentially block the calling thread.
    void SendEventReport(WriteHelper *writer, Notifiable *done);

protected:
    /// Registers this event handler with the global event manager. Call this
    /// from the constructor of the derived class.
    void register_handler(uint64_t event_on, uint64_t event_off);
    /// Removes this event handler from the global event manager. Call this
    /// from the destructor of the derived class.
    void unregister_handler();

    /// Sends off two packets using event_write_helper{1,2} of
    /// ProducerIdentified
    /// for handling a global identify events message. Allocates children from
    /// barrier done (but does not notify it).
    ///
    /// @TODO: for consistency of API this function should be changed to notify
    /// the barrier. The caller should always use new_child.
    void SendProducerIdentified(EventReport *event, BarrierNotifiable *done);

    /// Sends off two packets using event_write_helper{3,4} of
    /// ConsumerIdentified
    /// for handling a global identify events message. Allocates children from
    /// barrier done (but does not notify it).
    ///
    /// @TODO: for consistency of API this function should be changed to notify
    /// the barrier. The caller should always use new_child.
    void SendConsumerIdentified(EventReport *event, BarrierNotifiable *done);

    /// Checks if the event in the report is something we are interested in, and
    /// if so, sends off a {Producer|Consumer}Identified{Valid|Invalid} message
    /// depending on the current state of the hardware bit. Uses
    /// event_write_helper<1>. Notifies done.
    void HandlePCIdentify(Defs::MTI mti_valid, EventReport *event,
                          BarrierNotifiable *done);

    BitEventInterface *bit_;

private:
    /// Bits defined in the user_data value for the registrations in this event
    /// handler.
    enum {
        /// This registration is for a single event_on.
        EVENT_ON = 1,
        /// This registration is for a single event_off.
        EVENT_OFF = 2,
        /// This registration is for two events, and the lower numbered is the
        /// event on.
        BOTH_ON_IS_ZERO = 4,
        /// This registration is for two events, and the lower numbered is the
        /// event off.
        BOTH_OFF_IS_ZERO = 8,
    };

    DISALLOW_COPY_AND_ASSIGN(BitEventHandler);
};

/// Event handler for a single-bit producer, e.g. an individual pin of GPIO
/// input. Exports two event producers on the OpenLCB bus, one for the bit
/// state == ON, and another for the bit state == OFF.
///
/// Usage:
///
/// Implement the hardware accessor methods in a descendant of
/// BitEventInterface. Instantiate this class by passing the pointer to it in
/// the constructor. When the hardware state changes, call the @ref
/// SendEventReport method.
class BitEventProducer : public BitEventHandler
{
public:
    BitEventProducer(BitEventInterface *bit) : BitEventHandler(bit)
    {
        /// @TODO (balazs.racz) this should be more efficient when done from
        /// the update configuration callback.
        register_handler(bit->event_on(), bit->event_off());
    }
    ~BitEventProducer()
    {
        unregister_handler();
    }

    /// Requests the event associated with the current value of the bit to be
    /// produced (unconditionally).
    ///
    /// @param writer is the output flow to be used.
    ///
    /// @param done is the notification callback. If it is NULL, the writer will
    /// be invoked inline and potentially block the calling thread.
    ///
    /// @TODO: remove this function and change all callers to use
    /// SendEventReport
    /// instead.
    void Update(WriteHelper *writer, Notifiable *done)
    {
        SendEventReport(writer, done);
    }

    /// Queries consumers and acquires the current state of the bit.
    void SendQuery(WriteHelper *writer, BarrierNotifiable *done);

    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;
    void handle_identify_producer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;

private:
    DISALLOW_COPY_AND_ASSIGN(BitEventProducer);
};

/// Event handler for a single-bit consumer, e.g. an individual pin of GPIO
/// output, or a turnout driver, etc. Exports two event consumers on the
/// OpenLCB bus, one for the bit state == ON, and another for the bit state ==
/// OFF. Calls the hardware implementation when any of the defined events come
/// on the bus.
///
/// Usage:
///
/// Implement the hardware accessor methods in a descendant of
/// BitEventInterface. Instantiate this class by passing the pointer to it in
/// the constructor. When the hardware state changes, call the @ref
/// SendEventReport method.
class BitEventConsumer : public BitEventHandler
{
public:
    BitEventConsumer(BitEventInterface *bit) : BitEventHandler(bit)
    {
        register_handler(bit->event_on(), bit->event_off());
    }
    ~BitEventConsumer()
    {
        unregister_handler();
    }

    /// Queries producers and acquires the current state of the bit.
    void SendQuery(WriteHelper *writer, BarrierNotifiable *done);

    void handle_event_report(const EventRegistryEntry &entry, EventReport *event,
                           BarrierNotifiable *done) override;
    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_producer_identified(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override;
};

/// Producer-Consumer event handler for a single bit represented by two event
/// IDs.
///
/// This event handler exports two events as producer and consumer from the
/// current node. If an event report comes in, the internal bit will be flipped
/// and the necessary setter method of the @ref BitEventInterface will be
/// called.
///
/// When the current state of the bit changes, the application must call @ref
/// SendEventReport on this object, which will produce the necessary event
/// message to the bus.
class BitEventPC : public BitEventConsumer
{
public:
    /// @param bit represents the event bits and the getter/setter of the
    /// hardware state.
    BitEventPC(BitEventInterface *bit) : BitEventConsumer(bit)
    {
    }

    /// Queries producers and acquires the current state of the bit.
    void SendQueryProducer(WriteHelper *writer, BarrierNotifiable *done)
    {
        SendQuery(writer, done);
    }

    /// Queries consumer and acquires the current state of the bit.
    void SendQueryConsumer(WriteHelper *writer, BarrierNotifiable *done);

    void handle_identify_producer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;
    void handle_consumer_identified(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override;
};

/// Producer-Consumer event handler for a sequence of bits represented by a
/// dense block of consecutive event IDs.
class BitRangeEventPC : public SimpleEventHandler
{
public:
    /// Creates a new bit range listener. backing store points to memory of at
    /// least size bits (round up to multiple of 32). This class will advertise
    /// producing and consuming size * 2 events contiguous from
    /// event_base. event_base will turn bit 0 on, event_base + 1 will turn bit
    /// 0
    /// off, event_base + 2 will turn bit 1 on, event_base + 3 will turn bit 1
    /// off, etc.
    BitRangeEventPC(Node *node, uint64_t event_base, uint32_t *backing_store,
                    unsigned size);
    virtual ~BitRangeEventPC();

    /// Requests the event associated with the current value of the bit to be
    /// produced (unconditionally).
    ///
    /// @param node specifies the source node from which to produce the event.
    ///
    /// @param bit is the offset of the bit to set (0 <= bit < size)
    ///
    /// @param new_value is the new value of the bit
    ///
    /// @param writer is the output flow to be used.
    ///
    /// @param done is the notification callback. If it is NULL, the writer will
    /// be invoked inline and potentially block the calling thread.
    void Set(unsigned bit, bool new_value, WriteHelper *writer,
             BarrierNotifiable *done);

    /// @returns the value of a given bit. 0 <= bit < size_.
    bool Get(unsigned bit) const;

    /// Sends out a ProducerRangeIdentified.
    void SendIdentified(WriteHelper *writer, BarrierNotifiable *done);

    void handle_event_report(const EventRegistryEntry &entry, EventReport *event,
                           BarrierNotifiable *done) override;
    void handle_identify_producer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;

    /// @returns the number of bits maintained.
    unsigned size() { return size_; }

private:
    void HandleIdentifyBase(Defs::MTI mti_valid, EventReport *event,
                            BarrierNotifiable *done);
    void GetBitAndMask(unsigned bit, uint32_t **data, uint32_t *mask) const;

    uint64_t event_base_;
    Node *node_;
    uint32_t *data_;
    unsigned size_; //< number of bits stored.
};

/// Consumer event handler for a sequence of bytes represented by a dense block
/// of consecutive event IDs. Each byte has a consecutive block of 256 events.
class ByteRangeEventC : public SimpleEventHandler
{
public:
    /// Creates a new byte range listener. backing store points to memory of at
    /// least size bytes. This class will advertise consuming size * 256 events
    /// contiguous from event_base. event_base will set byte 0 to value 0,
    /// event_base + 1 will set byte 0 to value 1, event_base + 256 will set
    /// byte
    /// 1 to value zero, event_base + 257 will set byte 1 to value 1, etc.
    ByteRangeEventC(Node *node, uint64_t event_base, uint8_t *backing_store,
                    unsigned size);
    virtual ~ByteRangeEventC();

    /// Sends out a ConsumerRangeIdentified.
    void SendIdentified(WriteHelper *writer, BarrierNotifiable *done);

    /// This function is called by the handler when a data value overwrite event
    /// arrives.
    virtual void notify_changed(unsigned offset)
    {
    }

    void handle_event_report(const EventRegistryEntry &entry, EventReport *event,
                           BarrierNotifiable *done) override;
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;

protected:
    /// takes an event ID and checks if we are responsible for it. Returns false
    /// if it is an uninteresting eventid, returns true and fills *data with the
    /// byte pointer and *value with the corresponding value.
    bool DecodeEventId(uint64_t event_id, uint8_t **data, uint8_t *value);

    uint64_t event_base_;
    Node *node_;
    uint8_t *data_;
    unsigned size_; //< number of bytes consumed.
};

/** TODO(balazs.racz): Add another class here, ByteRangeEventPC. It should be
 * in the middle between C and P: it should already have the export
 * functionality of P but not remove the import functionality of C yet. */

/// Producer event handler for a sequence of bytes represented by a dense block
/// of consecutive event IDs. Each byte has a consecutive block of 256 events.
class ByteRangeEventP : public ByteRangeEventC
{
public:
    /// Creates a new byte range producer. backing store points to memory of at
    /// least size bytes. This class will advertise producing size * 256 events
    /// contiguous from event_base. event_base will set byte 0 to value 0,
    /// event_base + 1 will set byte 0 to value 1, event_base + 256 will set
    /// byte 1 to value zero, event_base + 257 will set byte 1 to value 1, etc.
    ByteRangeEventP(Node *node, uint64_t event_base, uint8_t *backing_store,
                    unsigned size);

    /// Requests the event associated with the current value of a specific byte
    /// to
    /// be produced (unconditionally).
    ///
    /// @param byte is the offset of the value to produce (0 <= byte < size)
    ///
    /// @param writer is the output flow to be used.
    ///
    /// @param done is the notification callback. Must not be NULL.
    void Update(unsigned byte, WriteHelper *writer, BarrierNotifiable *done);

    /// Sends out a ProducerRangeIdentified.
    void SendIdentified(WriteHelper *writer, BarrierNotifiable *done);

    // Need to override C behavior.
    void handle_event_report(const EventRegistryEntry &entry, EventReport *event,
                           BarrierNotifiable *done) override;
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    // Own behavior.
    void handle_identify_producer(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;
    void handle_identify_global(const EventRegistryEntry &entry,
                              EventReport *event,
                              BarrierNotifiable *done) override;
    // Responses to possible queries.
    void handle_consumer_identified(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override;
    void handle_consumer_range_identified(const EventRegistryEntry &entry,
                                       EventReport *event,
                                       BarrierNotifiable *done) override;

private:
    /// Creates the eventid of the currently valid value of a given byte.
    uint64_t CurrentEventId(unsigned byte);
};

} // namespace openlcb

#endif // _OPENLCB_EVENTHANDLERTEMPLATES_HXX_

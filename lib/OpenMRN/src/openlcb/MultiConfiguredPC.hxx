/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file MultiConfiguredPC.hxx
 *
 * Producer-Consumer class that uses CDI configuration and many GPIO pins to
 * export a multiple of configurable input or output pins.
 *
 * @author Balazs Racz
 * @date 28 Oct 2018
 */

#ifndef _OPENLCB_MULTICONFIGUREDPC_HXX_
#define _OPENLCB_MULTICONFIGUREDPC_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/RefreshLoop.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

static const char PC_ACTION_MAP[] =
    "<relation><property>0</property><value>Output</value></relation>"
    "<relation><property>1</property><value>Input</value></relation>";

CDI_GROUP(PCLineConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<20>, //
    Name("Description"), Description("User name of this line."));

/// Specifies the event ID to set the output to ON.
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Event On"),
    Description("This event ID will turn the output on / be produced when the "
                "input goes on."));
/// Specifies the event ID to set the output to OFF.
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("This event ID will turn the output off / be produced when the "
                "input goes off."));
CDI_GROUP_END();

/// CDI Configuration for a @ref ConfiguredConsumer.
CDI_GROUP(PCConfig);
enum class ActionConfig : uint8_t
{
    OUTPUT = 0,
    INPUT = 1
};

CDI_GROUP_ENTRY(action, Uint8ConfigEntry, Default(1), MapValues(PC_ACTION_MAP),
    Name("Configuration"));

/// Configures the debounce parameter.
CDI_GROUP_ENTRY(debounce, Uint8ConfigEntry, Name("Debounce parameter"),
    Default(3),
    Description("Used for inputs only. Amount of time to wait for the input to "
                "stabilize before "
                "producing the event. Unit is 30 msec of time. Usually a value "
                "of 2-3 works well in a non-noisy environment. In high noise "
                "(train wheels for example) a setting between 8 -- 15 makes "
                "for a slower response time but a more stable "
                "signal.\nFormally, the parameter tells how many times of "
                "tries, each 30 msec apart, the input must have the same value "
                "in order for that value to be accepted and the event "
                "transition produced."),
    Default(3));

CDI_GROUP_ENTRY(unused, EmptyGroup<1>);
// We factor out the description, and event on/off to a separate group to give
// JMRI a chance to render the make turnout/make sensor buttons.
CDI_GROUP_ENTRY(pc, PCLineConfig);
CDI_GROUP_END();

class MultiConfiguredPC : public ConfigUpdateListener,
                          private SimpleEventHandler,
                          private Polling,
                          private Notifiable
{
public:
    typedef PCConfig config_entry_type;
    typedef QuiesceDebouncer debouncer_type;

    /// Usage: ```
    ///
    /// constexpr const Gpio *const kDirectGpio[] = {
    /// TDRV1_Pin::instance(), TDRV2_Pin::instance(),
    /// TDRV3_Pin::instance(), TDRV4_Pin::instance(),
    /// };
    /// openlcb::MultiConfiguredPC direct_pc(stack.node(),
    ///    kDirectGpio, ARRAYSIZE(kDirectGpio), cfg.seg().direct_pc());
    /// ```
    ///
    /// @param node is the OpenLCB node object from the stack.
    /// @param pins is the list of pins represented by the Gpio* object
    /// instances. Can be constant from FLASH space.
    /// @param size is the length of the list of pins array.
    /// @param config is the repeated group object from the configuration space
    /// that represents the locations of the events.
    template <unsigned N>
    __attribute__((noinline))
    MultiConfiguredPC(Node *node, const Gpio *const *pins, unsigned size,
        const RepeatedGroup<config_entry_type, N> &config)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , offset_(config)
    {
        // Mismatched sizing of the GPIO array from the configuration array.
        HASSERT(size == N);
        ConfigUpdateService::instance()->register_update_listener(this);
        producedEvents_ = new EventId[size * 2];
        std::allocator<debouncer_type> alloc;
        debouncers_ = alloc.allocate(size_);
        for (unsigned i = 0; i < size_; ++i)
        {
            alloc.construct(debouncers_ + i, 3);
        }
    }

    ~MultiConfiguredPC()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
        delete[] producedEvents_;
        std::allocator<debouncer_type> alloc;
        for (unsigned i = 0; i < size_; ++i)
        {
            alloc.destroy(debouncers_ + i);
        }
        alloc.deallocate(debouncers_, size_);
    }

    /// @return the instance to give to the RefreshLoop object.
    Polling *polling()
    {
        return this;
    }

    /// Call from the refresh loop.
    void poll_33hz(WriteHelper *helper, Notifiable *done) override
    {
        nextPinToPoll_ = 0;
        pollingHelper_ = helper;
        pollingDone_ = done;
        this->notify();
    }

    /// Asynchronous callback when the previous polling message has left via
    /// the bus. Used as a poor man's iterative state machine.
    void notify() override
    {
        for (; nextPinToPoll_ < size_; ++nextPinToPoll_)
        {
            auto i = nextPinToPoll_;
            if (pins_[i]->direction() == Gpio::Direction::DOUTPUT)
            {
                continue;
            }
            if (debouncers_[i].update_state(pins_[i]->is_set()))
            {
                // Pin flipped.
                ++nextPinToPoll_; // avoid infinite loop.
                auto event = producedEvents_[2 * i +
                    (debouncers_[i].current_state() ? 1 : 0)];
                pollingHelper_->WriteAsync(node_, Defs::MTI_EVENT_REPORT,
                    WriteHelper::global(), eventid_to_buffer(event), this);
                return;
            }
        }
        pollingDone_->notify();
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            // There is no way to figure out what the previously registered
            // eventid values were for the individual pins. Therefore we always
            // unregister everything and register them anew. It also causes us
            // to identify all. This is not a problem since apply_configuration
            // is coming from a user action.
            do_unregister();
        }
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event_on = cfg_ref.pc().event_on().read(fd);
            EventId cfg_event_off = cfg_ref.pc().event_off().read(fd);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_off, i * 2), 0);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_on, i * 2 + 1), 0);
            uint8_t action = cfg_ref.action().read(fd);
            if (action == (uint8_t)PCConfig::ActionConfig::OUTPUT)
            {
                pins_[i]->set_direction(Gpio::Direction::DOUTPUT);
                producedEvents_[i * 2] = 0;
                producedEvents_[i * 2 + 1] = 0;
            }
            else
            {
                uint8_t param = cfg_ref.debounce().read(fd);
                pins_[i]->set_direction(Gpio::Direction::DINPUT);
                debouncers_[i].reset_options(param);
                debouncers_[i].initialize(pins_[i]->read());
                producedEvents_[i * 2] = cfg_event_off;
                producedEvents_[i * 2 + 1] = cfg_event_on;
            }
        }
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).pc().description().write(fd, "");
            CDI_FACTORY_RESET(grp_ref.entry(i).action);
            CDI_FACTORY_RESET(grp_ref.entry(i).debounce);
        }
    }

    /// Factory reset helper function. Sets all names to something 1..N.
    /// @param fd pased on from factory reset argument.
    /// @param basename name of repeats.
    void factory_reset_names(int fd, const char *basename)
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            string v(basename);
            v.push_back(' ');
            char buf[10];
            unsigned_integer_to_buffer(i + 1, buf);
            v += buf;
            grp_ref.entry(i).pc().description().write(fd, v);
        }
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DINPUT)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
        else
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DOUTPUT)
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_producer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DINPUT)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        const Gpio *pin = pins_[registry_entry.user_arg >> 1];
        if (pin->direction() == Gpio::Direction::DOUTPUT)
        {
            const bool is_on = (registry_entry.user_arg & 1);
            pin->write(is_on);
        }
    }

private:
    /// Removes registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        unsigned b1 = pins_[registry_entry.user_arg >> 1]->is_set() ? 1 : 0;
        unsigned b2 = registry_entry.user_arg & 1; // on or off event?
        if (b1 ^ b2)
        {
            mti++; // INVALID
        }
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    /// Sends out a ProducerIdentified message for the given registration
    /// entry.
    void SendProducerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_PRODUCER_IDENTIFIED_VALID;
        unsigned b1 = pins_[registry_entry.user_arg >> 1]->is_set() ? 1 : 0;
        unsigned b2 = registry_entry.user_arg & 1; // on or off event?
        if (b1 ^ b2)
        {
            mti++; // INVALID
        }
        event->event_write_helper<4>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    // Variables used for asynchronous state during the polling loop.
    /// Which pin to next check when polling.
    unsigned nextPinToPoll_;
    /// Write helper to use for producing messages during the polling loop.
    WriteHelper *pollingHelper_;
    /// Notifiable to call when the polling loop is done.
    Notifiable *pollingDone_;

    /// virtual node to export the consumer / producer on.
    Node *node_;
    /// Array of all GPIO pins to use. Externally owned.
    const Gpio *const *pins_;
    /// Number of GPIO pins to export.
    size_t size_;
    /// Offset in the configuration space for our configs.
    ConfigReference offset_;
    /// Event IDs shadowing from the config file for producing them. We own
    /// this memory.
    EventId *producedEvents_;
    /// One debouncer per pin, created for produced pins. We own this memory.
    debouncer_type *debouncers_;
};
}

#endif // _OPENLCB_MULTICONFIGUREDPC_HXX_

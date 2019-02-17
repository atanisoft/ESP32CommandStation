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
 * \file ConfiguredConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a GPIO template structure to
 * export a single bit as two event consumers to OpenLCB.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _OPENLCB_CONFIGUREDCONSUMER_HXX_
#define _OPENLCB_CONFIGUREDCONSUMER_HXX_

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "openlcb/RefreshLoop.hxx"

namespace openlcb
{

/// CDI Configuration for a @ref ConfiguredConsumer.
CDI_GROUP(ConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<8>, //
                Name("Description"), Description("User name of this output."));
/// Specifies the event ID to set the output to ON.
CDI_GROUP_ENTRY(
    event_on, EventConfigEntry, //
    Name("Event On"),
    Description("Receiving this event ID will turn the output on."));
/// Specifies the event ID to set the output to OFF.
CDI_GROUP_ENTRY(
    event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("Receiving this event ID will turn the output off."));
CDI_GROUP_END();

/// CDI Configuration for a @ref ConfiguredPulseConsumer.
CDI_GROUP(PulseConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<16>, //
                Name("Description"), Description("User name of this output."));
/// Specifies the event ID to trigger the output pulse.
CDI_GROUP_ENTRY(
    event, EventConfigEntry, //
    Name("Event"),
    Description(
        "Receiving this event ID will generate a pulse on the output."));
/// Allows the user to configure the output pulse length.
CDI_GROUP_ENTRY(
    duration, Uint8ConfigEntry, Default(3), //
    Name("Pulse duration"),
    Description("Length of the pulse to output (unit of 30 msec)."));
CDI_GROUP_END();

/// OpenLCB Consumer class integrating a simple CDI-based configuration for two
/// event IDs, and an output GPIO object that will be turned on or off
/// depending on the incoming event notifications. This is usually the most
/// important object for a simple IO node.
class ConfiguredConsumer : public ConfigUpdateListener
{
public:
    using Impl = GPIOBit;

    ConfiguredConsumer(Node *node, const ConsumerConfig &cfg, const Gpio *gpio)
        : impl_(node, 0, 0, gpio)
        , consumer_(&impl_)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    template <class HW>
    ConfiguredConsumer(Node *node, const ConsumerConfig &cfg, const HW &, const Gpio* g = HW::instance(), decltype(HW::instance)* = 0)
        : impl_(node, 0, 0, g)
        , consumer_(&impl_)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);
        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        if (cfg_event_off != impl_.event_off() ||
            cfg_event_on != impl_.event_on())
        {
            auto saved_gpio = impl_.gpio_;
            auto saved_node = impl_.node();
            // Need to reinitialize the consumer. We do this with in-place
            // destruction and construction.
            consumer_.~BitEventConsumer();
            impl_.Impl::~Impl();
            new (&impl_)
                Impl(saved_node, cfg_event_on, cfg_event_off, saved_gpio);
            new (&consumer_) BitEventConsumer(&impl_);
            return REINIT_NEEDED; // Causes events identify.
        }
        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
    }

private:
    Impl impl_;
    BitEventConsumer consumer_;
    const ConsumerConfig cfg_;
};

/// OpenLCB Consumer class integrating a simple CDI-based configuration for a
/// single event IDs, and an output GPIO object that will be turned on for a
/// short period of time when the incoming event notifications arrives. Useful
/// for driving magnetic coil-based turnouts and signals.
class ConfiguredPulseConsumer : public ConfigUpdateListener,
                                private SimpleEventHandler,
                                public Polling
{
public:
    template <class HW>
    ConfiguredPulseConsumer(Node *node, const PulseConsumerConfig &cfg,
                            const HW &, const Gpio* g = HW::instance())
        : node_(node)
        , gpio_(g)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ConfiguredPulseConsumer(Node *node, const PulseConsumerConfig &cfg,
                            const Gpio *gpio)
        : node_(node)
        , gpio_(gpio)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredPulseConsumer()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);
        EventId cfg_event_on = cfg_.event().read(fd);
        pulseLength_ = cfg_.duration().read(fd);
        if (cfg_event_on == event_)
            return UPDATED; // nothing to do
        if (!initial_load)
        {
            do_unregister();
        }
        event_ = cfg_event_on;
        do_register();
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.duration);
    }

private:
    /// Registers the event handler with the global event registry.
    void do_register()
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, event_), 0);
    }

    /// Removed registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    // Implementations for the event handler functions.

    void handle_identify_global(const EventRegistryEntry &registry_entry,
                              EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        if (event->dst_node && event->dst_node != node_)
        {
            return done->notify();
        }
        SendConsumerIdentified(event, done);
    }

    void SendConsumerIdentified(EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        if (!pulseRemaining_)
        {
            mti++; // INVALID
        }
        event->event_write_helper<3>()->WriteAsync(
            node_, mti, WriteHelper::global(), eventid_to_buffer(event_), done);
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->event != event_)
        {
            return done->notify();
        }
        SendConsumerIdentified(event, done);
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->event == event_)
        {
            pulseRemaining_ = pulseLength_;
        }
        done->notify();
    }

    // Polling interface
    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (pulseRemaining_ > 0)
        {
            gpio_->set();
            --pulseRemaining_;
        }
        else
        {
            gpio_->clr();
        }
        done->notify();
    }

    Node *node_;                    //< virtual node to export the consumer on
    const Gpio *gpio_;              //< hardware output pin to drive
    EventId event_{0};              //< Event ID to listen for
    const PulseConsumerConfig cfg_; //< offset to the config in EEPROM
    uint8_t pulseLength_{1};        //< length of pulse (in polling count)
    uint8_t pulseRemaining_{0};     //< remaining polling count to keep pulse on
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDCONSUMER_HXX_

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
 * \file ConfiguredProducer.hxx
 *
 * Producer class that uses CDI configuration and a GPIO template structure to
 * export a single bit as two event producers to OpenLCB.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _OPENLCB_CONFIGUREDPRODUCER_HXX_
#define _OPENLCB_CONFIGUREDPRODUCER_HXX_

#include "openlcb/PolledProducer.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "utils/Debouncer.hxx"

namespace openlcb
{

/// CDI Configuration for a @ref ConfiguredProducer.
CDI_GROUP(ProducerConfig);
/// Allows the user to assign a name for this input.
CDI_GROUP_ENTRY(description, StringConfigEntry<15>, //
                Name("Description"), Description("User name of this input."));
/// Configures the debounce parameter.
CDI_GROUP_ENTRY(
    debounce, Uint8ConfigEntry, Name("Debounce parameter"),
    Default(3),
    Description("Amount of time to wait for the input to stabilize before "
                "producing the event. Unit is 30 msec of time. Usually a value "
                "of 2-3 works well in a non-noisy environment. In high noise "
                "(train wheels for example) a setting between 8 -- 15 makes "
                "for a slower response time but a more stable "
                "signal.\nFormally, the parameter tells how many times of "
                "tries, each 30 msec apart, the input must have the same value "
                "in order for that value to be accepted and the event "
                "transition produced."),
    Default(3));
/// This event will be produced when the input goes to HIGH.
CDI_GROUP_ENTRY(
    event_on, EventConfigEntry, //
    Name("Event On"),
    Description("This event will be produced when the input goes to HIGH."));
/// This event will be produced when the input goes to LOW.
CDI_GROUP_ENTRY(
    event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("This event will be produced when the input goes to LOW."));
CDI_GROUP_END();

extern "C" {
void ignore_fn();
}

/// OpenLCB Producer class integrating a simple CDI-based configuration for two
/// event IDs, and an input GPIO object whose value will determine when to
/// produce events. This is usually the most important object for a simple IO
/// node.
///
/// Usage: Must be called repeatedly via the Polling implementation exposed by
/// @ref polling(). Use for example the @ref RefreshLoop class and supply the
/// polling argument at the constructor to it:
/// 
/// openlcb::RefreshLoop loop(
///    stack.node(), {producer_sw1.polling(), producer_sw2.polling()});
class ConfiguredProducer : public ConfigUpdateListener
{
public:
    using Impl = GPIOBit;
    using ProducerClass = PolledProducer<QuiesceDebouncer, Impl>;

    ConfiguredProducer(Node *node, const ProducerConfig &cfg, const Gpio *gpio)
        : producer_(QuiesceDebouncer::Options(3), node, 0, 0, gpio)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    template <class HW>
    ConfiguredProducer(Node *node, const ProducerConfig &cfg, const HW &,
        const Gpio *g = HW::instance(), decltype(HW::instance) * = 0)
        : producer_(QuiesceDebouncer::Options(3), node, 0, 0, g)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        uint8_t debounce_arg = cfg_.debounce().read(fd);
        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        if (cfg_event_off != producer_.event_off() ||
            cfg_event_on != producer_.event_on())
        {
            auto saved_gpio = producer_.gpio_;
            auto saved_node = producer_.node();
            // Need to reinitialize the producer. We do this with in-place
            // destruction and construction.
            producer_.ProducerClass::~ProducerClass();
            new (&producer_) ProducerClass(
                QuiesceDebouncer::Options(debounce_arg), saved_node,
                cfg_event_on, cfg_event_off, saved_gpio);
            return REINIT_NEEDED; // Causes events identify.
        }
        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.debounce);
    }

    Polling *polling()
    {
        return &producer_;
    }

private:
    ProducerClass producer_;
    const ProducerConfig cfg_;
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDPRODUCER_HXX_

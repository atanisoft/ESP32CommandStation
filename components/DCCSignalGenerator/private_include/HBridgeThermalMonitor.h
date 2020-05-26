/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef H_BRIDGE_THERMAL_MONITOR_H_
#define H_BRIDGE_THERMAL_MONITOR_H_

#include "TrackOutputDescriptor.h"

#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/Node.hxx>
#include <openlcb/PolledProducer.hxx>
#include <os/Gpio.hxx>
#include <utils/ConfigUpdateListener.hxx>
#include <utils/Debouncer.hxx>

namespace esp32cs
{

class HBridgeThermalMonitor : public DefaultConfigUpdateListener
{
public:
    using ProducerClass =
        openlcb::PolledProducer<QuiesceDebouncer, openlcb::GPIOBit>;

    HBridgeThermalMonitor(openlcb::Node *node
                        , const TrackOutputConfig &cfg
                        , const Gpio *gpio)
                        : producer_(QuiesceDebouncer::Options(2)
                                  , node, 0, 0, gpio)
                        , cfg_(cfg)
    {
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        UpdateAction res = UPDATED;
        uint8_t thermal_debounce = cfg_.thermal_debounce().read(fd);
        openlcb::EventId thermal_shutdown = cfg_.event_thermal_shutdown().read(fd);
        openlcb::EventId thermal_shutdown_cleared = cfg_.event_thermal_shutdown_cleared().read(fd);
        if (thermal_shutdown_cleared != producer_.event_off() || thermal_shutdown != producer_.event_on())
        {
            auto saved_gpio = producer_.gpio_;
            auto saved_node = producer_.node();
            producer_.ProducerClass::~ProducerClass();
            new (&producer_) ProducerClass(
                QuiesceDebouncer::Options(thermal_debounce), saved_node,
                thermal_shutdown, thermal_shutdown_cleared, saved_gpio);
            res = REINIT_NEEDED;
        }
        return res;
    }

    void factory_reset(int fd) override
    {
        CDI_FACTORY_RESET(cfg_.thermal_debounce);
    }

    openlcb::Polling *polling()
    {
        return &producer_;
    }

private:
    ProducerClass producer_;
    const TrackOutputConfig cfg_;
};

} // namespace esp32cs

#endif // H_BRIDGE_THERMAL_MONITOR_H_
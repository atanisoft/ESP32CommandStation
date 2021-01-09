/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2021 Mike Dunston

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

#ifndef MONITORED_H_BRIDGE_
#define MONITORED_H_BRIDGE_

#include "TrackOutputDescriptor.h"
#include "sdkconfig.h"

#include <executor/StateFlow.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/Node.hxx>
#include <openlcb/PolledProducer.hxx>
#include <os/Gpio.hxx>
#include <os/OS.hxx>
#include <utils/ConfigUpdateListener.hxx>
#include <utils/Debouncer.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_bit_defs.h>

namespace esp32cs
{

class HBridgeShortDetector : public DefaultConfigUpdateListener
                           , public openlcb::Polling
{
public:
  HBridgeShortDetector(openlcb::Node *node
                     , const adc1_channel_t senseChannel
                     , const Gpio *enablePin
                     , const uint32_t limitMilliAmps
                     , const uint32_t maxMilliAmps
                     , const string &name
                     , const string &bridgeType
                     , const esp32cs::TrackOutputConfig &cfg);

  HBridgeShortDetector(openlcb::Node *node
                     , const adc1_channel_t senseChannel
                     , const Gpio *enablePin
                     , const uint32_t
                     , const std::string &name
                     , const std::string &bridgeType
                     , const esp32cs::TrackOutputConfig &cfg);

  enum STATE : uint8_t
  {
    STATE_OVERCURRENT       = BIT(0)
  , STATE_SHUTDOWN          = BIT(1)
  , STATE_ON                = BIT(2)
  , STATE_OFF               = BIT(3)
  };

  std::string getName()
  {
    return name_;
  }

  uint32_t getMaxMilliAmps()
  {
    return maxMilliAmps_;
  }

  uint32_t getLastReading()
  {
    return lastReading_;
  }

  bool isProgrammingTrack()
  {
    return isProgTrack_;
  }

  bool isEnabled()
  {
    return state_ != STATE_OFF;
  }

  float getUsage()
  {
    if (state_ != STATE_OFF)
    {
      return ((lastReading_ * maxMilliAmps_) / 4096.0f);
    }
    return 0.0f;
  }

  std::string getState();

  std::string getStateAsJson();

  std::string getStatusData();

  std::string get_state_for_dccpp();

  UpdateAction apply_configuration(int fd, bool initial_load, BarrierNotifiable *done) override
  {
    AutoNotify n(done);
    UpdateAction res = initial_load ? REINIT_NEEDED : UPDATED;
    openlcb::EventId short_detected = cfg_.event_short().read(fd);
    openlcb::EventId short_cleared = cfg_.event_short_cleared().read(fd);
    openlcb::EventId shutdown = cfg_.event_shutdown().read(fd);
    openlcb::EventId shutdown_cleared = cfg_.event_shutdown_cleared().read(fd);

    auto saved_node = shortBit_.node();
    if (short_detected != shortBit_.event_on() ||
        short_cleared != shortBit_.event_off())
    {
      shortBit_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
      new (&shortBit_)openlcb::MemoryBit<uint8_t>(saved_node, short_detected, short_cleared, &state_, STATE_OVERCURRENT);
      shortProducer_.openlcb::BitEventProducer::~BitEventProducer();
      new (&shortProducer_)openlcb::BitEventProducer(&shortBit_);
      res = REINIT_NEEDED;
    }

    if (shutdown != shortBit_.event_on() ||
        shutdown_cleared != shortBit_.event_off())
    {
      saved_node = shutdownBit_.node();
      shutdownBit_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
      new (&shutdownBit_)openlcb::MemoryBit<uint8_t>(saved_node, shutdown, shutdown_cleared, &state_, STATE_SHUTDOWN);
      shutdownProducer_.openlcb::BitEventProducer::~BitEventProducer();
      new (&shutdownProducer_)openlcb::BitEventProducer(&shutdownBit_);
      res = REINIT_NEEDED;
    }
    configure();
    return res;
  }

  void factory_reset(int fd) override
  {
    LOG(INFO
      , "[LCC] MonitoredHBridge(%s) factory_reset(%d) invoked, defaulting "
        "configuration", name_.c_str(), fd);
    cfg_.description().write(fd, name_.c_str());
  }

  void poll_33hz(openlcb::WriteHelper *helper, Notifiable *done) override;

  void enable_prog_response(bool enable)
  {
    progEnable_ = enable;
  }

private:
  const adc1_channel_t channel_;
  const Gpio *enablePin_;
  const Gpio *thermalWarningPin_;
  const uint32_t maxMilliAmps_;
  const std::string name_;
  const std::string bridgeType_;
  const uint32_t overCurrentLimit_;
  const uint32_t shutdownLimit_;
  const bool isProgTrack_;
  const uint32_t progAckLimit_;
  const esp32cs::TrackOutputConfig cfg_;
  const uint8_t targetLED_;
  const uint8_t adcSampleCount_{CONFIG_ADC_AVERAGE_READING_COUNT};
  const uint8_t overCurrentRetryCount_{CONFIG_DCC_HBRIDGE_OVERCURRENT_BEFORE_SHUTDOWN};
  const uint64_t currentReportInterval_{SEC_TO_USEC(CONFIG_DCC_HBRIDGE_USAGE_REPORT_INTERVAL)};
  uint32_t warnLimit_{0};
  openlcb::MemoryBit<uint8_t> shortBit_;
  openlcb::MemoryBit<uint8_t> shutdownBit_;
  openlcb::BitEventProducer shortProducer_;
  openlcb::BitEventProducer shutdownProducer_;
  uint64_t lastReport_{0};
  uint32_t lastReading_{0};
  uint8_t state_{STATE_OFF};
  uint8_t overCurrentCheckCount_{0};
  bool progEnable_{false};

  void configure();
};

} // namespace esp32cs
#endif // MONITORED_H_BRIDGE_
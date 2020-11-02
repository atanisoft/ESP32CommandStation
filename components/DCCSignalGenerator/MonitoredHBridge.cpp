/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#include "MonitoredHBridge.h"
#include <dcc/ProgrammingTrackBackend.hxx>
#include <json.hpp>
#include <numeric>
#include <StatusLED.h>

namespace esp32cs
{

HBridgeShortDetector::HBridgeShortDetector(openlcb::Node *node
                                         , const adc1_channel_t senseChannel
                                         , const Gpio *enablePin
                                         , const uint32_t limitMilliAmps
                                         , const uint32_t maxMilliAmps
                                         , const string &name
                                         , const string &bridgeType
                                         , const esp32cs::TrackOutputConfig &cfg)
  : DefaultConfigUpdateListener()
  , channel_(senseChannel)
  , enablePin_(enablePin)
  , maxMilliAmps_(maxMilliAmps)
  , name_(name)
  , bridgeType_(bridgeType)
  , overCurrentLimit_(((((limitMilliAmps << 3) + limitMilliAmps) / 10) << 12) / maxMilliAmps_) // ~90% max value
  , shutdownLimit_(4080)
  , isProgTrack_(false)
  , progAckLimit_(0)
  , cfg_(cfg)
  , targetLED_(StatusLED::LED::OPS_TRACK)
  , shortBit_(node, 0, 0, &state_, STATE_OVERCURRENT)
  , shutdownBit_(node, 0, 0, &state_, STATE_SHUTDOWN)
  , shortProducer_(&shortBit_)
  , shutdownProducer_(&shortBit_)
{
  // set warning limit to ~75% of overcurrent limit
  warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
}

HBridgeShortDetector::HBridgeShortDetector(openlcb::Node *node
                                         , const adc1_channel_t senseChannel
                                         , const Gpio *enablePin
                                         , const uint32_t maxMilliAmps
                                         , const string &name
                                         , const string &bridgeType
                                         , const esp32cs::TrackOutputConfig &cfg)
  : DefaultConfigUpdateListener()
  , channel_(senseChannel)
  , enablePin_(enablePin)
  , maxMilliAmps_(maxMilliAmps)
  , name_(name)
  , bridgeType_(bridgeType)
  , overCurrentLimit_((250 << 12) / maxMilliAmps_) // ~250mA
  , shutdownLimit_((500 << 12) / maxMilliAmps_)
  , isProgTrack_(true)
  , progAckLimit_((60 << 12) / maxMilliAmps_)      // ~60mA
  , cfg_(cfg)
  , targetLED_(StatusLED::LED::PROG_TRACK)
  , shortBit_(node, 0, 0, &state_, STATE_OVERCURRENT)
  , shutdownBit_(node, 0, 0, &state_, STATE_SHUTDOWN)
  , shortProducer_(&shortBit_)
  , shutdownProducer_(&shortBit_)
{
  // set warning limit to ~75% of overcurrent limit
  warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
}

string HBridgeShortDetector::getState()
{
  switch (state_)
  {
    case STATE_ON:
      return "Normal";
    case STATE_OVERCURRENT:
      return "Fault";
    case STATE_SHUTDOWN:
      return "Shutdown";
    case STATE_OFF:
    default:
      return "Off";
  }
  return "Error";
}

string HBridgeShortDetector::getStateAsJson()
{
  return StringPrintf("{"
                        "\"name\":\"%s\","
                        "\"state\":\"%s\","
                        "\"usage\":%.2f,"
                        "\"prog\":%s"
                      "}"
                      , name_.c_str(), getState().c_str(), getUsage()
                      , isProgrammingTrack() ? "true" : "false"
                      );
}

string HBridgeShortDetector::getStatusData()
{
  if (state_ == STATE_ON)
  {
    return StringPrintf("%s:On (%.2f mA)", name_.c_str(), getUsage());
  }
  else if (state_ == STATE_OVERCURRENT)
  {
    return StringPrintf("%s:F (%.2f mA)", name_.c_str(), getUsage());
  }
  return StringPrintf("%s:Off", name_.c_str());
}

string HBridgeShortDetector::get_state_for_dccpp()
{
  if (state_ == STATE_ON)
  {
    // NOTE: the <a TRACK READING> is 0-1023 due to AVR only using 10-bit ADC
    // and ESP32 CS using 12 bit (0-4095). Due to this the last reading needs
    // to be divided by four to keep it in range for JMRI to display it
    // correctly.
    return StringPrintf("<p1 %s><a %s %d>", name_.c_str(), name_.c_str()
                      , getLastReading() / 4);
  }
  else if (state_ == STATE_OVERCURRENT)
  {
    return StringPrintf("<p2 %s>", name_.c_str());
  }
  return StringPrintf("<p0 %s>", name_.c_str());
}

void HBridgeShortDetector::configure(bool initial_load)
{
  if (initial_load)
  {
    adc1_config_channel_atten(channel_, (adc_atten_t)CONFIG_ADC_ATTENUATION);
    LOG(INFO, "[%s] Configuring H-Bridge (%s %u mA max) using ADC 1:%d"
      , name_.c_str(), bridgeType_.c_str(), maxMilliAmps_, channel_);
    if (isProgTrack_)
    {
      LOG(INFO, "[%s] Prog ACK: %u/4096 (%6.2f mA)", name_.c_str()
        , progAckLimit_, ((progAckLimit_ * maxMilliAmps_) / 4096.0f));
    }
  }
  LOG(INFO, "[%s] Short limit %u/4096 (%6.2f mA), events (on: %s, off: %s)"
    , name_.c_str(), overCurrentLimit_
    , ((overCurrentLimit_ * maxMilliAmps_) / 4096.0f)
    , uint64_to_string_hex(shortBit_.event_on()).c_str()
    , uint64_to_string_hex(shortBit_.event_off()).c_str());
  LOG(INFO, "[%s] Shutdown limit %u/4096 (%6.2f mA), events (on: %s, off: %s)"
    , name_.c_str(), shutdownLimit_
    , ((shutdownLimit_ * maxMilliAmps_) / 4096.0f)
    , uint64_to_string_hex(shutdownBit_.event_on()).c_str()
    , uint64_to_string_hex(shutdownBit_.event_off()).c_str());
}

void HBridgeShortDetector::poll_33hz(openlcb::WriteHelper *helper
                                   , Notifiable *done)
{
  vector<int> samples;

  // collect samples from ADC
  while (samples.size() < adcSampleCount_)
  {
    samples.push_back(adc1_get_raw(channel_));
  }
  // average the collected samples
  lastReading_ = (std::accumulate(samples.begin(), samples.end(), 0) / samples.size());

  // if this is the PROG track check up front if we have a short or ACK.
  if (isProgTrack_ && progEnable_)
  {
    LOG(VERBOSE, "[%s] reading: %d", name_.c_str(), lastReading_);
    auto backend = Singleton<ProgrammingTrackBackend>::instance();
    if (lastReading_ >= overCurrentLimit_)
    {
      // note that only over current is checked here since this should be
      // triggered before the shutdown current has been reached.
      backend->notify_service_mode_short();
    }
    else if (lastReading_ >= progAckLimit_)
    {
      // send the ack over to the backend since it is over the limit.
      backend->notify_service_mode_ack();
    }
  }

  uint8_t previous_state = state_;

  if (lastReading_ >= shutdownLimit_)
  {
    // If the average sample exceeds the shutdown limit (~90% typically)
    // trigger an immediate shutdown.
    LOG_ERROR("[%s] Shutdown threshold breached %6.2f mA (raw: %d / %d)"
            , name_.c_str(), getUsage(), lastReading_, shutdownLimit_);
    enablePin_->clr();
    state_ = STATE_SHUTDOWN;
#if CONFIG_STATUS_LED
    Singleton<StatusLED>::instance()->setStatusLED((StatusLED::LED)targetLED_
                                                 , StatusLED::COLOR::RED_BLINK);
#endif // CONFIG_STATUS_LED
  }
  else if (lastReading_ >= overCurrentLimit_)
  {
    // If we have at least a couple averages that are over the soft limit
    // trigger an immediate shutdown as a short is likely to have occurred.
    if (overCurrentCheckCount_++ >= overCurrentRetryCount_)
    {
      // disable the h-bridge output
      enablePin_->clr();
      LOG_ERROR("[%s] Overcurrent detected %6.2f mA (raw: %d / %d)"
              , name_.c_str(), getUsage(), lastReading_, overCurrentLimit_);
      state_ = STATE_OVERCURRENT;
#if CONFIG_STATUS_LED
      Singleton<StatusLED>::instance()->setStatusLED((StatusLED::LED)targetLED_
                                                   , StatusLED::COLOR::RED);
#endif // CONFIG_STATUS_LED
    }
  }
  else
  {
    if (enablePin_->is_set())
    {
      overCurrentCheckCount_ = 0;
      state_ = STATE_ON;
#if CONFIG_STATUS_LED
      // check if we are over the warning limit and update the LED accordingly.
      if (lastReading_ >= warnLimit_)
      {
        Singleton<StatusLED>::instance()->setStatusLED((StatusLED::LED)targetLED_
                                                    , StatusLED::COLOR::YELLOW);
      }
      else
      {
        Singleton<StatusLED>::instance()->setStatusLED((StatusLED::LED)targetLED_
                                                    , StatusLED::COLOR::GREEN);
      }
#endif // CONFIG_STATUS_LED
    }
    else
    {
      state_ = STATE_OFF;
    }
  }
  if (state_ == STATE_ON &&
     (esp_timer_get_time() - lastReport_) >= currentReportInterval_)
  {
    lastReport_ = esp_timer_get_time();
    LOG(INFO, "[%s] %6.2f mA / %d mA", name_.c_str(), getUsage(), maxMilliAmps_);
  }

  // if our state has changed send out applicable events
  bool async_event_req = false;
  if (previous_state != state_)
  {
    if (previous_state == STATE_SHUTDOWN || state_ == STATE_SHUTDOWN)
    {
      shutdownProducer_.SendEventReport(helper, done);
      async_event_req = true;
    }
    else if (previous_state == STATE_OVERCURRENT || state_ == STATE_OVERCURRENT)
    {
      shortProducer_.SendEventReport(helper, done);
      async_event_req = true;
    }
  }

  if (!async_event_req)
  {
    done->notify();
  }
}

} // namespace esp32cs

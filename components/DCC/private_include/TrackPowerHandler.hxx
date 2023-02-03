/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020-2021 Mike Dunston

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
#ifndef TRACK_POWER_HANDLER_HXX_
#define TRACK_POWER_HANDLER_HXX_

#include <dcc/DccOutput.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <utils/StringUtils.hxx>

namespace esp32cs
{
class TrackPowerBit : public openlcb::BitEventInterface
{
public:
  TrackPowerBit(openlcb::Node *node)
    : openlcb::BitEventInterface(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT,
                                 openlcb::Defs::EMERGENCY_OFF_EVENT),
    node_(node)
  {
    LOG(INFO,
       "[Track Power] Registering OpenLCB event consumer (On:%s, Off:%s)",
       utils::event_id_to_string(event_on()).c_str(),
       utils::event_id_to_string(event_off()).c_str());
  }

  openlcb::EventState get_current_state() override
  {
    LOG(VERBOSE, "[Track Power] Query event state: %d",
        get_dcc_output(DccOutput::TRACK)->get_disable_output_reasons());
    if (get_dcc_output(DccOutput::TRACK)->get_disable_output_reasons())
    {
      LOG(VERBOSE, "[Track Power] OFF (%s)",
          utils::event_id_to_string(event_off()).c_str());
      return openlcb::EventState::INVALID;
    }
    LOG(VERBOSE, "[Track Power] ON (%s)",
        utils::event_id_to_string(event_on()).c_str());
    return openlcb::EventState::VALID;
  }

  void set_state(bool new_value) override
  {
    auto track = get_dcc_output(DccOutput::TRACK);
    if (new_value)
    {
      LOG(INFO, "[Track Power] Clearing global emergency off");
      track->clear_disable_output_for_reason(DccOutput::DisableReason::GLOBAL_EOFF);
    }
    else
    {
      LOG(INFO, "[Track Power] Setting global emergency off");
      track->disable_output_for_reason(DccOutput::DisableReason::GLOBAL_EOFF);
    }
  }

  openlcb::Node *node()
  {
    return node_;
  }

private:
  openlcb::Node *node_;
};

} // namespace esp32cs

#endif // TRACK_POWER_HANDLER_HXX_
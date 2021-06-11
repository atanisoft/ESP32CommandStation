/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

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

#include "Turnouts.h"

#include <EventBroadcastHelper.hxx>
#include <HttpStringUtils.h>
#include <StringUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>

namespace esp32cs
{

OpenLCBTurnout::OpenLCBTurnout(const uint16_t address,
                               std::string closed_events,
                               std::string thrown_events, TurnoutType type,
                               bool state) : TurnoutBase(address, state, type)
{
  LOG(INFO,
      "[OpenLCBTurnout %d] Registered as type %s and initial state of %s",
      address, TURNOUT_TYPE_STRINGS[type_], state_ ? "Thrown" : "Closed");
  update_events(closed_events, thrown_events);
}

void OpenLCBTurnout::update_events(std::string closed_events, std::string thrown_events)
{
  vector<string> closed;
  vector<string> thrown;
  http::tokenize(closed_events, closed, ",", true, true);
  http::tokenize(thrown_events, thrown, ",", true, true);
  closed_.clear();
  for (auto event : closed)
  {
    LOG(INFO, "[OpenLCBTurnout %d] Closed event: %s", address(),
        event.c_str());
    closed_.push_back(string_to_uint64(event));
  }
  thrown_.clear();
  for (auto event : thrown)
  {
    LOG(INFO, "[OpenLCBTurnout %d] Thrown event: %s", address(),
        event.c_str());
    thrown_.push_back(string_to_uint64(event));
  }
}

bool OpenLCBTurnout::set(bool thrown, bool send_event)
{
  TurnoutBase::set(thrown, send_event);
  if (send_event)
  {
    auto eventHelper = Singleton<esp32cs::EventBroadcastHelper>::instance();
    if (thrown)
    {
      for (auto event : thrown_)
      {
        eventHelper->send_event(event);
      }
    }
    else
    {
      for (auto event : closed_)
      {
        eventHelper->send_event(event);
      }
    }
  }
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[OpenLCBTurnout %d] Set to %s", address(),
      get() ? "Thrown" : "Closed");
  return false;
}

std::string OpenLCBTurnout::to_json(bool readableStrings)
{
  std::vector<string> closed_events;
  std::vector<string> thrown_events;
  for (auto event : closed_)
  {
    closed_events.push_back(uint64_to_string_hex(event));
  }
  for (auto event : thrown_)
  {
    thrown_events.push_back(uint64_to_string_hex(event));
  }
  string serialized =
    StringPrintf(R"!^!({"address":%d,"type":%d,"openlcb":{"closed":"%s","thrown":"%s"},"state":)!^!",
                 address(), type_,
                 http::string_join(closed_events, ",").c_str(),
                 http::string_join(thrown_events, ",").c_str());
  if (readableStrings)
  {
    serialized +=
      StringPrintf(R"!^!("%s")!^!", get() ? "Thrown" : "Closed");
  }
  else
  {
    serialized += integer_to_string(get());
  }
  serialized += "}";
  return serialized;
}

} // namespace esp32cs
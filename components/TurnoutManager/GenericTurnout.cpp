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

#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>
#include <EventBroadcastHelper.hxx>

namespace esp32cs
{

void TurnoutBase::update(uint16_t address, TurnoutType type)
{
  address_ = address;
  if (type != TurnoutType::NO_CHANGE)
  {
    type_ = type;
  }
  
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout %d] Updated type %s", address_,
      TURNOUT_TYPE_STRINGS[type_]);
}

Turnout::Turnout(uint16_t address, bool state, TurnoutType type)
               : TurnoutBase(address, state, type)
{
  LOG(INFO, "[Turnout %d] Registered as type %s and initial state of %s",
      address_, TURNOUT_TYPE_STRINGS[type_], state_ ? "Thrown" : "Closed");
}

string Turnout::to_json(bool readableStrings)
{
  string serialized =
    StringPrintf("{\"address\":%d,\"type\":%d,\"state\":",
                 address(), type());
  if (readableStrings)
  {
    serialized += StringPrintf("\"%s\"", get() ? "Thrown" : "Closed");
  }
  else
  {
    serialized += integer_to_string(get());
  }
  serialized += "}";
  return serialized;
}

bool Turnout::set(bool thrown, bool send_event)
{
  TurnoutBase::set(thrown, send_event);
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout %d] Set to %s", address(),
      get() ? "Thrown" : "Closed");
  return send_event;
}

} // namespace esp32cs
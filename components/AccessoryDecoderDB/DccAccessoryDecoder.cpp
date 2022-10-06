/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "DccAccessoryDecoder.hxx"

#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>
#include <EventBroadcastHelper.hxx>

namespace esp32cs
{

DccAccessoryDecoder::DccAccessoryDecoder(
  uint16_t address, std::string name, bool state, AccessoryType type) :
  AccessoryBaseType(address, name, state, type)
{
  LOG(CONFIG_TURNOUT_LOG_LEVEL,
      "[DccAccessoryDecoder %d] Registered as %s with state of %s",
      address, ACCESSORY_TYPE_STRINGS[type], state ? "Thrown" : "Closed");
}

string DccAccessoryDecoder::to_json(bool readableStrings)
{
  string serialized =
    StringPrintf("{\"addr\":%d,\"name\":\"%s\",\"type\":%d,\"state\":",
    address(), name().c_str(), type());
  if (readableStrings)
  {
    if (get())
    {
      serialized += "\"Thrown\"";
    }
    else
    {
      serialized += "\"Closed\"";
    }
  }
  else
  {
    serialized += integer_to_string(get());
  }
  serialized += "}";
  return serialized;
}

bool DccAccessoryDecoder::set(bool state, bool is_on)
{
  AccessoryBaseType::set(state, is_on);
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[DccAccessoryDecoder %d] Set to %s (%s)",
      address(), get() ? "Thrown" : "Closed", is_on ? "On" : "Off");
  return true;
}

} // namespace esp32cs
/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "OpenLCBAccessoryDecoder.hxx"

#include <EventBroadcastHelper.hxx>
#include <HttpStringUtils.h>
#include <utils/StringUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>

namespace esp32cs
{

OpenLCBAccessoryDecoder::OpenLCBAccessoryDecoder(
  const uint16_t address, string name, string closed_events,
  string thrown_events, AccessoryType type, bool state) :
  AccessoryBaseType(address, name, state, type)
{
  LOG(CONFIG_TURNOUT_LOG_LEVEL,
      "[OpenLCBAccessoryDecoder %d] Registered as %s with state of %s",
      address, ACCESSORY_TYPE_STRINGS[type], state ? "Thrown" : "Closed");
  update_events(closed_events, thrown_events);
}

void OpenLCBAccessoryDecoder::update_events(string closed_events,
                                            string thrown_events)
{
  vector<string> closed;
  vector<string> thrown;
  http::tokenize(closed_events, closed, ",", true, true);
  http::tokenize(thrown_events, thrown, ",", true, true);
  closed_.clear();
  for (auto event : closed)
  {
    uint64_t evt = utils::string_to_uint64(event);
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[OpenLCBAccessoryDecoder %d] Closed event: %s", address(),
        utils::event_id_to_string(evt).c_str());
    closed_.push_back(evt);
  }
  thrown_.clear();
  for (auto event : thrown)
  {
    uint64_t evt = utils::string_to_uint64(event);
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[OpenLCBAccessoryDecoder %d] Thrown event: %s", address(),
        utils::event_id_to_string(evt).c_str());
    thrown_.push_back(evt);
  }
}

bool OpenLCBAccessoryDecoder::set(bool state, bool is_on)
{
  AccessoryBaseType::set(state, is_on);
  auto eventHelper = Singleton<esp32cs::EventBroadcastHelper>::instance();
  if (get())
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
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[OpenLCBAccessoryDecoder %d] Set to %s",
      address(), get() ? "Thrown" : "Closed");
  return false;
}

std::string OpenLCBAccessoryDecoder::to_json(bool readableStrings)
{
  vector<string> closed_events;
  vector<string> thrown_events;
  for (auto event : closed_)
  {
    closed_events.push_back(uint64_to_string_hex(event));
  }
  for (auto event : thrown_)
  {
    thrown_events.push_back(uint64_to_string_hex(event));
  }
  string serialized =
    StringPrintf(R"!^!({"addr":%d,"name":"%s","type":%d,"olcb":{"closed":"%s","thrown":"%s"},"state":)!^!",
                 address(), name().c_str(), type(),
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
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

#include "AccessoryDecoderDatabase.hxx"
#include "DccAccessoryDecoder.hxx"
#include "OpenLCBAccessoryDecoder.hxx"

#include <algorithm>
#include <cJSON.h>
#include <dcc/DccDebug.hxx>
#include <dcc/UpdateLoop.hxx>
#include <HttpStringUtils.h>
#include <openlcb/TractionDefs.hxx>
#include <utils/FileUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>

namespace esp32cs
{

using openlcb::Defs;
using openlcb::EncodeRange;
using openlcb::eventid_to_buffer;
using openlcb::EventRegistry;
using openlcb::EventRegistryEntry;
using openlcb::EventReport;
using openlcb::EventState;
using openlcb::invert_event_state;
using openlcb::TractionDefs;
using openlcb::WriteHelper;

DECLARE_CONST(dcc_accessory_packet_repeats);

static constexpr const char * ACCESSORIES_JSON_FILE = "/fs/decoders.json";
static constexpr uint64_t DB_PERSIST_INTERVAL = 
  SEC_TO_NSEC(CONFIG_TURNOUT_PERSISTENCE_INTERVAL_SEC);

#define FIND_ACCESSORY(address)                           \
  std::find_if(accessories_.begin(), accessories_.end(),  \
    [address](auto & decoder) -> bool                     \
    {                                                     \
      return (decoder->address() == address);             \
    }                                                     \
  )

AccessoryDecoderDB::AccessoryDecoderDB(openlcb::Node *node, Service *service)
  : node_(node),
    persistFlow_(service, DB_PERSIST_INTERVAL,
                 std::bind(&AccessoryDecoderDB::persist, this)),
    dirty_(false)
{
  LOG(INFO, "[AccessoryDecoderDB] Initializing");
  struct stat statbuf;
  if (!stat(ACCESSORIES_JSON_FILE, &statbuf))
  {
    LOG(INFO, "[AccessoryDecoderDB] Loading %s", ACCESSORIES_JSON_FILE);
    auto turnout_data = read_file_to_string(ACCESSORIES_JSON_FILE);
    cJSON *root =
      cJSON_ParseWithLength(turnout_data.c_str(), turnout_data.size());
    if (cJSON_IsArray(root))
    {
      cJSON *accessory;
      cJSON_ArrayForEach(accessory, root)
      {
        if (cJSON_HasObjectItem(accessory, "address") &&
            cJSON_HasObjectItem(accessory, "state") && 
            cJSON_HasObjectItem(accessory, "type"))
        {
          uint16_t address = cJSON_GetObjectItem(accessory, "address")->valueint;
          if (address < 1 || address > 2044)
          {
            continue;
          }
          bool state = cJSON_IsTrue(cJSON_GetObjectItem(accessory, "state"));
          AccessoryType type =
            (AccessoryType)cJSON_GetObjectItem(accessory, "type")->valueint;
          if (cJSON_HasObjectItem(accessory, "olcb"))
          {
            cJSON *events = cJSON_GetObjectItem(accessory, "olcb");
            auto closed_events =
              cJSON_GetObjectItem(events, "closed")->valuestring;
            auto thrown_events =
              cJSON_GetObjectItem(events, "thrown")->valuestring;
            LOG(CONFIG_TURNOUT_LOG_LEVEL,
                "[AccessoryDecoderDB %d] OpenLCB closed:%s, thrown:%s",
                address, closed_events, thrown_events);
            accessories_.push_back(
              std::make_unique<OpenLCBAccessoryDecoder>(address, closed_events,
                                                        thrown_events, type, state));
          }
          else
          {
            LOG(CONFIG_TURNOUT_LOG_LEVEL, "[AccessoryDecoderDB %d] DCC", address);
            accessories_.push_back(
              std::make_unique<DccAccessoryDecoder>(address, state, type));
          }
        }
        else
        {
          LOG_ERROR("[AccessoryDecoderDB] Entry is missing required data:\n%s",
                    cJSON_Print(accessory));
        }
      }
    }
    else if (cJSON_IsInvalid(root))
    {
      LOG_ERROR("[AccessoryDecoderDB] Persistent storage is corrupt and will "
                "not be loaded!");
    }
    cJSON_Delete(root);
  }
  else
  {
    LOG(WARNING, "[AccessoryDecoderDB] %s does not exist, skipping loading.",
        ACCESSORIES_JSON_FILE);
  }
  LOG(INFO, "[AccessoryDecoderDB] Loaded %d accessory decoder(s)",
      accessories_.size());
  packet_processor_add_refresh_source(this);

  EventRegistry::instance()->register_handler(
      EventRegistryEntry(
          this, TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE),
      12);
  EventRegistry::instance()->register_handler(
      EventRegistryEntry(
          this, TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE),
      12);
}

AccessoryDecoderDB::~AccessoryDecoderDB()
{
  EventRegistry::instance()->unregister_handler(this);
}

void AccessoryDecoderDB::handle_identify_global(const EventRegistryEntry &entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
  AutoNotify an(done);
  if (event->dst_node && event->dst_node != node_)
  {
    return;
  }
  event->event_write_helper<1>()->WriteAsync(node_,
    Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
    eventid_to_buffer(EncodeRange(
      TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE, 2044)),
    done->new_child());
  event->event_write_helper<2>()->WriteAsync(node_,
    Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
    eventid_to_buffer(EncodeRange(
      TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE, 2044)),
    done->new_child());
}

void AccessoryDecoderDB::handle_event_report(const EventRegistryEntry &entry,
                                             EventReport *event,
                                             BarrierNotifiable *done)
{
  AutoNotify an(done);
  if (event->event >= TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
      event->event < TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
  {
    // accessory decoder index, this is the same as the dcc address with the
    // decoder state bit added.
    uint16_t index =
      event->event - TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
    // drop the lowest bit to get the decoder address without the state bit
    uint16_t address = index >> 1;
    set(address, index & 0x01, true);
  }
  else if (event->event >= TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
           event->event < TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
  {
    // accessory decoder index, this is the same as the dcc address with the
    // decoder state bit added.
    uint16_t index =
      event->event - TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
    // drop the lowest bit to get the decoder address without the state bit
    uint16_t address = index >> 1;
    set(address, index & 0x01, false);
  }
}

void AccessoryDecoderDB::handle_identify_consumer(const EventRegistryEntry &entry,
                                                  EventReport *event,
                                                  BarrierNotifiable *done)
{
  AutoNotify an(done);
  EventState s = EventState::UNKNOWN;
  if (event->event >= TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
      event->event < TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
  {
    // accessory decoder index, this is the same as the dcc address with the
    // decoder state bit added.
    uint16_t index =
      event->event - TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
    // drop the lowest bit to get the decoder address without the state bit
    uint16_t address = index >> 1;
    auto turnout = get(address, true);
    if (turnout)
    {
      if (turnout->get())
      {
        s = EventState::VALID;
      }
      else
      {
        s = EventState::INVALID;
      }
      if (index & 0x1)
      {
        // query was reversed. invert the event state
        s = invert_event_state(s);
      }
    }
  }
  else if (event->event >= TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
          event->event < TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
  {
    // accessory decoder index, this is the same as the dcc address with the
    // decoder state bit added.
    uint16_t index =
      event->event - TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
    // drop the lowest bit to get the decoder address without the state bit
    uint16_t address = index >> 1;
    auto turnout = get(address, true);
    if (turnout)
    {
      if (turnout->get())
      {
        s = EventState::VALID;
      }
      else
      {
        s = EventState::INVALID;
      }
      if (index & 0x1)
      {
        // query was reversed. invert the event state
        s = invert_event_state(s);
      }
    }
  }
  else
  {
    return;
  }
  Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID + s;
  event->event_write_helper<1>()->WriteAsync(node_, mti,
      WriteHelper::global(), eventid_to_buffer(event->event),
      done->new_child());
}

void AccessoryDecoderDB::clear()
{
  SpinlockHolder lock(&lock_);
  for (auto & accessory : accessories_)
  {
    accessory.reset(nullptr);
  }
  accessories_.clear();
  dirty_ = true;
}

void AccessoryDecoderDB::set(uint16_t address, bool thrown, bool on_off)
{
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    if ((*elem)->set(thrown, on_off))
    {
      packet_processor_notify_update(this, (*elem)->address());
    }
    dirty_ = true;
  }
#if CONFIG_TURNOUT_CREATE_ON_DEMAND
  // we didn't find it, create it and set it
  accessories_.push_back(
    std::make_unique<DccAccessoryDecoder>(address, address));
  if (accessories_.back()->set(thrown, on_off))
  {
    packet_processor_notify_update(this, accessories_.back()->address());
  }
#endif // CONFIG_TURNOUT_CREATE_ON_DEMAND
}

bool AccessoryDecoderDB::toggle(uint16_t address)
{
  LOG(CONFIG_TURNOUT_LOG_LEVEL
    , "[AccessoryDecoderDB] Request to toggle turnout address %d", address);
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[AccessoryDecoderDB] Turnout found, toggling");
    if((*elem)->toggle())
    {
      packet_processor_notify_update(this, (*elem)->address());
    }
    dirty_ = true;
    return (*elem).get();
  }

#if CONFIG_TURNOUT_CREATE_ON_DEMAND
  LOG(CONFIG_TURNOUT_LOG_LEVEL,
      "[AccessoryDecoderDB] Turnout not found, creating and toggling");

  // we didn't find it, create it and throw it
  accessories_.push_back(std::make_unique<DccAccessoryDecoder>(address));
  if(accessories_.back()->toggle())
  {
    packet_processor_notify_update(this, accessories_.back()->address());
  }
  return accessories_.back().get();
#else
  return false;
#endif // CONFIG_TURNOUT_CREATE_ON_DEMAND
}

string AccessoryDecoderDB::to_json(bool readable)
{
  SpinlockHolder lock(&lock_);
  return to_json_locked(readable);
}

std::string AccessoryDecoderDB::to_json(const uint16_t address, bool readable)
{
  auto turnout = get(address);
  if (turnout)
  {
    return turnout->to_json(readable);
  }
  return "{}";
}

void AccessoryDecoderDB::createOrUpdateDcc(const uint16_t address,
                                           const AccessoryType type)
{
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[AccessoryDecoderDB %d] Updated existing DCC decoder",
        address);
    elem->get()->update(address, type);
  }
  else
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[AccessoryDecoderDB %d] Created new DCC decoder", address);
    // we didn't find it, create it!
    accessories_.push_back(
      std::make_unique<DccAccessoryDecoder>(
        address, false,
        type != AccessoryType::UNCHANGED ? type : AccessoryType::UNKNOWN));
  }
  dirty_ = true;
}

void AccessoryDecoderDB::createOrUpdateOlcb(const uint16_t address,
                                            std::string closed_events,
                                            std::string thrown_events,
                                            const AccessoryType type)
{
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[AccessoryDecoderDB %d] Updated existing OpenLCB virtual decoder",
        address);
    elem->get()->update(address, type);
    static_cast<OpenLCBAccessoryDecoder *>(elem->get())->update_events(closed_events
                                                                     , thrown_events);
  }
  else
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[AccessoryDecoderDB %d] Created OpenLCB virtual decoder", address);
    // we didn't find it, create it!
    accessories_.push_back(
      std::make_unique<OpenLCBAccessoryDecoder>(address, closed_events,
                                                thrown_events, type, false));
  }
  dirty_ = true;
}

bool AccessoryDecoderDB::remove(const uint16_t address)
{
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    LOG(INFO, "[AccessoryDecoderDB %d] Deleted", address);
    accessories_.erase(elem);
    dirty_ = true;
    return true;
  }
  LOG(WARNING, "[AccessoryDecoderDB %d] not found", address);
  return false;
}

uint16_t AccessoryDecoderDB::count()
{
  return accessories_.size();
}

void AccessoryDecoderDB::get_next_packet(unsigned code, dcc::Packet* packet)
{
  // If the code is zero it is a general update packet which we do not use for
  // turnouts, default it to an idle packet instead.
  if (!code)
  {
    packet->set_dcc_idle();
    return;
  }

  // retrieve the turnout based on the provided code (address).
  auto turnout = get(code);

  // shift address by one to account for the output pair state bit (thrown).
  // decrement the address prior to shift to bring it into the 0-2047 range.
  uint16_t addr = (((turnout->address() - 1) << 1) | turnout->get());

  packet->add_dcc_basic_accessory(addr, turnout->is_on());
  packet->packet_header.rept_count = config_dcc_accessory_packet_repeats();

  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[TurnoutDB] Sending packet: %s",
      packet_to_string(*packet, true).c_str());
}

AccessoryBaseType *AccessoryDecoderDB::get(const uint16_t address, bool silent)
{
  SpinlockHolder lock(&lock_);
  auto const &elem = FIND_ACCESSORY(address);
  if (elem != accessories_.end())
  {
    return elem->get();
  }
  if (!silent)
  {
    LOG(WARNING, "[TurnoutDB] Address %d not found", address);
  }
  return nullptr;
}

string AccessoryDecoderDB::to_json_locked(bool readableStrings)
{
  string content = "[";
  for (const auto& turnout : accessories_)
  {
    // only add the seperator if we have already serialized at least one
    // turnout.
    if (content.length() > 1)
    {
      content += ",";
    }
    content += turnout->to_json(readableStrings);
  }
  content += "]";
  return content;
}

void AccessoryDecoderDB::persist()
{
  SpinlockHolder lock(&lock_);
  bool dirtyFlag = dirty_;
  dirty_ = false;
  // Check if we have any changes to persist, if not exit early.
  if (!dirtyFlag || accessories_.empty())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL,
        "[TurnoutDB] No entries require persistence.");
    return;
  }
  LOG(INFO, "[TurnoutDB] Persisting %zu turnouts", accessories_.size());
  write_string_to_file(ACCESSORIES_JSON_FILE, to_json_locked(false));
}

} // namespace esp32cs

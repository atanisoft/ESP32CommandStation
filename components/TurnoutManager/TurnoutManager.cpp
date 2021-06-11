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

#include <algorithm>
#include <cJSON.h>
#include <dcc/DccDebug.hxx>
#include <dcc/UpdateLoop.hxx>
#include <HttpStringUtils.h>
#include <utils/FileUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/StringPrintf.hxx>

namespace esp32cs
{

DECLARE_CONST(dcc_turnout_packet_repeats);

static constexpr const char * TURNOUTS_JSON_FILE = "turnouts.json";

TurnoutManager::TurnoutManager(openlcb::Node *node, Service *service)
  : turnoutEventConsumer_(node, this),
    persistFlow_(service, SEC_TO_NSEC(CONFIG_TURNOUT_PERSISTENCE_INTERVAL_SEC),
                 std::bind(&TurnoutManager::persist, this)),
    dirty_(false)
{
  const std::lock_guard<std::mutex> lock(mux_);
  LOG(INFO, "[TurnoutManager] Initializing Turnout database");
  struct stat statbuf;
  if (!stat(TURNOUTS_JSON_FILE, &statbuf))
  {
    auto turnout_data = read_file_to_string(TURNOUTS_JSON_FILE);
    cJSON *root = cJSON_Parse(turnout_data.c_str());
    if (cJSON_IsArray(root))
    {
      cJSON *turnout;
      cJSON_ArrayForEach(turnout, root)
      {
        uint16_t address = cJSON_GetObjectItem(turnout, "address")->valueint;
        if (address < 1 || address > 2044)
        {
          continue;
        }
        bool state = cJSON_IsTrue(cJSON_GetObjectItem(turnout, "state"));
        TurnoutType type = (TurnoutType)cJSON_GetObjectItem(turnout, "type")->valueint;
        if (cJSON_HasObjectItem(turnout, "openlcb"))
        {
          cJSON *events = cJSON_GetObjectItem(turnout, "openlcb");
          turnouts_.push_back(
            std::make_unique<OpenLCBTurnout>(address,
                                             cJSON_GetObjectItem(events, "closed")->valuestring,
                                             cJSON_GetObjectItem(events, "thrown")->valuestring,
                                             type, state));
        }
        else
        {
          turnouts_.push_back(std::make_unique<Turnout>(address, state, type));
        }
      }
    }
    cJSON_Delete(root);
  }
  LOG(INFO, "[TurnoutManager] Loaded %d turnout(s)", turnouts_.size());
  packet_processor_add_refresh_source(this);
}

void TurnoutManager::clear()
{
  const std::lock_guard<std::mutex> lock(mux_);
  for (auto & turnout : turnouts_)
  {
    turnout.reset(nullptr);
  }
  turnouts_.clear();
  dirty_ = true;
}

#define FIND_TURNOUT(address)                             \
  std::find_if(turnouts_.begin(), turnouts_.end(),        \
    [address](auto & turnout) -> bool                     \
    {                                                     \
      return (turnout->address() == address);             \
    }                                                     \
  )

void TurnoutManager::set(uint16_t address, bool thrown, bool send_event)
{
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    if ((*elem)->set(thrown, send_event))
    {
      packet_processor_notify_update(this, (*elem)->address());
    }
    dirty_ = true;
  }
#if CONFIG_TURNOUT_CREATE_ON_DEMAND
  // we didn't find it, create it and set it
  turnouts_.push_back(std::make_unique<Turnout>(address, address));
  if (turnouts_.back()->set(thrown, send_event))
  {
    packet_processor_notify_update(this, turnouts_.back()->address());
  }
#endif // CONFIG_TURNOUT_CREATE_ON_DEMAND
}

bool TurnoutManager::toggle(uint16_t address)
{
  LOG(CONFIG_TURNOUT_LOG_LEVEL
    , "[TurnoutManager] Request to toggle turnout address %d", address);
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL, "[TurnoutManager] Turnout found, toggling");
    if((*elem)->toggle())
    {
      packet_processor_notify_update(this, (*elem)->address());
    }
    dirty_ = true;
    return (*elem).get();
  }

#if CONFIG_TURNOUT_CREATE_ON_DEMAND
  LOG(CONFIG_TURNOUT_LOG_LEVEL
    , "[TurnoutManager] Turnout not found, creating and toggling");

  // we didn't find it, create it and throw it
  turnouts_.push_back(std::make_unique<Turnout>(address));
  if(turnouts_.back()->toggle())
  {
    packet_processor_notify_update(this, turnouts_.back()->address());
  }
  return turnouts_.back().get();
#else
  return false;
#endif // CONFIG_TURNOUT_CREATE_ON_DEMAND
}

string TurnoutManager::to_json(bool readable)
{
  const std::lock_guard<std::mutex> lock(mux_);
  return to_json_locked(readable);
}

std::string TurnoutManager::to_json(const uint16_t address, bool readable)
{
  auto turnout = get(address);
  if (turnout)
  {
    return turnout->to_json(readable);
  }
  return "{}";
}

void TurnoutManager::createOrUpdateDcc(const uint16_t address,
                                       const TurnoutType type)
{
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    elem->get()->update(address, type);
    dirty_ = true;
  }
  else
  {
    // we didn't find it, create it!
    turnouts_.push_back(
      std::make_unique<Turnout>(address, false
                              , type != TurnoutType::NO_CHANGE ? type
                                                              : TurnoutType::LEFT));
    dirty_ = true;
  }
  dirty_ = true;
}

void TurnoutManager::createOrUpdateOlcb(const uint16_t address,
                                        std::string closed_events,
                                        std::string thrown_events,
                                        const TurnoutType type)
{
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    elem->get()->update(address, type);
    static_cast<OpenLCBTurnout *>(elem->get())->update_events(closed_events
                                                            , thrown_events);
  }
  else
  {
    // we didn't find it, create it!
    turnouts_.push_back(
      std::make_unique<OpenLCBTurnout>(address, closed_events, thrown_events
                                    , type, false));
  }
  dirty_ = true;
}

bool TurnoutManager::remove(const uint16_t address)
{
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    LOG(INFO, "[Turnout %d] Deleted", address);
    turnouts_.erase(elem);
    dirty_ = true;
    return true;
  }
  LOG(WARNING, "[Turnout %d] not found", address);
  return false;
}

uint16_t TurnoutManager::count()
{
  return turnouts_.size();
}

// TODO: shift this to consume the LCC event directly
void TurnoutManager::send(Buffer<dcc::Packet> *b, unsigned prio)
{
  dcc::Packet *pkt = b->data();
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "TurnoutManager::send(%s)"
    , dcc::packet_to_string(*pkt).c_str());
  // Verify that the packet looks like a DCC Accessory decoder packet
  if(!pkt->packet_header.is_marklin &&
      pkt->payload[0] & 0x80 && pkt->payload[1] & 0x80)
  {
    // packet data format:
    // payload[0]  payload[1]
    // 10aaaaaa    1AAACDDD
    // ^ ^^^^^^    ^^^^^^^^ 
    // | |         ||  || |
    // | |         ||  || \-state bit
    // | |         ||  |\-output index
    // | |         ||  \-activate/deactivate output flag (ignored)
    // | |         |\-board address most significant three bits
    // | |         |  stored in 1s complement (1=0, 0=1)
    // | |         \-accessory packet flag
    // | \-board address (least significant six bits)
    // \-accessory packet flag
    // converting back to a single address using the following: AAAaaaaaaDDD
    // note that only the output index is used in calculation of the final
    // address since only the base address is stored in the CS.
    uint16_t boardAddress = ((~pkt->payload[1] & 0b01110000) << 2) |
                            (pkt->payload[0] & 0b00111111);
    uint8_t boardIndex = (pkt->payload[1] & 0b00000110) >> 1;
    // least significant bit of the second byte is thrown/closed indicator.
    bool state = pkt->payload[1] & 0b00000001;
    uint16_t address = decodeDCCAccessoryAddress(boardAddress, boardIndex) - 4;
    LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout %d %d:%d] Setting to %s", address
      , boardAddress, boardIndex
      , state ? "Thrown" : "Closed");
    // Set the turnout to the requested state, don't send a DCC packet.
    set(address, state);
  }
  b->unref();
}

void TurnoutManager::get_next_packet(unsigned code, dcc::Packet* packet)
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

  // always send activate as true (sets C to 1)
  packet->add_dcc_basic_accessory(addr, true);
  packet->packet_header.rept_count = config_dcc_turnout_packet_repeats();

  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[TurnoutManager] Sending packet: %s"
    , packet_to_string(*packet, true).c_str());
}

TurnoutBase *TurnoutManager::get(const uint16_t address)
{
  const std::lock_guard<std::mutex> lock(mux_);
  auto const &elem = FIND_TURNOUT(address);
  if (elem != turnouts_.end())
  {
    return elem->get();
  }
  LOG(WARNING, "[Turnout] Address %d not found", address);
  return nullptr;
}

string TurnoutManager::to_json_locked(bool readableStrings)
{
  string content = "[";
  for (const auto& turnout : turnouts_)
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

void TurnoutManager::persist()
{
  const std::lock_guard<std::mutex> lock(mux_);
  bool dirtyFlag = dirty_;
  dirty_ = false;
  // Check if we have any changes to persist, if not exit early.
  if (!dirtyFlag || turnouts_.empty())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout] No entries require persistence.");
    return;
  }
  LOG(INFO, "[Turnout] Persisting %zu turnouts", turnouts_.size());
  write_string_to_file(TURNOUTS_JSON_FILE, to_json_locked(false));
}

void encodeDCCAccessoryAddress(uint16_t *board, int8_t *port
                             , uint16_t address)
{
  // NOTE: This will convert from a USER visible DCC address to the on-the-wire
  // DCC board:port address format.
  *board = ((address - 1) / 4);
  *port = (address - 1) % 4;
}

uint16_t decodeDCCAccessoryAddress(uint16_t board, int8_t port)
{
  // NOTE: this decodes the board:port into the USER visible DCC address and
  // not the DCC on-the-wire address.
  uint32_t addr = (board << 2) + (port + 1);
  return (uint16_t)(addr & 0xFFFF);
}

} // namespace esp32cs
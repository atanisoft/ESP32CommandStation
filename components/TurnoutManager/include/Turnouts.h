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

#ifndef TURNOUTS_H_
#define TURNOUTS_H_

#include <AutoPersistCallbackFlow.h>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/PacketSource.hxx>
#include <mutex>
#include <openlcb/DccAccyConsumer.hxx>
#include <utils/Singleton.hxx>

namespace esp32cs
{

enum TurnoutType
{
  LEFT=0,
  RIGHT,
  WYE,
  MULTI,
  NO_CHANGE,
  MAX_TURNOUT_TYPES // NOTE: this must be the last entry in the enum.
};

static constexpr const char *TURNOUT_TYPE_STRINGS[] =
{
  "LEFT",
  "RIGHT",
  "WYE",
  "MULTI",
  "UNKNOWN"
};

void encodeDCCAccessoryAddress(uint16_t *board, int8_t *port, uint16_t address);
uint16_t decodeDCCAccessoryAddress(uint16_t board, int8_t port);

class TurnoutBase
{
public:
  uint16_t address()
  {
    return address_;
  }
  bool toggle()
  {
    return set(!get());
  }
  bool get()
  {
    return state_;
  }
  TurnoutType type()
  {
    return type_;
  }
  void type(TurnoutType type)
  {
    type_ = type;
  }
  virtual bool set(bool state, bool send_event = true)
  {
    state_ = state;
    return false;
  }
  virtual std::string to_json(bool readable_strings = false)
  {
    return "{}";
  }
  void update(uint16_t address, TurnoutType type);

protected:
  TurnoutBase(uint16_t address, bool state, TurnoutType type = TurnoutType::LEFT)
            : address_(address), state_(state), type_(type)
  {
  }

  uint16_t address_;
  bool state_;
  TurnoutType type_;
};

class Turnout : public TurnoutBase
{
public:
  Turnout(uint16_t address, bool thrown = false,
          TurnoutType type = TurnoutType::LEFT);
  bool set(bool state, bool send_event = true) override;
  std::string to_json(bool readable_strings = false) override;
};

class OpenLCBTurnout : public TurnoutBase
{
public:
  OpenLCBTurnout(const uint16_t address, std::string closed_events,
                 std::string thrown_events, TurnoutType type, bool state);
  bool set(bool state, bool send_event = true) override;
  std::string to_json(bool readable_strings = false) override;
  void update_events(std::string closed_events, std::string thrown_events);
private:
  std::vector<openlcb::EventId> closed_;
  std::vector<openlcb::EventId> thrown_;
};

class TurnoutManager : public dcc::PacketFlowInterface,
                       public Singleton<TurnoutManager>,
                       public dcc::NonTrainPacketSource
{
public:
  TurnoutManager(openlcb::Node *node, Service *service);
  void stop()
  {
    persistFlow_.stop();
  }
  void clear();
  void set(uint16_t address, bool thrown = false, bool event = true);
  bool toggle(uint16_t address);
  std::string to_json(bool readable = true);
  std::string to_json(const uint16_t address, bool readable = true);
  void createOrUpdateDcc(const uint16_t address,
                         const TurnoutType type = TurnoutType::LEFT);
  void createOrUpdateOlcb(const uint16_t address,
                          std::string closed_events,
                          std::string thrown_events,
                          const TurnoutType = TurnoutType::LEFT);
  bool remove(const uint16_t address);
  uint16_t count();
  void send(Buffer<dcc::Packet> *, unsigned);
  void get_next_packet(unsigned code, dcc::Packet* packet) override;
private:
  TurnoutBase *get(const uint16_t address);
  std::string to_json_locked(bool);
  void persist();
  std::vector<std::unique_ptr<TurnoutBase>> turnouts_;
  openlcb::DccAccyConsumer turnoutEventConsumer_;
  AutoPersistFlow persistFlow_;
  bool dirty_;
  std::mutex mux_;
};

} // namespace esp32cs

#endif // TURNOUTS_H_
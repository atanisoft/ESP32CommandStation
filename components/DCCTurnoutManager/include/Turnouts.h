/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

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
#include <DCCppProtocol.h>
#include <mutex>
#include <openlcb/DccAccyConsumer.hxx>
#include <utils/Singleton.hxx>

enum TurnoutType
{
  LEFT=0,
  RIGHT,
  WYE,
  MULTI,
  NO_CHANGE,
  MAX_TURNOUT_TYPES // NOTE: this must be the last entry in the enum.
};

void encodeDCCAccessoryAddress(uint16_t *board, int8_t *port, uint16_t address);
uint16_t decodeDCCAccessoryAddress(uint16_t board, int8_t port);

class Turnout : public dcc::NonTrainPacketSource
{
public:
  Turnout(uint16_t address, int16_t id = -1, bool thrown = false
        , TurnoutType type = TurnoutType::LEFT);
  virtual ~Turnout() {}
  void update(uint16_t address, TurnoutType type, int16_t id = -1);
  void set(bool thrown = false, bool sendDCCPacket = true);
  std::string toJson(bool readableStrings = false);
  uint16_t getAddress()
  {
    return _address;
  }
  uint16_t getID()
  {
    return _id;
  }
  bool isThrown()
  {
    return _thrown;
  }
  void toggle()
  {
    set(!_thrown);
  }
  TurnoutType getType()
  {
    return _type;
  }
  void setType(const TurnoutType type)
  {
    _type = type;
  }
  void get_next_packet(unsigned code, dcc::Packet* packet) override;
private:
  uint16_t _address;
  uint16_t _id;
  bool _thrown;
  TurnoutType _type;
};

class TurnoutManager : public dcc::PacketFlowInterface
                     , public Singleton<TurnoutManager>
{
public:
  TurnoutManager(openlcb::Node *, Service *);
  void stop()
  {
    persistFlow_.stop();
  }
  void clear();
  std::string set(uint16_t, bool=false, bool=true);
  std::string toggle(uint16_t);
  std::string getStateAsJson(bool=true);
  std::string get_state_for_dccpp();
  Turnout *createOrUpdate(const uint16_t address
                        , const TurnoutType = TurnoutType::LEFT
                        , const int16_t id = -1);
  bool remove(const uint16_t);
  Turnout *getByID(const uint16_t id);
  Turnout *get(const uint16_t);
  uint16_t count();
  void send(Buffer<dcc::Packet> *, unsigned);
private:
  std::string get_state_as_json(bool);
  void persist();
  std::vector<std::unique_ptr<Turnout>> turnouts_;
  openlcb::DccAccyConsumer turnoutEventConsumer_;
  AutoPersistFlow persistFlow_;
  bool dirty_;
  std::mutex mux_;
};

#endif // TURNOUTS_H_
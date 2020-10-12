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

#ifndef ESTOP_HANDLER_H_
#define ESTOP_HANDLER_H_

#include <dcc/PacketSource.hxx>
#include <dcc/SimpleUpdateLoop.hxx>
#include <mutex>
#include <openlcb/Defs.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <utils/logging.h>

namespace esp32cs
{

// Event handler for the E-Stop well known events. This will generate a
// continuous stream of e-stop DCC packets until the E-Stop event has been
// received or the state has been reset via API.
class EStopHandler : public openlcb::BitEventInterface
                   , public dcc::NonTrainPacketSource
{
public:
  EStopHandler(openlcb::Node *node)
    : BitEventInterface(openlcb::Defs::EMERGENCY_STOP_EVENT
                      , openlcb::Defs::CLEAR_EMERGENCY_STOP_EVENT)
    , node_(node)
  {
    LOG(INFO, "[eStop] Registering emergency stop handler (On: %s, Off:%s)"
      , uint64_to_string_hex(openlcb::Defs::EMERGENCY_STOP_EVENT).c_str()
      , uint64_to_string_hex(openlcb::Defs::CLEAR_EMERGENCY_STOP_EVENT).c_str());
  }

  openlcb::EventState get_current_state() override
  {
    return openlcb::EventState::INVALID;
  }

  void set_state(bool new_value) override;

  void get_next_packet(unsigned code, dcc::Packet* packet);

  openlcb::Node *node() override
  {
    return node_;
  }

private:
  openlcb::BitEventPC pc_{this};
  openlcb::Node *node_;
  std::mutex mux_;
};

} // namespace esp32cs

#endif // ESTOP_HANDLER_H_

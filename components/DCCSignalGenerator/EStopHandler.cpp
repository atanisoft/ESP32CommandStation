/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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

#include "EStopHandler.h"

#include <AllTrainNodes.hxx>

namespace esp32cs
{

void EStopHandler::set_state(bool new_value)
{
  if (new_value)
  {
    LOG(INFO, "[eStop] Received eStop request, sending eStop to all trains.");
    // TODO: add helper method on AllTrainNodes for this.
    auto  trains = Singleton<commandstation::AllTrainNodes>::instance();
    for (size_t id = 0; id < trains->size(); id++)
    {
      auto node = trains->get_train_node_id(id);
      if (node)
      {
        trains->get_train_impl(node)->set_emergencystop();
      }
    }
    packet_processor_add_refresh_source(this
                                      , dcc::UpdateLoopBase::ESTOP_PRIORITY);
  }
  else
  {
    packet_processor_remove_refresh_source(this);
  }
}

void EStopHandler::get_next_packet(unsigned code, dcc::Packet* packet)
{
  packet->set_dcc_speed14(dcc::DccShortAddress(0), true, false
                        , dcc::Packet::EMERGENCY_STOP);
}

}

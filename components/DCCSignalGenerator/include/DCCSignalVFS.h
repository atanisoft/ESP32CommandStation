/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#include "TrackOutputDescriptor.h"

#include <executor/Service.hxx>
#include <openlcb/Node.hxx>

namespace esp32cs
{

void init_dcc(openlcb::Node *node, Service *service
            , const esp32cs::TrackOutputConfig &ops_cfg
            , const esp32cs::TrackOutputConfig &prog_cfg);

void shutdown_dcc();

void toggle_estop();

bool is_ops_track_output_enabled();

void enable_ops_track_output();

void disable_track_outputs();

std::string get_track_state_json();

// retrive status of the track signal and current usage.
std::string get_track_state_for_dccpp();

} // namespace esp32cs
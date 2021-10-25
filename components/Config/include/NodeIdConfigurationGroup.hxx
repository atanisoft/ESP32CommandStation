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

#ifndef NODEID_CONFIG_GROUP_HXX_
#define NODEID_CONFIG_GROUP_HXX_

#include <openlcb/ConfigRepresentation.hxx>

#include "sdkconfig.h"

namespace esp32cs
{

CDI_GROUP(NodeIdConfig, Segment(CONFIG_OLCB_NODEID_MEMORY_SPACE_ID),
          Offset(CONFIG_OLCB_NODEID_MEMORY_SPACE_OFFSET));
CDI_GROUP_ENTRY(node_id, openlcb::StringConfigEntry<32>, Name("Node ID"),
                Description(
R"!^!(Identifier to use for this device.
NOTE: Changing this value will force a factory reset.)!^!"))
CDI_GROUP_END();

} // namespace esp32cs

#endif // NODEID_CONFIG_GROUP_HXX_

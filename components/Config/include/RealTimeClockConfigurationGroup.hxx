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

#ifndef REALTIME_CLOCK_CONFIG_GROUP_HXX_
#define REALTIME_CLOCK_CONFIG_GROUP_HXX_

#include <openlcb/ConfigRepresentation.hxx>

#include "sdkconfig.h"

namespace esp32cs
{

CDI_GROUP(RealTimeClockConfiguration,
          Segment(CONFIG_OLCB_REALTIMECLOCK_MEMORY_SPACE_ID));
CDI_GROUP_ENTRY(enabled, openlcb::Uint8ConfigEntry, Name("Enable Clock"),
                Description(
R"!^!(Enabling this option configures the Command Station to generate OpenLCB
FastClock events.)!^!"),
                Min(0), Max(1),
                MapValues(
R"!^!(<relation><property>0</property><value>Disabled</value></relation>"
                "<relation><property>1</property><value>Enabled</value></relation>)!^!"),
                Default(0));
CDI_GROUP_ENTRY(id, openlcb::StringConfigEntry<32>, Name("Clock ID"),
                Description(
R"!^!(The Clock ID is used as the upper six bytes of all BroadcastTime events
generated as part of the operation of the Fast Clock.
Common ID values:
01.01.00.00.01.00 - Default Fast Clock
01.01.00.00.01.01 - Default Real-time Clock
01.01.00.00.01.02 - Alternate Clock 1
01.01.00.00.01.03 - Alternate Clock 2)!^!"));
CDI_GROUP_END();

} // namespace esp32cs

#endif // REALTIME_CLOCK_CONFIG_GROUP_HXX_

/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

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

#ifndef TRACK_OUTPUT_DESCRIPTOR_H_
#define TRACK_OUTPUT_DESCRIPTOR_H_

#include <openlcb/ConfigRepresentation.hxx>

#include "hardware.hxx"

namespace esp32cs
{
  /// <map> of possible keys and descriptive values to show to the user for
  /// the enable fields below.
  static constexpr const char *DCC_BOOLEAN_MAP =
      "<relation><property>0</property><value>Disabled</value></relation>"
      "<relation><property>1</property><value>Enabled</value></relation>";
  /// DCC output behavior
  CDI_GROUP(AdvancedDCCConfig)
  CDI_GROUP_ENTRY(ops_preamble_bits, openlcb::Uint8ConfigEntry,
                  Name("Operations Track Preamble bit count"),
                  Min(11), Max(20), Default(16));
  CDI_GROUP_ENTRY(prog_preamble_bits, openlcb::Uint8ConfigEntry,
                  Name("Programming Track Preamble bit count"),
                  Min(22), Max(50), Default(22));
  CDI_GROUP_ENTRY(enable_railcom, openlcb::Uint8ConfigEntry,
                  Name("Enable RailCom cut out generation"),
                  Description(
R"!^!(Enabling this option configures the Command Station to generate the
RailCom cut-out.
NOTE: this applys to both the OPS track output and the OpenLCB output (if
enabled).)!^!"),
                  Min(0), Max(1),
                  Default(1), /* On */
                  MapValues(DCC_BOOLEAN_MAP));
  CDI_GROUP_ENTRY(enable_railcom_receiver, openlcb::Uint8ConfigEntry,
                  Name("Enable RailCom Receiver"),
                  Description(
R"!^!(Enabling this option configures the Command Station to interpret the
data received during the RailCom cut-out.
NOTE: this applys to both the OPS track output and the OpenLCB output (if
enabled).)!^!"),
                  Min(0), Max(1),
                  Default(1), /* On */
                  MapValues(DCC_BOOLEAN_MAP));
  CDI_GROUP_END();

  /// Track output configuration
  CDI_GROUP(TrackOutputConfig);
  CDI_GROUP_ENTRY(ops_current_limit, openlcb::Uint32ConfigEntry,
                  Name("Operations Track Current Limit (mA)"),
                  Min(0), Max(CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS),
                  Default(CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS));
  CDI_GROUP_ENTRY(event_short,
                  openlcb::EventConfigEntry,
                  Name("Short Detected"),
                  Description("This event will be produced when a short has "
                              "been detected on the track output."));
  CDI_GROUP_ENTRY(event_shutdown,
                  openlcb::EventConfigEntry,
                  Name("H-Bridge Shutdown"),
                  Description("This event will be produced when the track "
                              "output power has exceeded the safety threshold "
                              "of the H-Bridge."));
  CDI_GROUP_ENTRY(advanced, AdvancedDCCConfig,
                  Name("Advanced Configuration Settings"));
  CDI_GROUP_END();
} // namespace esp32cs

#endif // TRACK_OUTPUT_DESCRIPTOR_H_
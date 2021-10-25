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

#ifndef OPENLCB_CONFIG_GROUP_HXX_
#define OPENLCB_CONFIG_GROUP_HXX_

#include <openlcb/ConfigRepresentation.hxx>
#include <freertos_drivers/esp32/Esp32WiFiConfiguration.hxx>

#include "sdkconfig.h"

namespace esp32cs
{

static constexpr const char * const OLCB_BOOLEAN_OPTION_MAP =
R"!^!(<relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation>)!^!";

/// OpenLCB Configuration options
CDI_GROUP(AdvancedOpenLCBConfiguration);
/// Enables sending brownout events when detected.
CDI_GROUP_ENTRY(brownout_enable, openlcb::Uint8ConfigEntry,
                Name("Enable Brownout detection"),
                Description(
R"!^!(Enabling this option configures the Command Station to send brownout events when
the ESP32 restarts due to brownout.)!^!"),
                Min(0), Max(1), Default(1), /* On */
                MapValues(OLCB_BOOLEAN_OPTION_MAP));
/// Enables the DCC signal output to the OpenLCB RJ45 pins 4 and 5.
CDI_GROUP_ENTRY(dcc_enable, openlcb::Uint8ConfigEntry,
                Name("Enable DCC signal"),
                Description(
R"!^!(Enabling this option configures the Command Station to output the DCC signal and RailCom cut-out (if enabled) to the OpenLCB connection on pins 4 and 5.)!^!"),
                Min(0), Max(1), Default(1), /* On */
                MapValues(OLCB_BOOLEAN_OPTION_MAP));
/// Enables listening to the OpenLCB DCC Accessory event range.
CDI_GROUP_ENTRY(enable_accessory_bus, openlcb::Uint8ConfigEntry,
                Name("Enable DCC Accessory Support"),
                Description(
R"!^!(Enabling this option configures the Command Station to listen for any
DCC Accessory event and generate a compatible DCC Accessory packet or OpenLCB
events for virtual DCC accessories.)!^!"),
                Min(0), Max(1), Default(1), /* On */
                MapValues(OLCB_BOOLEAN_OPTION_MAP));
/// Enables listening to OpenLCB Throttle events.
CDI_GROUP_ENTRY(enable_throttles, openlcb::Uint8ConfigEntry,
                Name("Enable OpenLCB Throttle Support"),
                Description(
R"!^!(Enabling this option configures the Command Station to listen for OpenLCB
Throttle requests.)!^!"),
                Min(0), Max(1), Default(1), /* On */
                MapValues(OLCB_BOOLEAN_OPTION_MAP));
/// Configures how often OpenLCB throttles must send a heartbeat event before
/// any controlled trains will be stopped.
CDI_GROUP_ENTRY(throttle_heartbeat, openlcb::Uint8ConfigEntry,
                Name("Throttle Heartbeats"),
                Description(
R"!^!(Configures how often OpenLCB connected throttles must send a heartbeat
message to the Command Station. If a throttle does not send the heartbeat
within the configured limit, any trains controlled by the throttle will be
stopped. Set to zero to disable.)!^!"),
                Min(0), Max(240), Default(10));
CDI_GROUP_END();

/// OpenLCB Configuration options
CDI_GROUP(OpenLCBConfiguration);
/// OpenLCB WiFi Usage Configuration.
CDI_GROUP_ENTRY(wifi, openmrn_arduino::WiFiConfiguration, Name("WiFi Usage"));
/// Advanced configuration options.
CDI_GROUP_ENTRY(advanced, AdvancedOpenLCBConfiguration, Name("Advanced"));
CDI_GROUP_END();

} // namespace esp32cs

#endif // OPENLCB_CONFIG_GROUP_HXX_

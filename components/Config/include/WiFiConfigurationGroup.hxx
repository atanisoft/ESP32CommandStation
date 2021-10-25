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

#ifndef WIFI_CONFIGURATION_GROUP_HXX_
#define WIFI_CONFIGURATION_GROUP_HXX_

#include <openlcb/ConfigRepresentation.hxx>

#include "sdkconfig.h"

namespace esp32cs
{

CDI_GROUP(WiFiStationConfiguration);
/// Allows setting the SSID that the Station will attempt to connect to.
CDI_GROUP_ENTRY(ssid, openlcb::StringConfigEntry<32>,
    Name("SSID"),
    Description("Configures the SSID that the ESP32 will connect to."));
/// Allows setting the password that the Station will use for connecting to the
/// configured SSID. Note: This value will be prefixed with [b64] and be base64
/// encoded when read via CDI.
CDI_GROUP_ENTRY(password, openlcb::StringConfigEntry<128>,
    Name("Password"),
    Description("Configures the password that the ESP32 will use for the station SSID."));
CDI_GROUP_END();

CDI_GROUP(WiFiSoftAPConfiguration);

/// Allows configuration of the SSID used by the SoftAP.
CDI_GROUP_ENTRY(ssid, openlcb::StringConfigEntry<32>,
    Name("SSID"),
    Description("Configures the SSID that the ESP32 will use for the SoftAP."));
/// Allows configuration of the password used by the SoftAP. Note: This value
/// will be prefixed with [b64] and be base64 encoded when read via CDI.
CDI_GROUP_ENTRY(password, openlcb::StringConfigEntry<128>,
    Name("Password"),
    Description("Configures the password that the ESP32 will use for the SoftAP."));
/// Allows configuration of the authentication mode used by the SoftAP.
CDI_GROUP_ENTRY(auth, openlcb::Uint8ConfigEntry,
    Name("Authentication Mode"),
    Description("Configures the authentication mode of the SoftAP."),
    Min(0), Max(7), Default(3), /* WPA2 */
    MapValues(
R"!^!(<relation><property>0</property><value>Open</value></relation>
<relation><property>1</property><value>WEP</value></relation>
<relation><property>2</property><value>WPA</value></relation>
<relation><property>3</property><value>WPA2</value></relation>
<relation><property>4</property><value>WPA/WPA2</value></relation>
<relation><property>6</property><value>WPA3</value></relation>
<relation><property>7</property><value>WPA2/WPA3</value></relation>)!^!"));
/// Allows configuration of the WiFi channel used by the SoftAP.
CDI_GROUP_ENTRY(channel, openlcb::Uint8ConfigEntry,
    Name("Channel"),
    Description(
R"!^!(Configures the WiFi channel to use for the SoftAP.
Note: Some channels overlap eachother and may not provide optimal performance.
Recommended channels are: 1, 6, 11 since these do not overlap.)!^!"),
    Min(1), Max(14), Default(1));
CDI_GROUP_END();

CDI_GROUP(SntpConfiguration);
/// Enables the usage of SNTP on the node.
CDI_GROUP_ENTRY(enabled, openlcb::Uint8ConfigEntry,
    Name("Enable SNTP"),
    Description(
R"!^!(Enabling this option will allow the ESP32 to poll an SNTP server at
regular intervals to obtain the current time. The refresh interval roughly
once per hour.)!^!"),
    Min(0), Max(1), Default(0), /* Disabled */
    MapValues(
R"!^!(<relation><property>0</property><value>No</value></relation>
<relation><property>1</property><value>Yes</value></relation>)!^!"));
/// SNTP Server to poll for time updates.
CDI_GROUP_ENTRY(server, openlcb::StringConfigEntry<64>,
    Name("SNTP Server"),
    Description(
R"!^!(Enter the SNTP Server address. Example: pool.ntp.org
Most of the time this does not need to be changed.)!^!"));
/// Allows configuring the default TimeZone of the node.
CDI_GROUP_ENTRY(timezone, openlcb::StringConfigEntry<64>,
    Name("TimeZone"),
    Description(
R"!^!(This is the timezone that the ESP32 should use, note it must be in POSIX
notation. Note: The timezone is only configured when SNTP is also enabled.
A few common values:
PST8PDT,M3.2.0,M11.1.0 -- UTC-8 with automatic DST adjustment
MST7MDT,M3.2.0,M11.1.0 -- UTC-7 with automatic DST adjustment
CST6CDT,M3.2.0,M11.1.0 -- UTC-6 with automatic DST adjustment
EST5EDT,M3.2.0,M11.1.0 -- UTC-5 with automatic DST adjustment
A complete list can be seen here in the second column:
https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)!^!"));
CDI_GROUP_END();

CDI_GROUP(WiFiConfiguration, Segment(CONFIG_OLCB_WIFI_MEMORY_SPACE_ID),
          Offset(CONFIG_OLCB_WIFI_MEMORY_SPACE_OFFSET));

/// Allows configuring the WiFi operating mode of the node.
CDI_GROUP_ENTRY(wifi_mode, openlcb::Uint8ConfigEntry,
    Name("WiFi mode"),
    Description("Configures the WiFi operating mode."),
    Min(0), Max(3), Default(2), /* SoftAP */
    MapValues(
R"!^!(<relation><property>0</property><value>Off</value></relation>
<relation><property>1</property><value>Station Only</value></relation>
<relation><property>2</property><value>SoftAP Only</value></relation>
<relation><property>3</property><value>SoftAP and Station</value></relation>)!^!"));
/// Allows configuration of the node's hostname.
CDI_GROUP_ENTRY(hostname_prefix, openlcb::StringConfigEntry<21>,
    Name("Hostname prefix"),
    Description(
R"!^!(Configures the hostname prefix used by the node.
Note: the node ID will be appended to this value.)!^!"));
/// Allows configuration of the node's hostname.
CDI_GROUP_ENTRY(station, WiFiStationConfiguration,
    Name("Station Configuration"),
    Description(
R"!^!(Configures the station WiFi interface on the ESP32 node.
This is used to have the ESP32 join an existing WiFi network.)!^!"));
/// Allows configuration of the node's hostname.
CDI_GROUP_ENTRY(softap, WiFiSoftAPConfiguration, Name("SoftAP Configuration"),
    Description(
R"!^!(Configures the SoftAP WiFi interface on the ESP32 node.
This is used to have the ESP32 advertise itself as a access point.)!^!"));
/// Allows configuration of the node's hostname.
CDI_GROUP_ENTRY(sntp, SntpConfiguration,
    Name("SNTP Configuration"),
    Description(
R"!^!(Configures the simple network time server for the ESP32 to use.)!^!"));
CDI_GROUP_END();

} // namespace esp32cs

#endif // 
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

#ifndef CDI_HXX_
#define CDI_HXX_

#include "sdkconfig.h"
#include "FastClockConfigurationGroup.hxx"
#include "NodeIdConfigurationGroup.hxx"
#include "OpenLCBConfigurationGroup.hxx"
#include "RealTimeClockConfigurationGroup.hxx"
#include "ThermalConfigurationGroup.hxx"
#include "TrackOutputDescriptor.hxx"
#include "WiFiConfigurationGroup.hxx"
#if CONFIG_ROSTER_EXPOSE_VMS
#include <locodb/LocoDatabaseVirtualMemorySpace.hxx>
#endif
#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/MemoryConfig.hxx>

namespace esp32cs
{

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(CommandStationConfig,
          Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG), Offset(128));
/// Node internal configuration data.
CDI_GROUP_ENTRY(internal_config, openlcb::InternalConfigData);
/// Track output configuration data.
CDI_GROUP_ENTRY(track, TrackOutputConfig, Name("Track Configuration"));
/// Thermal monitoring configuration data.
CDI_GROUP_ENTRY(thermal, ThermalConfiguration,
                Name("Thermal Monitoring Configuration"));
CDI_GROUP_ENTRY(olcb, OpenLCBConfiguration, Name("OpenLCB Configuration"));
CDI_GROUP_END();

/// The main structure of the CDI
CDI_GROUP(ConfigDef, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA.
CDI_GROUP_ENTRY(ident, openlcb::Identification);
/// Adds an <acdi> tag.
CDI_GROUP_ENTRY(acdi, openlcb::Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
CDI_GROUP_ENTRY(userinfo, openlcb::UserInfoSegment, Name("User Info"));
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, CommandStationConfig,
                Name("General Configuration Settings"));
/// Virtual memory space for Node ID reconfiguration.
CDI_GROUP_ENTRY(node, NodeIdConfig, Name("Node ID"));
/// OpenLCB Fast Clock configuration.
CDI_GROUP_ENTRY(wifi, esp32cs::WiFiConfiguration, Name("WiFi Configuration"));
/// OpenLCB Fast Clock configuration.
CDI_GROUP_ENTRY(fastclock, FastClockConfiguration, Name("Fast Clock"));
/// OpenLCB Real Time Clock configuration.
CDI_GROUP_ENTRY(realtime_clock, RealTimeClockConfiguration,
                Name("Real-time Clock"));
#if CONFIG_ROSTER_EXPOSE_VMS
CDI_GROUP_ENTRY(loco_db, locodb::TrainDatabaseSegment,
                Name("Locomotive Roster"));
#endif
CDI_GROUP_END();

} // namespace esp32cs

// CONTENT BELOW IS GENERATED, DO NOT DIRECTLY EDIT!
namespace openlcb
{

extern const char CDI_DATA[];
const char CDI_DATA[] = R"xmlpayload(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>)xmlpayload" SNIP_PROJECT_PAGE R"xmlpayload(</manufacturer>
<model>)xmlpayload" SNIP_PROJECT_NAME R"xmlpayload(</model>
<hardwareVersion>)xmlpayload" SNIP_HW_VERSION " " CONFIG_IDF_TARGET R"xmlpayload(</hardwareVersion>
<softwareVersion>)xmlpayload" SNIP_SW_VERSION R"xmlpayload(</softwareVersion>
</identification>
<acdi/>
<segment space='251' origin='1'>
<name>User Info</name>
<string size='63'>
<name>User Name</name>
<description>This name will appear in network browsers for this device.</description>
</string>
<string size='64'>
<name>User Description</name>
<description>This description will appear in network browsers for this device.</description>
</string>
</segment>
<segment space='253' origin='128'>
<name>General Configuration Settings</name>
<group>
<name>Internal data</name>
<description>Do not change these settings.</description>
<int size='2'>
<name>Version</name>
</int>
<int size='2'>
<name>Next event ID</name>
</int>
</group>
<group>
<name>Track Configuration</name>
<int size='4'>
<name>Operations Track Current Limit (mA)</name>
<min>0</min>
<max>10000</max>
<default>5000</default>
</int>
<eventid>
<name>Short Detected</name>
<description>This event will be produced when a short has been detected on the track output.</description>
</eventid>
<eventid>
<name>H-Bridge Shutdown</name>
<description>This event will be produced when the track output power has exceeded the safety threshold of the H-Bridge.</description>
</eventid>
<group>
<name>Advanced Configuration Settings</name>
<int size='1'>
<name>Operations Track Preamble bit count</name>
<min>11</min>
<max>20</max>
<default>16</default>
</int>
<int size='1'>
<name>Programming Track Preamble bit count</name>
<min>22</min>
<max>50</max>
<default>22</default>
</int>
<int size='1'>
<name>Enable RailCom cut out generation</name>
<description>Enabling this option configures the Command Station to generate the RailCom cut-out.
NOTE: this applys to both the OPS track output and the OpenLCB output (if enabled).</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map>
<relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Enabled</value></relation>
</map>
</int>
<int size='1'>
<name>Enable RailCom Receiver</name>
<description>Enabling this option configures the Command Station to interpret the data received during the RailCom cut-out.
NOTE: this applys to both the OPS track output and the OpenLCB output (if enabled).</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map>
<relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Enabled</value></relation>
</map>
</int>
</group>
</group>
<group>
<name>Thermal Monitoring Configuration</name>
<int size='1'>
<name>Enable thermal monitoring</name>
<description>Enabling this option will allow the node to monitor an external temperature sensor and emit events when configured thresholds are breached.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map>
<relation><property>0</property><value>No</value></relation>
<relation><property>1</property><value>Yes</value></relation>
</map>
</int>
<int size='1'>
<name>Warning Temperature</name>
<description>Temperature (in celsius) to use for thermal warning threshold.</description>
<min>0</min>
<max>125</max>
<default>50</default>
</int>
<int size='1'>
<name>Shutdown Temperature</name>
<description>Temperature (in celsius) to use for thermal shutdown threshold.</description>
<min>0</min>
<max>125</max>
<default>80</default>
</int>
<eventid>
<name>Warning Temperature Exceeded</name>
<description>This event will be produced when the temperature has exceeded the warning temperature but is below the shutdown temperature.</description>
</eventid>
<eventid>
<name>Shutdown Temperature Exceeded</name>
<description>This event will be produced when the temperature has exceeded the shutdown temperature or there is a failure in reading the temperature.</description>
</eventid>
</group>
<group>
<name>OpenLCB Configuration</name>
<group>
<name>WiFi Usage</name>
<int size='1'>
<name>WiFi Power Savings Mode</name>
<description>When enabled this allows the ESP32 WiFi radio to use power savings mode which puts the radio to sleep except to receive beacon updates from the connected SSID. This should generally not need to be enabled unless you are powering the ESP32 from a battery.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>No</value></relation><relation><property>1</property><value>Yes</value></relation></map>
</int>
<int size='1'>
<name>Connection Mode</name>
<description>Defines whether to allow accepting connections (according to the Hub configuration), making a connection (according to the Uplink configuration), or both.
This setting can be set to Disabled if the ESP32 will be using the TWAI (CAN) driver instead for the connection to other nodes.
Note: it is not recommended to enable the Hub functionality on single-core ESP32 models.</description>
<min>0</min>
<max>3</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Uplink Only</value></relation><relation><property>2</property><value>Hub Only</value></relation><relation><property>3</property><value>Hub+Uplink</value></relation></map>
</int>
<group>
<name>Hub Configuration</name>
<description>Configuration settings for an OpenLCB Hub</description>
<int size='2'>
<name>Hub Listener Port</name>
<description>Defines the TCP/IP listener port this node will use when operating as a hub. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
<string size='48'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<group offset='6'/>
</group>
<group>
<name>Node Uplink Configuration</name>
<description>Configures how this node will connect to other nodes.</description>
<int size='1'>
<name>Search Mode</name>
<description>Defines the order of how to locate the server to connect to. 'auto' uses the mDNS protocol to find the IP address automatically. 'manual' uses the IP address entered in this settings.</description>
<min>0</min>
<max>3</max>
<default>0</default>
<map><relation><property>0</property><value>Auto, Manual</value></relation><relation><property>1</property><value>Manual, Auto</value></relation><relation><property>2</property><value>Auto Only</value></relation><relation><property>3</property><value>Manual Only</value></relation></map>
</int>
<group>
<name>Manual Address</name>
<description>Set IP address here if auto-detection does not work.</description>
<string size='32'>
<name>IP Address</name>
<description>Enter the server IP address. Example: 192.168.0.55</description>
</string>
<int size='2'>
<name>Port Number</name>
<description>TCP port number of the server. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
</group>
<group>
<name>Auto Address</name>
<description>Advanced settings for the server IP address auto-detection (mDNS).</description>
<string size='48'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<string size='48'>
<name>Only Hostname</name>
<description>Use when multiple servers provide the same service on the network. If set, selects this specific host name; the connection will fail if none of the servers have this hostname (use correct capitalization!). Example: My JMRI Railroad</description>
</string>
</group>
<int size='1'>
<name>Reconnect</name>
<description>If enabled, tries the last known good IP address before searching for the server.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<group offset='34'/>
</group>
<group offset='6'/>
</group>
<group>
<name>Advanced</name>
<int size='1'>
<name>Enable Brownout detection</name>
<description>Enabling this option configures the Command Station to send brownout events when the ESP32 restarts due to brownout.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>Enable DCC signal</name>
<description>Enabling this option configures the Command Station to output the DCC signal and RailCom cut-out (if enabled) to the OpenLCB connection on pins 4 and 5.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>Enable DCC Accessory Support</name>
<description>Enabling this option configures the Command Station to listen for any
DCC Accessory event and generate a compatible DCC Accessory packet or OpenLCB
events for virtual DCC accessories.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>Enable OpenLCB Throttle Support</name>
<description>Enabling this option configures the Command Station to listen for OpenLCB
Throttle requests.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>Throttle Heartbeats</name>
<description>Configures how often OpenLCB connected throttles must send a heartbeat
message to the Command Station. If a throttle does not send the heartbeat
within the configured limit, any trains controlled by the throttle will be
stopped. Set to zero to disable.</description>
<min>0</min>
<max>240</max>
<default>10</default>
</int>
</group>
</group>
</segment>
<segment space='170' origin='0'>
<name>Node ID</name>
<string size='32'>
<name>Node ID</name>
<description>Identifier to use for this device.
NOTE: Changing this value will force a factory reset.</description>
</string>
</segment>
<segment space='171' origin='250'>
<name>WiFi Configuration</name>
<int size='1'>
<name>WiFi mode</name>
<description>Configures the WiFi operating mode.</description>
<min>0</min>
<max>3</max>
<default>2</default>
<map><relation><property>0</property><value>Off</value></relation>
<relation><property>1</property><value>Station Only</value></relation>
<relation><property>2</property><value>SoftAP Only</value></relation>
<relation><property>3</property><value>SoftAP and Station</value></relation></map>
</int>
<string size='21'>
<name>Hostname prefix</name>
<description>Configures the hostname prefix used by the node.
Note: the node ID will be appended to this value.</description>
</string>
<group>
<name>Station Configuration</name>
<description>Configures the station WiFi interface on the ESP32 node.
This is used to have the ESP32 join an existing WiFi network.</description>
<string size='32'>
<name>SSID</name>
<description>Configures the SSID that the ESP32 will connect to.</description>
</string>
<string size='128'>
<name>Password</name>
<description>Configures the password that the ESP32 will use for the station SSID.</description>
</string>
</group>
<group>
<name>SoftAP Configuration</name>
<description>Configures the SoftAP WiFi interface on the ESP32 node.
This is used to have the ESP32 advertise itself as a access point.</description>
<string size='32'>
<name>SSID</name>
<description>Configures the SSID that the ESP32 will use for the SoftAP.</description>
</string>
<string size='128'>
<name>Password</name>
<description>Configures the password that the ESP32 will use for the SoftAP.</description>
</string>
<int size='1'>
<name>Authentication Mode</name>
<description>Configures the authentication mode of the SoftAP.</description>
<min>0</min>
<max>7</max>
<default>3</default>
<map><relation><property>0</property><value>Open</value></relation>
<relation><property>1</property><value>WEP</value></relation>
<relation><property>2</property><value>WPA</value></relation>
<relation><property>3</property><value>WPA2</value></relation>
<relation><property>4</property><value>WPA/WPA2</value></relation>
<relation><property>6</property><value>WPA3</value></relation>
<relation><property>7</property><value>WPA2/WPA3</value></relation></map>
</int>
<int size='1'>
<name>Channel</name>
<description>Configures the WiFi channel to use for the SoftAP.
Note: Some channels overlap eachother and may not provide optimal performance.
Recommended channels are: 1, 6, 11 since these do not overlap.</description>
<min>1</min>
<max>14</max>
<default>1</default>
</int>
</group>
<group>
<name>SNTP Configuration</name>
<description>Configures the simple network time server for the ESP32 to use.</description>
<int size='1'>
<name>Enable SNTP</name>
<description>Enabling this option will allow the ESP32 to poll an SNTP server at
regular intervals to obtain the current time. The refresh interval roughly
once per hour.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>No</value></relation>
<relation><property>1</property><value>Yes</value></relation></map>
</int>
<string size='64'>
<name>SNTP Server</name>
<description>Enter the SNTP Server address. Example: pool.ntp.org
Most of the time this does not need to be changed.</description>
</string>
<string size='64'>
<name>TimeZone</name>
<description>This is the timezone that the ESP32 should use, note it must be in POSIX
notation. Note: The timezone is only configured when SNTP is also enabled.
A few common values:
PST8PDT,M3.2.0,M11.1.0 -- UTC-8 with automatic DST adjustment
MST7MDT,M3.2.0,M11.1.0 -- UTC-7 with automatic DST adjustment
CST6CDT,M3.2.0,M11.1.0 -- UTC-6 with automatic DST adjustment
EST5EDT,M3.2.0,M11.1.0 -- UTC-5 with automatic DST adjustment
A complete list can be seen here in the second column:
https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv</description>
</string>
</group>
</segment>
<segment space='172' origin='1000'>
<name>Fast Clock</name>
<int size='1'>
<name>Enable Clock</name>
<description>Enabling this option configures the Command Station to generate OpenLCB
FastClock events.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='32'>
<name>Clock ID</name>
<description>The Clock ID is used as the upper six bytes of all BroadcastTime events
generated as part of the operation of the Fast Clock.
Common ID values:
01.01.00.00.01.00 - Default Fast Clock
01.01.00.00.01.01 - Default Real-time Clock
01.01.00.00.01.02 - Alternate Clock 1
01.01.00.00.01.03 - Alternate Clock 2</description>
</string>
<int size='2'>
<name>Year</name>
<description>Current year on the fast clock</description>
<min>1900</min>
</int>
<int size='1'>
<name>Month</name>
<description>Current month on the fast clock</description>
<min>1</min>
<max>12</max>
<map><relation><property>1</property><value>January</value></relation>
<relation><property>2</property><value>Feburary</value></relation>
<relation><property>3</property><value>March</value></relation>
<relation><property>4</property><value>April</value></relation>
<relation><property>5</property><value>May</value></relation>
<relation><property>6</property><value>June</value></relation>
<relation><property>7</property><value>July</value></relation>
<relation><property>8</property><value>August</value></relation>
<relation><property>9</property><value>September</value></relation>
<relation><property>10</property><value>October</value></relation>
<relation><property>11</property><value>November</value></relation>
<relation><property>12</property><value>December</value></relation></map>
</int>
<int size='1'>
<name>Day</name>
<description>Current day on the fast clock</description>
<min>1</min>
<max>31</max>
</int>
<int size='1'>
<name>Hour</name>
<description>Current hour of the day on the fast clock</description>
<min>0</min>
<max>23</max>
</int>
<int size='1'>
<name>Day</name>
<description>Current minute of the hour on the fast clock</description>
<min>0</min>
<max>59</max>
</int>
<int size='2'>
<name>Rate</name>
<description>Rate at which the clock advances</description>
<min>-512</min>
<max>512</max>
<default>4</default>
</int>
</segment>
<segment space='173' origin='1500'>
<name>Real-time Clock</name>
<int size='1'>
<name>Enable Clock</name>
<description>Enabling this option configures the Command Station to generate OpenLCB
FastClock events.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='32'>
<name>Clock ID</name>
<description>The Clock ID is used as the upper six bytes of all BroadcastTime events
generated as part of the operation of the Fast Clock.
Common ID values:
01.01.00.00.01.00 - Default Fast Clock
01.01.00.00.01.01 - Default Real-time Clock
01.01.00.00.01.02 - Alternate Clock 1
01.01.00.00.01.03 - Alternate Clock 2</description>
</string>
</segment>)xmlpayload"
#if CONFIG_ROSTER_EXPOSE_VMS
<segment space='187' origin='2048'>
<name>Locomotive Roster</name>
<int size='2'>
<name>Entry Number</name>
<description>Locomotive Database entry number, write to this value and refresh other fields to load current values.</description>
<default>0</default>
</int>
<int size='2'>
<name>Entry Number</name>
<description>Maximum value Locomotive Database entry number, read-only. Attempting to write to this value will be ignored.</description>
<default>0</default>
</int>
<int size='2'>
<name>Address</name>
<description>Track protocol address of the train.</description>
<default>0</default>
</int>
<int size='1'>
<name>Protocol</name>
<description>Protocol to use on the track for driving this train.</description>
<default>11</default>
<map><relation><property>9</property><value>DCC (14 speed steps)</value></relation><relation><property>10</property><value>DCC (28 speed steps)</value></relation><relation><property>11</property><value>DCC (128 speed steps)</value></relation><relation><property>13</property><value>DCC (14 speed steps, long address)</value></relation><relation><property>14</property><value>DCC (28 speed steps, long address)</value></relation><relation><property>15</property><value>DCC (128 speed steps, long address)</value></relation><relation><property>4</property><value>Marklin (default)</value></relation><relation><property>5</property><value>Marklin v1 (F0 only)</value></relation><relation><property>6</property><value>Marklin v2 (F0-F4)</value></relation><relation><property>7</property><value>Marklin v2+ (F0-F8, two addresses)</value></relation></map>
</int>
<string size='63'>
<name>Name</name>
<description>Identifies the train node on the LCC bus.</description>
</string>
<string size='64'>
<name>Description</name>
<description>Describes the train node on the LCC bus.</description>
</string>
<group>
<name>F0</name>
<description>F0 is permanently assigned to the Headlight.</description>
<group offset='2'/>
</group>
<group replication='28'>
<name>Functions</name>
<description>Defines what each function button does.</description>
<repname>Fn</repname>
<int size='1'>
<name>Display</name>
<description>Defines how throttles display this function.</description>
<default>0</default>
<map><relation><property>0</property><value>Not Available</value></relation><relation><property>1</property><value>Headlight</value></relation><relation><property>4</property><value>Engine</value></relation><relation><property>6</property><value>Announce</value></relation><relation><property>7</property><value>Shunting Mode</value></relation><relation><property>8</property><value>Momentum</value></relation><relation><property>9</property><value>Uncouple</value></relation><relation><property>10</property><value>Smoke</value></relation><relation><property>11</property><value>Pantograph</value></relation><relation><property>12</property><value>Far Light</value></relation><relation><property>13</property><value>Bell</value></relation><relation><property>14</property><value>Horn</value></relation><relation><property>15</property><value>Whistle</value></relation><relation><property>16</property><value>Light</value></relation><relation><property>17</property><value>Mute</value></relation><relation><property>127</property><value>Unknown</value></relation><relation><property>255</property><value>Undefined</value></relation></map>
</int>
<int size='1'>
<name>Momentary</name>
<description>Momentary functions are automatically turned off when you release the respective button on the throttles.</description>
<default>0</default>
<map><relation><property>0</property><value>Latching</value></relation><relation><property>1</property><value>Momentary</value></relation></map>
</int>
</group>
</segment>
#endif
R"xmlpayload(
</cdi>
)xmlpayload";
extern const size_t CDI_SIZE;
const size_t CDI_SIZE = sizeof(CDI_DATA);

extern const uint16_t CDI_EVENT_OFFSETS[] =
{
  136, 144, 159, 167, 0
};
}  // namespace openlcb

#endif // CDI_HXX_
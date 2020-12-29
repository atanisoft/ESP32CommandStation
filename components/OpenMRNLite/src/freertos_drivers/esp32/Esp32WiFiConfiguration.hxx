/** \copyright
 * Copyright (c) 2019, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file Esp32WiFiConfiguration.hxx
 *
 * ESP32 WiFiConfiguration CDI declarations
 *
 * @author Mike Dunston
 * @date 11 February 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_

#include "sdkconfig.h"
#include "openlcb/ConfigRepresentation.hxx"

namespace openmrn_arduino
{

/// Names and Descriptions for all ESP32 exposed WiFi configuration options.
class Esp32WiFiConfigurationParams
{
public:
    /// <map> of possible keys and descriptive values to show to the user for
    /// the wifi_sleep and hub_mode fields.
    static constexpr const char *BOOLEAN_MAP =
        "<relation><property>0</property><value>Disabled</value></relation>"
        "<relation><property>1</property><value>Enabled</value></relation>";

    /// Visible name for the WiFi Power Savings mode.
    static constexpr const char *WIFI_POWER_SAVE_NAME =
        "WiFi Power Savings Mode";

    /// Visible description for the WiFi Power Savings mode.
    static constexpr const char *WIFI_POWER_SAVE_DESC =
        "When enabled this allows the ESP32 WiFi radio to use power savings "
        "mode which puts the radio to sleep except to receive beacon updates "
        "from the connected SSID. This should generally not need to be "
        "enabled unless you are powering the ESP32 from a battery.";

    /// Visible name for the WiFi TX Power.
    static constexpr const char *WIFI_TX_POWER_NAME =
        "WiFi Transmit Power";

    /// Visible description for the WiFi TX Power.
    static constexpr const char *WIFI_TX_POWER_DESC =
        "WiFi Radio transmit power in dBm. This can be used to limit the WiFi "
        "range. This option generally does not need to be changed.\n"
        "NOTE: Setting this option to a very low value can cause communication "
        "failures.";

    /// <map> of possible key and descriptive values to show to the user for
    /// the power field.
    static constexpr const char *WIFI_TX_POWER_MAP =
        "<relation><property>8</property><value>2 dBm</value></relation>"
        "<relation><property>20</property><value>5 dBm</value></relation>"
        "<relation><property>28</property><value>7 dBm</value></relation>"
        "<relation><property>34</property><value>8 dBm</value></relation>"
        "<relation><property>44</property><value>11 dBm</value></relation>"
        "<relation><property>52</property><value>13 dBm</value></relation>"
        "<relation><property>56</property><value>14 dBm</value></relation>"
        "<relation><property>60</property><value>15 dBm</value></relation>"
        "<relation><property>66</property><value>16 dBm</value></relation>"
        "<relation><property>72</property><value>18 dBm</value></relation>"
        "<relation><property>78</property><value>20 dBm</value></relation>";

    /// Visible name for the Hub Configuration group.
    static constexpr const char *HUB_NAME = "Hub Configuration";

    /// Visible description for the Hub Configuration group.
    static constexpr const char *HUB_DESC =
        "Configuration settings for an OpenLCB Hub";

    /// Visible name for the hub enable field.
    static constexpr const char *HUB_ENABLE_NAME = "Enable";

    /// Visible description for the hub enable field.
    static constexpr const char *HUB_ENABLE_DESC =
        "Configures this node as an OpenLCB hub which can accept connections "
        "from other nodes.\nNOTE: This may cause some instability as the "
        "number of connected nodes increases.";

    /// Visible name for the hub_listener_port field.
    static constexpr const char *HUB_LISTENER_PORT_NAME = "Hub Listener Port";

    /// Visible name for the hub_listener_port field.
    static constexpr const char *HUB_LISTENER_PORT_DESC =
        "Defines the TCP/IP listener port this node will use when operating "
        "as a hub. Most of the time this does not need to be changed.";

    /// Visible name for the uplink group.
    static constexpr const char *UPLINK_NAME = "Uplink Configuration";

    /// Visible name for the uplink group.
    static constexpr const char *UPLINK_DESC =
        "Configures how this node will connect to other nodes.";

    /// Visible name for the hub enable field.
    static constexpr const char *UPLINK_ENABLE_NAME = "Enable";

    /// Visible description for the hub enable field.
    static constexpr const char *UPLINK_ENABLE_DESC =
        "Enables connecting to an OpenLCB Hub. In some cases it may be "
        "desirable to disable the uplink, such as a CAN only configuration.";

    /// Visible name for the advanced configuration group.
    static constexpr const char *ADVANCED_CONFIG_NAME =
        "Advanced Configuration Options";

    /// Visible description for the advanced configuration group.
    static constexpr const char *ADVANCED_CONFIG_DESC =
        "These are advanced settings that typically do not need to be adjusted.";

    /// <map> of supported values for the wifi_mode field.
    static constexpr const char *WIFI_MODES_MAP =
        "<relation><property>0</property><value>Off</value></relation>"
        "<relation><property>1</property><value>Station Only</value></relation>"
        "<relation><property>2</property><value>SoftAP Only</value></relation>"
        "<relation><property>3</property><value>SoftAP and Station</value></relation>";

    /// Visible name for the wifi_mode field.
    static constexpr const char *WIFI_MODE_NAME = "WiFi mode";

    /// Visible description for the wifi_mode field.
    static constexpr const char *WIFI_MODE_DESC =
        "Configures the WiFi operating mode.";

    /// Visible name for the hostname_prefix field.
    static constexpr const char *HOSTNAME_NAME = "Hostname prefix";

    /// Visible description for the hostname_prefix field.
    static constexpr const char *HOSTNAME_DESC =
        "Configures the hostname prefix used by the node.\nNote: the node ID "
        "will be appended to this value.";

    /// Visible name for the station group.
    static constexpr const char *STATION_NAME = "Station Configuration";

    /// Visible description for the station group.
    static constexpr const char *STATION_DESC =
        "Configures the station WiFi interface on the ESP32 node.\n"
        "This is used to have the ESP32 join an existing WiFi network.";

    /// Visible name for the softap group.
    static constexpr const char *SOFTAP_NAME = "SoftAP Configuration";

    /// Visible description for the softap group
    static constexpr const char *SOFTAP_DESC =
        "Configures the SoftAP WiFi interface on the ESP32 node.\n"
        "This is used to have the ESP32 advertise itself as a access point.";

    /// Visible name for the ssid field.
    static constexpr const char *STATION_SSID_NAME = "Station SSID";

    /// Visible description for the Station ssid field.
    static constexpr const char *STATION_SSID_DESC =
        "Configures the SSID that the ESP32 will connect to.";

    /// Visible name for the ssid field.
    static constexpr const char *SOFTAP_SSID_NAME = "SoftAP SSID";

    /// Visible description for the SoftAP ssid field.
    static constexpr const char *SOFTAP_SSID_DESC =
        "Configures the SSID that the ESP32 will use for the SoftAP.";

    /// Visible name for the Station reboot_on_failure field.
    static constexpr const char *STATION_WAIT_FOR_CONNECT_NAME =
        "Wait for successful SSID connection";

    /// Visible description for the Station reboot_on_failure field.
    static constexpr const char *STATION_WAIT_FOR_CONNECT_DESC =
        "Enabling this option will cause the node to restart when there is a "
        "failure (or timeout) during the SSID connection process.";

    /// Visible name for the password field.
    static constexpr const char *STATION_PASSWORD_NAME = "Station password";

    /// Visible description for the Station password field.
    static constexpr const char *STATION_PASSWORD_DESC =
        "Configures the password that the ESP32 will use for the station SSID.";

    /// Visible name for the password field.
    static constexpr const char *SOFTAP_PASSWORD_NAME = "SoftAP assword";

    /// Visible description for the SoftAP password field.
    static constexpr const char *SOFTAP_PASSWORD_DESC =
        "Configures the password that the ESP32 will use for the SoftAP.";

    /// <map> of supported values for the SoftAP authentication field.
    static constexpr const char *SOFTAP_AUTH_MAP =
        "<relation><property>0</property><value>Open</value></relation>"
        "<relation><property>1</property><value>WEP</value></relation>"
        "<relation><property>2</property><value>WPA</value></relation>"
        "<relation><property>3</property><value>WPA2</value></relation>"
        "<relation><property>4</property><value>WPA/WPA2</value></relation>"
        // WPA2 Enterprise omitted
        "<relation><property>6</property><value>WPA3</value></relation>"
        "<relation><property>7</property><value>WPA2/WPA3</value></relation>";

    /// Visible name for the SoftAP auth field.
    static constexpr const char *SOFTAP_AUTH_NAME = "Authentication Mode";

    /// Visible description for the SoftAP auth field.
    static constexpr const char *SOFTAP_AUTH_DESC =
        "Configures the authentication mode of the SoftAP.";

    /// Visible name for the SoftAP channel field.
    static constexpr const char *SOFTAP_CHANNEL_NAME = "WiFi Channel";

    /// Visible description for the SoftAP channel field.
    static constexpr const char *SOFTAP_CHANNEL_DESC =
        "Configures the WiFi channel to use for the SoftAP.\nNote: Some "
        "channels overlap eachother and may not provide optimal performance."
        "Recommended channels are: 1, 6, 11 since these do not overlap.";

    /// Visible name of the service name field in the uplink and hub groups.
    static constexpr const char *SERVICE_NAME = "mDNS Service";
    /// Visible description of the service name field in the uplink and hub
    /// groups.
    static constexpr const char *SERVICE_DESC =
        "mDNS or Bonjour service name, such as _openlcb-can._tcp";

    /// Visible name of the ip address field inside the uplink group.
    static constexpr const char *IP_ADDRESS_NAME = "IP Address";
    /// Visible description of the ip address field inside the uplink group.
    static constexpr const char *IP_ADDRESS_DESC =
        "Enter the server IP address. Example: 192.168.0.55\n"
        "Note: This will be used as a fallback when mDNS lookup is not "
        "successful.";

    /// Default value for the port number field. This will also be used for
    /// factory reset code, not just for user documentation.
    static constexpr int DEFAULT_PORT = 12021;
    /// Visible name of the port number field.
    static constexpr const char *PORT_NAME = "Port Number";
    /// Visible description of the port number field.
    static constexpr const char *PORT_DESC =
        "TCP port number of the server. Most of the time this does not need to "
        "be changed.";

    /// Visible name of the sntp_enable field.
    static constexpr const char *SNTP_ENABLE_NAME = "Enable SNTP";

    /// Visible description of the sntp_enable field.
    static constexpr const char *SNTP_ENABLE_DESC =
        "Enabling this option will allow the ESP32 to poll an SNTP server at "
        "regular intervals to obtain the current time. The refresh interval "
        "roughly once per hour.";

    /// Visible name of the sntp_server field.
    static constexpr const char *SNTP_SERVER_NAME = "SNTP Server";

    /// Visible description of the sntp_server field.
    static constexpr const char *SNTP_SERVER_DESC =
        "Enter the SNTP Server address. Example: pool.ntp.org\n"
        "Most of the time this does not need to be changed.";

    /// Visible name of the timezone field.
    static constexpr const char *TIMEZONE_NAME = "TimeZone";
    
    /// Visible description of the timezone field.
    static constexpr const char *TIMEZONE_DESC =
        "This is the timezone that the ESP32 should use, note it must be in "
        "POSIX notation. Note: The timezone is only configured when SNTP is "
        "also enabled.\n"
        "A few common values:\n"
        "PST8PDT,M3.2.0,M11.1.0 -- UTC-8 with automatic DST adjustment\n"
        "MST7MDT,M3.2.0,M11.1.0 -- UTC-7 with automatic DST adjustment\n"
        "CST6CDT,M3.2.0,M11.1.0 -- UTC-6 with automatic DST adjustment\n"
        "EST5EDT,M3.2.0,M11.1.0 -- UTC-5 with automatic DST adjustment\n"
        "A complete list can be seen here in the second column:\n"
        "https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv";
};

/// CDI Configuration for an @ref Esp32WiFiManager managed hub.
CDI_GROUP(HubConfiguration);
/// Allows the node to become a Grid Connect Hub.
CDI_GROUP_ENTRY(enable, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::HUB_ENABLE_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_ENABLE_DESC),
    Min(0), Max(1), Default(0), /* Disabled */
    MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// Specifies the port which should be used by the hub.
CDI_GROUP_ENTRY(port, openlcb::Uint16ConfigEntry,
    Name(Esp32WiFiConfigurationParams::HUB_LISTENER_PORT_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_LISTENER_PORT_DESC),
    Min(1), Max(65535),
    Default(Esp32WiFiConfigurationParams::DEFAULT_PORT));
/// Specifies the mDNS service name to advertise for the hub.
CDI_GROUP_ENTRY(service_name, openlcb::StringConfigEntry<64>,
    Name(Esp32WiFiConfigurationParams::SERVICE_NAME),
    Description(Esp32WiFiConfigurationParams::SERVICE_DESC));
/// Reserved space for future expansion.
CDI_GROUP_ENTRY(reserved, openlcb::BytesConfigEntry<6>, Hidden(true));
CDI_GROUP_END();

/// CDI Configuration for an @ref Esp32WiFiManager managed uplink connection.
CDI_GROUP(UplinkConfiguration);
/// Enables the uplink connection.
CDI_GROUP_ENTRY(enable, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::UPLINK_ENABLE_NAME),
    Description(Esp32WiFiConfigurationParams::UPLINK_ENABLE_DESC),
    Min(0), Max(1), Default(1), /* Enabled */
    MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// Specifies the mDNS service name to search for.
CDI_GROUP_ENTRY(service_name, openlcb::StringConfigEntry<64>,
    Name(Esp32WiFiConfigurationParams::SERVICE_NAME),
    Description(Esp32WiFiConfigurationParams::SERVICE_DESC));
CDI_GROUP_ENTRY(ip_address, openlcb::StringConfigEntry<64>,
    Name(Esp32WiFiConfigurationParams::IP_ADDRESS_NAME),
    Description(Esp32WiFiConfigurationParams::IP_ADDRESS_DESC));
CDI_GROUP_ENTRY(port, openlcb::Uint16ConfigEntry,
    Name(Esp32WiFiConfigurationParams::PORT_NAME),
    Description(Esp32WiFiConfigurationParams::PORT_DESC),
    Min(1), Max(65535),
    Default(Esp32WiFiConfigurationParams::DEFAULT_PORT));
CDI_GROUP_END();

/// CDI Configuration for an @ref Esp32WiFiManager managed node.
CDI_GROUP(WiFiConfiguration);
/// Allows configuring the WiFi operating mode of the node.
CDI_GROUP_ENTRY(wifi_mode, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::WIFI_MODE_NAME),
    Description(Esp32WiFiConfigurationParams::WIFI_MODE_DESC),
    Min(0), Max(3), Default(2), /* SoftAP */
    MapValues(Esp32WiFiConfigurationParams::WIFI_MODES_MAP));
/// Allows configuration of the node's hostname.
CDI_GROUP_ENTRY(hostname_prefix, openlcb::StringConfigEntry<21>,
    Name(Esp32WiFiConfigurationParams::HOSTNAME_NAME),
    Description(Esp32WiFiConfigurationParams::HOSTNAME_DESC));
/// Allows setting the SSID that the Station will attempt to connect to.
CDI_GROUP_ENTRY(station_ssid, openlcb::StringConfigEntry<32>,
    Name(Esp32WiFiConfigurationParams::STATION_SSID_NAME),
    Description(Esp32WiFiConfigurationParams::STATION_SSID_DESC));
/// Allows setting the password that the Station will use for connecting to the
/// configured SSID. Note: This value will be prefixed with [b64] and be base64
/// encoded when read via CDI.
CDI_GROUP_ENTRY(station_password, openlcb::StringConfigEntry<128>,
    Name(Esp32WiFiConfigurationParams::STATION_PASSWORD_NAME),
    Description(Esp32WiFiConfigurationParams::STATION_PASSWORD_DESC));
/// Allows configuration of the SSID used by the SoftAP.
CDI_GROUP_ENTRY(softap_ssid, openlcb::StringConfigEntry<32>,
    Name(Esp32WiFiConfigurationParams::SOFTAP_SSID_NAME),
    Description(Esp32WiFiConfigurationParams::SOFTAP_SSID_DESC));
/// Allows configuration of the password used by the SoftAP. Note: This value
/// will be prefixed with [b64] and be base64 encoded when read via CDI.
CDI_GROUP_ENTRY(softap_password, openlcb::StringConfigEntry<128>,
    Name(Esp32WiFiConfigurationParams::SOFTAP_PASSWORD_NAME),
    Description(Esp32WiFiConfigurationParams::SOFTAP_PASSWORD_DESC));
/// Allows configuration of the authentication mode used by the SoftAP.
CDI_GROUP_ENTRY(softap_auth, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::SOFTAP_AUTH_NAME),
    Description(Esp32WiFiConfigurationParams::SOFTAP_AUTH_DESC),
    Min(0), Max(7), Default(3), /* WPA2 */
    MapValues(Esp32WiFiConfigurationParams::SOFTAP_AUTH_MAP));
/// Allows configuration of the WiFi channel used by the SoftAP.
CDI_GROUP_ENTRY(softap_channel, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::SOFTAP_CHANNEL_NAME),
    Description(Esp32WiFiConfigurationParams::SOFTAP_CHANNEL_DESC),
    Min(1), Max(14), Default(1));
/// Enables the usage of SNTP on the node.
CDI_GROUP_ENTRY(sntp_enabled, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::SNTP_ENABLE_NAME),
    Description(Esp32WiFiConfigurationParams::SNTP_ENABLE_DESC),
    Min(0), Max(1), Default(0), /* Disabled */
    MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// SNTP Server to poll for time updates.
CDI_GROUP_ENTRY(sntp_server, openlcb::StringConfigEntry<64>,
    Name(Esp32WiFiConfigurationParams::SNTP_SERVER_NAME),
    Description(Esp32WiFiConfigurationParams::SNTP_SERVER_DESC));
/// Allows configuring the default TimeZone of the node.
CDI_GROUP_ENTRY(timezone, openlcb::StringConfigEntry<64>,
    Name(Esp32WiFiConfigurationParams::TIMEZONE_NAME),
    Description(Esp32WiFiConfigurationParams::TIMEZONE_DESC));
/// CDI Configuration to enable this node to be a hub.
CDI_GROUP_ENTRY(hub, HubConfiguration,
    Name(Esp32WiFiConfigurationParams::HUB_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_DESC)
// On the ESP32 S2 hide the Hub option by default since there is less system
// resources available.
#ifndef CONFIG_IDF_TARGET_ESP32
    , Hidden(true)
#endif // CONFIG_IDF_TARGET_ESP32
);
/// CDI Configuration for this node's connection to an uplink hub.
CDI_GROUP_ENTRY(uplink, UplinkConfiguration,
    Name(Esp32WiFiConfigurationParams::UPLINK_NAME),
    Description(Esp32WiFiConfigurationParams::UPLINK_DESC));
/// Allows the WiFi system to use power-saving techniques to conserve power
/// when the node is powered via battery.
CDI_GROUP_ENTRY(sleep, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::WIFI_POWER_SAVE_NAME),
    Description(Esp32WiFiConfigurationParams::WIFI_POWER_SAVE_DESC),
    Min(0), Max(1), Default(0), /* Disabled */
    MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// Allows adjustment of the WiFi TX power. This can be beneficial for reducing
/// the available range of the SoftAP. However, it can cause communication
/// failures when connecting nodes via TCP/IP.
CDI_GROUP_ENTRY(tx_power, openlcb::Int8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::WIFI_TX_POWER_NAME),
    Description(Esp32WiFiConfigurationParams::WIFI_TX_POWER_DESC),
    Min(8), Max(78), Default(78), /* Max power */
    MapValues(Esp32WiFiConfigurationParams::WIFI_TX_POWER_MAP));
/// Allows controlling if the ESP32 will reboot the node if there is a timeout
/// while connecting to the configured SSID.
CDI_GROUP_ENTRY(station_wait_for_connect, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::STATION_WAIT_FOR_CONNECT_NAME),
    Description(Esp32WiFiConfigurationParams::STATION_WAIT_FOR_CONNECT_DESC),
    Min(0), Max(1), Default(1), /* Enabled */
    MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));

CDI_GROUP_END(); // WiFiConfiguration

} // namespace openmrn_arduino

using openmrn_arduino::WiFiConfiguration;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_
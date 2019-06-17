/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

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

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters
//
#ifdef CONFIG_SSID_NAME
  #define SSID_NAME CONFIG_SSID_NAME
#else
  #define SSID_NAME "SSID_NAME_HERE"
#endif
#ifdef CONFIG_SSID_PASSWORD
  #define SSID_PASSWORD CONFIG_SSID_PASSWORD
#else
  #define SSID_PASSWORD "SSID_PASSWORD_HERE"
#endif

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS DETAILS OR LEAVE COMMENTED FOR DHCP
//

//#define WIFI_STATIC_IP_ADDRESS "192.168.0.115"
//#define WIFI_STATIC_IP_GATEWAY "192.168.0.1"
//#define WIFI_STATIC_IP_SUBNET "255.255.255.0"

// WIFI_STATIC_IP_DNS is optional, if not defined the value below will be used
// automatically. This is a Google provided DNS server.
//#define WIFI_STATIC_IP_DNS "8.8.8.8"

/////////////////////////////////////////////////////////////////////////////////////
//
// Remote Sensors are enabled by default, the following parameters can be used
// to control their behavior.

//#define SCAN_REMOTE_SENSORS_ON_STARTUP true
//#define REMOTE_SENSORS_PREFIX "sensor"
//#define REMOTE_SENSORS_DECAY 60000
//#define REMOTE_SENSORS_FIRST_SENSOR 100

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE HOSTNAME TO USE FOR WiFi CONNECTIONS AND mDNS BROADCASTS
//
#ifdef CONFIG_HOSTNAME
  #define HOSTNAME CONFIG_HOSTNAME
#else
  #define HOSTNAME "ESP32CS"
#endif

/////////////////////////////////////////////////////////////////////////////////////

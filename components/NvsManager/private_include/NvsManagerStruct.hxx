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

#ifndef NVS_STRUCT_HXX_
#define NVS_STRUCT_HXX_

#include <esp_wifi_types.h>

namespace esp32cs
{
    typedef struct
    {
        uint64_t node_id;
        wifi_mode_t wifi_mode;
        char hostname_prefix[16];
        char station_ssid[33];
        char station_pass[33];
        char softap_ssid[33];
        char softap_pass[33];
        wifi_auth_mode_t softap_auth;
        uint8_t softap_channel;
        char sntp_server[33];
        char timezone[33];
        bool sntp_enabled;
        uint8_t led_brightness;
        uint64_t fastclock_id;
        int16_t fastclock_rate;   //
        uint8_t fastclock_year;   // 0 = 1900
        uint8_t fastclock_month;  // 1-12
        uint8_t fastclock_day;    // 1-31
        uint8_t fastclock_hour;   // 0-23
        uint8_t fastclock_minute; // 0-59
        bool fastclock_enabled;
        bool realtimeclock_enabled;
        uint64_t realtimeclock_id;
        uint8_t reserved[20];
    } node_config_t;

} // namespace esp32cs

#endif // NVS_STRUCT_HXX_
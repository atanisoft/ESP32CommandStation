/**********************************************************************
DCC COMMAND STATION FOR ESP32

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

#pragma once

#include <WString.h>
#include "Config.h"

#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 1
#define INFO_SCREEN_WS_CLIENTS_LINE 2
#define INFO_SCREEN_TRACK_POWER_LINE 3
#define INFO_SCREEN_ROTATING_STATUS_LINE 4
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD && INFO_SCREEN_LCD_LINES > 2
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 1
#define INFO_SCREEN_WS_CLIENTS_LINE -1
#define INFO_SCREEN_TRACK_POWER_LINE 2
#define INFO_SCREEN_ROTATING_STATUS_LINE 3
#else
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 0
#define INFO_SCREEN_WS_CLIENTS_LINE -1
#define INFO_SCREEN_TRACK_POWER_LINE -1
#define INFO_SCREEN_ROTATING_STATUS_LINE 1
#endif

class InfoScreen {
  public:
    static void init();
    static void clear();
    static void print(int, int, const __FlashStringHelper *, ...);
    static void print(int, int, const String &, ...);
    static void replaceLine(int, const __FlashStringHelper *, ...);
    static void replaceLine(int, const String &, ...);
    static void update();
  private:
    static bool _enabled;
};

/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

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

#ifndef _INFOSCREEN_H_
#define _INFOSCREEN_H_

#define INFO_SCREEN_MAX_LINES 5

class InfoScreen {
  public:
    static void init();
    static void clear();
    static void printf(int col, int row, const __FlashStringHelper *format, ...);
    static void printf(int col, int row, const char *format, ...);
  private:
    static bool _enabled;
  #if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    static String _lines[INFO_SCREEN_MAX_LINES];
  #endif
};

#endif

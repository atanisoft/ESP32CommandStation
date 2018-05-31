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

#ifndef _WIFI_INTERFACE_H_
#define _WIFI_INTERFACE_H_

#include <ESPAsyncWebServer.h>

class WiFiInterface {
public:
	WiFiInterface();
	void begin();
	void update();
	void showConfiguration();
	void showInitInfo();
	void send(const String &);
	void printf(const __FlashStringHelper *fmt, ...);
};

#endif

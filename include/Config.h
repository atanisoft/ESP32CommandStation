/**********************************************************************
DCC++ BASE STATION FOR ESP32

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
// The WiFi interface is part of the DCC++ESP32 core functionality
// and will require modification in the Config_WiFi.h file.
#include "Config_WiFi.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The Config_MotorBoard.h file will likely require modifications
// to define pins and motor board type(s)
#include "Config_MotorBoard.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The following line alters the default behavior of the DCC++ESP32 Command Station
// in such that it will automatically energize the OPS track power on startup. This
// may not be desirable and is left disabled by default. Uncomment the next line to
// enable this functionality.
//#define ENERGIZE_OPS_TRACK_ON_STARTUP true

/////////////////////////////////////////////////////////////////////////////////////
//
// The S88 module is optional and if enabled will allow communication with
// one (or more) S88 buses. To utilize this functionality uncomment the
// include below and edit the Config_S88.h file to match your configuration.
//#include "Config_S88.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The InfoScreen module is optional and displays base station information on either
// an OLED or LCD screen.
//#include "Config_OLED.h"
//#include "Config_LCD.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The Nextion interface is an optional module that allows a Nextion display to
// be connected directly to the base station as a throttle and programming interface.
//#include "Config_Nextion.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The HC12 interface is an optional module which allows for an HC12 to be connected
// to the base station allowing the text based protocol to be utilized by external
// devices via an HC12.
//#include "Config_HC12.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The LocoNet interface is an optional module, if you wish to interface with LocoNet
// you must uncomment the next line and edit the Config_LocoNet.h file to match your
// configuration.
//#include "Config_LocoNet.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The LCC interface is an optional module, by default the WiFi interface is available
// uncomment the line below if you wish to have a hardware CAN interface used as well.
//#include "Config_LCC.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The following pins are considered reserved pins by Espressif and should not
// generally be used by:
// GPIO6, GPIO7, GPIO8, GPIO9, GPIO10, GPIO11 -- used by flash
// GPIO0, GPIO2, GPIO5, GPIO12, GPIO15 -- used as bootstrap pins
//
// details of these usages can be found in section 2.2 of:
// https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf
//
// uncomment the following define to allow these pins to be used for sensors or
// outputs.

//#define ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS

/////////////////////////////////////////////////////////////////////////////////////

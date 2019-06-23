/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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
// The following pin is used for three RGB LEDs to indicate the Command Station
// status in a visual manner.
//
// LED 1: WiFi status (GREEN = connected, RED = disconnected, FLASHING GREEN = connecting, FLASHING YELLOW = AP not found, YELLOW = connection failed)
// LED 2: OPS track status (GREEN = ON, BLACK = OFF, RED = FAULT/SHORT, FLASHING RED = THERMAL FAULT)
// LED 3: PROG track status (GREEN = ON, BLACK = OFF, RED = FAULT/SHORT)
//
// Uncomment the following define to enable this functionality.
// 
#define STATUS_LED_DATA_PIN 22

// This defines how bright the Status LEDs will be, supported values are 0 (off) to 255 (full brightness). 
#define STATUS_LED_BRIGHTNESS 128

// Define the type of RGB LEDs being used, the following types are supported organized by
// the value to use in the STATUS_LED_TYPE define:
//
// WS281X_800: NeoPixel, WS2811, WS2812, WS2813, APA106
// WS281X_400: NeoPixel, WS2811, WS2812, WS2813, APA106
// SK6812: SK6812
// LC6812: LC6812
// 
// All testing is done with WS2811/APA106 style LEDs.
#define STATUS_LED_TYPE WS281X_800

// Defines the color order used by the Status LEDs
//
// Possible values: RGB (default), GRB, RGBW, GRBW, BRG, RBG
#define STATUS_LED_COLOR_ORDER RGB

/////////////////////////////////////////////////////////////////////////////////////

#define STATUS_LED_ENABLED true
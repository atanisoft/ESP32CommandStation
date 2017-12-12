/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2013-2016 Gregg E. Berman
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

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE PORT TO USE FOR ETHERNET COMMUNICATIONS INTERFACE
//
#define DCCPP_CLIENT_PORT 2560

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE HOSTNAME TO USE FOR WiFi CONNECTIONS AND mDNS BROADCASTS
//
#define HOSTNAME "DCCpp32"

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED BY MAIN/PROG MOTOR SHIELDS
//
// MAIN TRACK SIGNAL ENABLED PIN
#define SIGNAL_ENABLE_PIN_MAIN 25
// PROG TRACK SIGNAL ENABLED PIN
#define SIGNAL_ENABLE_PIN_PROG 23
// MAIN TRACK DIRECTION PIN
#define DIRECTION_MOTOR_CHANNEL_PIN_MAIN 19
// PROG TRACK DIRECTION PIN
#define DIRECTION_MOTOR_CHANNEL_PIN_PROG 18

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters
//
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//

//#define IP_ADDRESS { 192, 168, 1, 200 }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE INFO SCREEN Parameters
//
// BOTH OLED AND LCD SCREENS ARE SUPPORTED

// OLED SCREEN PARAMETERS
//#define INFO_SCREEN_OLED true
//#define OLED_CHIPSET SH1106
//#define INFO_SCREEN_OLED_I2C_ADDRESS 0x3C
//#define INFO_SCREEN_OLED_VERTICAL_FLIP false

// LCD SCREEN PARAMETERS
//#define INFO_SCREEN_LCD true
//#define INFO_SCREEN_LCD_I2C_ADDRESS 0x3F
//#define INFO_SCREEN_LCD_LINES 2
//#define INFO_SCREEN_LCD_COLUMNS 20

/////////////////////////////////////////////////////////////////////////////////////

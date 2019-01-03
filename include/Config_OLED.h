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
// DEFINE INFO SCREEN Parameters
//
// BOTH OLED AND LCD SCREENS ARE SUPPORTED

// For the Heltec or TTGO boards the below checks will automatically enable the OLED
// and configure it for use. To use this setup be sure to select the correct board
// type in platformio.ini:
// Heltec: heltec_wifi_kit_32 or heltec_wifi_lora_32
// TTGO: ttgo-lora32-v1
// D-duino-32: d-duino-32 (if available, if not use block below)
#if defined(OLED_SDA) && defined(OLED_SCL)
#define OLED_CHIPSET SH1306
#define INFO_SCREEN_OLED_I2C_ADDRESS 0x3C
#define INFO_SCREEN_SDA_PIN OLED_SDA
#define INFO_SCREEN_SCL_PIN OLED_SCL
#ifdef OLED_RST
#define INFO_SCREEN_RESET_PIN OLED_RST
#endif
#else
// For the D-duino-32/WeMos (LilyGo) OLED board the following
// settings should be used if d-duino-32 is not available in platformio.ini

//#define OLED_CHIPSET SH1306
//#define INFO_SCREEN_OLED_I2C_ADDRESS 0x3C
//#define INFO_SCREEN_SDA_PIN 5
//#define INFO_SCREEN_SCL_PIN 4

// If the ESP32 board does not use standard SDA/SCL pins as defined in pins_arduino.h
// uncomment the next two lines and define the pins that need to be used instead.
//#define INFO_SCREEN_SDA_PIN 23
//#define INFO_SCREEN_SCL_PIN 22

// If the OLED screen requires a reset pulse on startup, uncomment the following line
// to enable the reset pulse support.
//#define INFO_SCREEN_RESET_PIN 15

// OLED SUPPORTED CHIPSETS: SH1106, SH1306
#define OLED_CHIPSET SH1106
#define INFO_SCREEN_OLED_I2C_ADDRESS 0x3C
#define INFO_SCREEN_OLED_VERTICAL_FLIP false
#define INFO_SCREEN_OLED_LINES 5
#endif

/////////////////////////////////////////////////////////////////////////////////////

#define INFO_SCREEN_OLED true
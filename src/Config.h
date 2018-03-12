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
// DEFINE PORT TO USE FOR JMRI WiFi INTERFACE
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
// CURRENT SENSE PIN MAPPINGS:
// ADC1_CHANNEL_0 : 36
// ADC1_CHANNEL_1 : 37 -- NOT USABLE
// ADC1_CHANNEL_2 : 38 -- NOT USABLE
// ADC1_CHANNEL_3 : 39
// ADC1_CHANNEL_4 : 32
// ADC1_CHANNEL_5 : 33
// ADC1_CHANNEL_6 : 34
// ADC1_CHANNEL_7 : 35
//
// MOST ESP32 BOARDS DO NOT EXPOSE GPIO38 so ADC1_CHANNEL_2 MAY NOT BE USABLE.

// MAIN TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_MAIN "MAIN"
// MAIN TRACK NOTORBOARD ENABLED PIN
#define MOTORBOARD_ENABLE_PIN_MAIN 25
// MAIN TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define MOTORBOARD_CURRENT_SENSE_MAIN ADC1_CHANNEL_0
// MAIN TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define MOTORBOARD_TYPE_MAIN MOTOR_BOARD_TYPE::ARDUINO_SHIELD

// PROG TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_PROG "PROG"
// PROG TRACK NOTORBOARD ENABLED PIN
#define MOTORBOARD_ENABLE_PIN_PROG 23
// PROG TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define MOTORBOARD_CURRENT_SENSE_PROG ADC1_CHANNEL_3
// PROG TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define MOTORBOARD_TYPE_PROG MOTOR_BOARD_TYPE::ARDUINO_SHIELD

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR DCC SIGNAL GENERATION
//
// OPERATIONS TRACK DCC SIGNAL PIN
#define DCC_SIGNAL_PIN_OPERATIONS 19
// PROGRAMMING TRACK DCC SIGNAL PIN
#define DCC_SIGNAL_PIN_PROGRAMMING 18

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters
//
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

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
// DEFINE WHICH PINS ARE USED FOR S88 INTERFACE
//
// NOTE: The S88 Bus will require a data pin in addition to the above pins that
// are common to all S88 Busses.
//

//#define S88_ENABLED true
//#define S88_CLOCK_PIN 17
//#define S88_RESET_PIN 16
//#define S88_LOAD_PIN 27

// S88 sensors are dynamically assigned based on the BUS ID * S88_MAX_SENSORS_PER_BUS + S88_FIRST_SENSOR
// This define allows shifting the S88 Sensors to start at a default value, you
// can start them at zero or as below S88_MAX_SENSORS_PER_BUS (default).
// S88_MAX_SENSORS_PER_BUS is defined as 512.
//#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE INFO SCREEN Parameters
//
// BOTH OLED AND LCD SCREENS ARE SUPPORTED

// If the ESP32 board does not use standard SDA/SCL pins as defined in pins_arduino.h
// uncomment the next two lines and define the pins that need to be used instead.
//#define INFO_SCREEN_SDA_PIN SDA
//#define INFO_SCREEN_SCL_PIN SCL

// If your board is a Heltec variant, uncomment the following line to enable the
// OLED reset support for this board.
//#define INFO_SCREEN_RESET_PIN 15

// OLED SCREEN PARAMETERS
//#define INFO_SCREEN_OLED true
// OLED SUPPORTED CHIPSETS: SH1106, SH1306
//#define OLED_CHIPSET SH1106
//#define INFO_SCREEN_OLED_I2C_ADDRESS 0x3C
//#define INFO_SCREEN_OLED_VERTICAL_FLIP false
//#define INFO_SCREEN_OLED_LINES 5

// LCD SCREEN PARAMETERS
//#define INFO_SCREEN_LCD true
//#define INFO_SCREEN_LCD_I2C_ADDRESS 0x27
//#define INFO_SCREEN_LCD_LINES 4
//#define INFO_SCREEN_LCD_COLUMNS 20

/////////////////////////////////////////////////////////////////////////////////////
//
// Remote Sensors are enabled by default, the following parameters can be used
// to control their behavior.

//#define SCAN_REMOTE_SENSORS_ON_STARTUP true
//#define REMOTE_SENSORS_PREFIX "sensor"
//#define REMOTE_SENSORS_DECAY 60000
//#define REMOTE_SENSORS_FIRST_SENSOR 100

/////////////////////////////////////////////////////////////////////////////////////

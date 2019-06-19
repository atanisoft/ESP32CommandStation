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

#pragma once

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "1.3.0"

#include <Arduino.h>
#include <algorithm>
#include <functional>
#include <StringArray.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>

#include <utils/logging.h>
#include <utils/StringPrintf.hxx>

#include "Config.h"

/////////////////////////////////////////////////////////////////////////////////////
//
// The following parameters define how many preamble bits will be transmitted as part
// of the DCC packet to the track. For some older sound decodes it may be necessary
// to increase from 22 bits on the PROG track to 30 or even 40.
//
// The maximum number of preamble bits is 50. For OPS the minimum to send is 11 but
// 16 is recommended for RailCom support.

#define OPS_TRACK_PREAMBLE_BITS 16
#define PROG_TRACK_PREAMBLE_BITS 22

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR OPS RAILCOM DETECTION
//
#define OPS_BRAKE_ENABLE_PIN NOT_A_PIN
#define OPS_RAILCOM_ENABLE_PIN NOT_A_PIN
#define OPS_RAILCOM_SHORT_PIN NOT_A_PIN
#define OPS_RAILCOM_UART 2
#define OPS_RAILCOM_UART_RX_PIN NOT_A_PIN

#ifndef STATUS_LED_ENABLED
#define STATUS_LED_ENABLED false
#endif

#ifndef OPS_TRACK_PREAMBLE_BITS
#define OPS_TRACK_PREAMBLE_BITS 16
#endif

#ifndef PROG_TRACK_PREAMBLE_BITS
#define PROG_TRACK_PREAMBLE_BITS 22
#endif

#if OPS_TRACK_PREAMBLE_BITS < 11
#error "OPS_TRACK_PREAMBLE_BITS is too low, a minimum of 11 bits must be transmitted for the DCC decoder to accept the packets."
#endif

#if OPS_TRACK_PREAMBLE_BITS > 20
#error "OPS_TRACK_PREAMBLE_BITS is too high. The OPS track only supports up to 20 preamble bits."
#endif

#if PROG_TRACK_PREAMBLE_BITS < 22
#error "PROG_TRACK_PREAMBLE_BITS is too low, a minimum of 22 bits must be transmitted for reliability on the PROG track."
#endif

#if OPS_TRACK_PREAMBLE_BITS > 50
#error "PROG_TRACK_PREAMBLE_BITS is too high. The PROG track only supports up to 50 preamble bits."
#endif

// initialize default values for various pre-compiler checks to simplify logic in a lot of places
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD) || (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_ENABLED true
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD)
#define INFO_SCREEN_OLED false
#endif
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_LCD false
#endif
#else
#define INFO_SCREEN_ENABLED false
#define INFO_SCREEN_LCD false
#define INFO_SCREEN_OLED false
#endif

#ifndef NEXTION_ENABLED
#define NEXTION_ENABLED false
#endif

#ifndef HC12_RADIO_ENABLED
#define HC12_RADIO_ENABLED false
#endif

#ifndef LCC_ENABLED
#define LCC_ENABLED false
#endif

#ifndef LCC_FORCE_FACTORY_RESET_ON_STARTUP 
#define LCC_FORCE_FACTORY_RESET_ON_STARTUP false
#endif

#ifndef LOCONET_ENABLED
#define LOCONET_ENABLED false
#endif

#ifndef S88_ENABLED
#define S88_ENABLED false
#endif

#ifndef ENERGIZE_OPS_TRACK_ON_STARTUP
#define ENERGIZE_OPS_TRACK_ON_STARTUP false
#endif

#ifndef LOCONET_INVERTED_LOGIC
#define LOCONET_INVERTED_LOGIC false
#endif

#ifndef LOCONET_ENABLE_RX_PIN_PULLUP
#define LOCONET_ENABLE_RX_PIN_PULLUP false
#endif

/////////////////////////////////////////////////////////////////////////////////////
// S88 Maximum sensors per bus.
/////////////////////////////////////////////////////////////////////////////////////
constexpr uint16_t S88_MAX_SENSORS_PER_BUS = 512;
#ifndef S88_FIRST_SENSOR
#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS
#endif

#include "JsonConstants.h"
#include "ConfigurationManager.h"
#include "WiFiInterface.h"
#include "InfoScreen.h"
#include "DCCppProtocol.h"
#include "DCCSignalGenerator.h"
#include "DCCSignalGenerator_RMT.h"
#include "DCCProgrammer.h"
#include "MotorBoard.h"
#include "Sensors.h"
#include "Locomotive.h"
#include "Outputs.h"
#include "NextionInterface.h"
#if LCC_ENABLED
#include <OpenMRNLite.h>
#include "LCCInterface.h"
#endif

extern std::vector<uint8_t> restrictedPins;

#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void esp32_restart();

extern bool otaComplete;
extern bool otaInProgress;

#if STATUS_LED_ENABLED

enum STATUS_LED_COLOR {
    LED_OFF,
    LED_RED,
    LED_GREEN,
    LED_YELLOW,
    LED_RED_BLINK,
    LED_GREEN_BLINK,
    LED_YELLOW_BLINK,
};

enum STATUS_LED {
    WIFI_LED,
    OPS_LED,
    PROG_LED,
    MAX_STATUS_LED
};

void initStatusLEDs();
void setStatusLED(const STATUS_LED, const STATUS_LED_COLOR);
#endif


#define MUTEX_LOCK(mutex)    do {} while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK(mutex)  xSemaphoreGive(mutex)

/////////////////////////////////////////////////////////////////////////////////////
// Ensure SSID and PASSWORD are provided.
/////////////////////////////////////////////////////////////////////////////////////
#if !defined(SSID_NAME) || !defined(SSID_PASSWORD)
#error "Invalid Configuration detected, Config_WiFi.h is a mandatory module."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Ensure the required h-bridge parameters are specified and not overlapping.
/////////////////////////////////////////////////////////////////////////////////////
#if !defined(MOTORBOARD_NAME_OPS) || !defined(MOTORBOARD_ENABLE_PIN_OPS) || \
  !defined(MOTORBOARD_CURRENT_SENSE_OPS) || !defined(MOTORBOARD_TYPE_OPS) || \
  !defined(MOTORBOARD_NAME_PROG) || !defined(MOTORBOARD_ENABLE_PIN_PROG) || \
  !defined(MOTORBOARD_CURRENT_SENSE_PROG) || !defined(MOTORBOARD_TYPE_PROG) || \
  !defined(DCC_SIGNAL_PIN_OPERATIONS) || !defined(DCC_SIGNAL_PIN_PROGRAMMING)
#error "Invalid Configuration detected, Config_MotorBoard.h is a mandatory module."
#endif

#if MOTORBOARD_ENABLE_PIN_OPS == MOTORBOARD_ENABLE_PIN_PROG
#error "Invalid Configuration detected, MOTORBOARD_ENABLE_PIN_OPS and MOTORBOARD_ENABLE_PIN_PROG must be unique."
#endif

#if DCC_SIGNAL_PIN_OPERATIONS == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == MOTORBOARD_ENABLE_PIN_OPS
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and MOTORBOARD_ENABLE_PIN_OPS must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == MOTORBOARD_ENABLE_PIN_PROG
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and MOTORBOARD_ENABLE_PIN_PROG must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == DCC_SIGNAL_PIN_OPERATIONS
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and DCC_SIGNAL_PIN_OPERATIONS must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Ensure either OLED or LCD display is active and not both.
/////////////////////////////////////////////////////////////////////////////////////
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED && defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#error "Invalid Configuration detected, it is not supported to include both OLED and LCD support."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Nextion interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if NEXTION_ENABLED
  #if NEXTION_UART_RX_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, NEXTION_UART_RX_PIN and NEXTION_UART_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_UART_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_UART_TX_PIN must be unique."
  #endif
  #if HC12_RADIO_ENABLED
    #if NEXTION_UART_NUM == HC12_UART_NUM
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same TX Pin."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if NEXTION_UART_NUM == LOCONET_UART
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same TX Pin."
    #endif
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// HC12 Radio interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if HC12_RADIO_ENABLED
  #if HC12_RX_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, HC12_RX_PIN and HC12_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and HC12_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LOCONET_UART == HC12_UART_NUM
    #error "Invalid Configuration detected, the LocoNet and HC12 can not share the UART interface."
    #endif
    #if LOCONET_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and HC12_RX_PIN must be unique."
    #endif
    #if LOCONET_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and HC12_TX_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// LocoNet interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if LOCONET_ENABLED
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LOCONET_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LOCONET_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// LCC interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if LCC_ENABLED
  #if LCC_CAN_RX_PIN != NOT_A_PIN
    #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LCC_CAN_RX_PIN must be unique."
    #endif
    #if S88_ENABLED
      #if S88_CLOCK_PIN == LCC_CAN_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
      #endif
      #if S88_CLOCK_PIN == LCC_CAN_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
      #endif
      #if S88_RESET_PIN == LCC_CAN_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
      #endif
    #endif
    #if LOCONET_ENABLED
      #if LCC_CAN_RX_PIN == LOCONET_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LOCONET_RX_PIN must be unique."
      #endif
      #if LCC_CAN_RX_PIN == LOCONET_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LOCONET_TX_PIN must be unique."
      #endif
    #endif
    #if NEXTION_ENABLED
      #if LCC_CAN_RX_PIN == NEXTION_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and NEXTION_RX_PIN must be unique."
      #endif
      #if LCC_CAN_RX_PIN == NEXTION_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_RX_PIN and NEXTION_TX_PIN must be unique."
      #endif
    #endif
  #endif
  #if LCC_CAN_TX_PIN != NOT_A_PIN
    #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LCC_CAN_TX_PIN must be unique."
    #endif
    #if S88_ENABLED
      #if S88_CLOCK_PIN == LCC_CAN_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
      #endif
      #if S88_CLOCK_PIN == LCC_CAN_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
      #endif
      #if S88_RESET_PIN == LCC_CAN_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
      #endif
    #endif
    #if LOCONET_ENABLED
      #if LCC_CAN_TX_PIN == LOCONET_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and LOCONET_RX_PIN must be unique."
      #endif
      #if LCC_CAN_TX_PIN == LOCONET_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and LOCONET_TX_PIN must be unique."
      #endif
    #endif
    #if NEXTION_ENABLED
      #if LCC_CAN_TX_PIN == NEXTION_RX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and NEXTION_RX_PIN must be unique."
      #endif
      #if LCC_CAN_TX_PIN == NEXTION_TX_PIN
      #error "Invalid Configuration detected, LCC_CAN_TX_PIN and NEXTION_TX_PIN must be unique."
      #endif
    #endif
  #endif
  #if LCC_CAN_RX_PIN == LCC_CAN_TX_PIN && LCC_CAN_RX_PIN != NOT_A_PIN && LCC_CAN_TX_PIN != NOT_A_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LCC_CAN_TX_PIN must be unique."
  #endif
#endif

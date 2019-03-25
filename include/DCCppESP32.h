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

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "1.2.3"

/////////////////////////////////////////////////////////////////////////////////////
// PERFORMANCE CONFIGURATION
/////////////////////////////////////////////////////////////////////////////////////

// Maximum clients connected to
#define MAX_DCCPP_CLIENTS 10

/////////////////////////////////////////////////////////////////////////////////////
// S88 Timing values (in microseconds)
/////////////////////////////////////////////////////////////////////////////////////
#define S88_SENSOR_LOAD_PRE_CLOCK_TIME 50
#define S88_SENSOR_LOAD_POST_RESET_TIME 50
#define S88_SENSOR_CLOCK_PULSE_TIME 50
#define S88_SENSOR_CLOCK_PRE_RESET_TIME 50
#define S88_SENSOR_RESET_PULSE_TIME 50
#define S88_SENSOR_READ_TIME 25
#define S88_MAX_SENSORS_PER_BUS 512

#include <Arduino.h>
#include <algorithm>
#include <functional>
#include <StringArray.h>
#include <ArduinoJson.h>
#include <esp32-hal-log.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>

#include "Config.h"

// initialize default values for various pre-compiler checks to simplify logic in a lot of places
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD) || (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_ENABLED true
#else
#define INFO_SCREEN_ENABLED false
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

#ifndef LOCONET_ENABLED
#define LOCONET_ENABLED false
#endif

#ifndef S88_ENABLED
#define S88_ENABLED false
#endif

#ifndef ENERGIZE_OPS_TRACK_ON_STARTUP
#define ENERGIZE_OPS_TRACK_ON_STARTUP false
#endif

#include "ConfigurationManager.h"
#include "WiFiInterface.h"
#include "InfoScreen.h"
#include "DCCppProtocol.h"
#include "DCCSignalGenerator.h"
#include "DCCSignalGenerator_Timer.h"
#if 0
#include "DCCSignalGenerator_RMT.h"
#endif
#include "DCCProgrammer.h"
#include "MotorBoard.h"
#include "Sensors.h"
#include "Locomotive.h"
#include "Outputs.h"
#include "NextionInterface.h"
#if LCC_ENABLED
#include <OpenMRN.h>
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

#define MUTEX_LOCK(mutex)    do {} while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK(mutex)  xSemaphoreGive(mutex)

// Perform some basic configuration validations to prevent common mistakes

#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "Invalid Configuration detected, Config_WiFi.h is a mandatory module."
#endif

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

#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED && defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#error "Invalid Configuration detected, it is not supported to include both OLED and LCD support."
#endif

#if NEXTION_ENABLED
  #if NEXTION_RX_PIN == NEXTION_TX_PIN
  #error "Invalid Configuration detected, NEXTION_RX_PIN and NEXTION_TX_PIN must be unique."
  #endif
  #if HC12_RADIO_ENABLED
    #if NEXTION_UART_NUM == HC12_UART_NUM
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the UART interface."
    #endif
    #if NEXTION_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same RX Pin."
    #endif
    #if NEXTION_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same TX Pin."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if NEXTION_UART_NUM == LOCONET_UART
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the UART interface."
    #endif
    #if NEXTION_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same RX Pin."
    #endif
    #if NEXTION_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same TX Pin."
    #endif
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

#if HC12_RADIO_ENABLED
  #if HC12_RX_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, HC12_RX_PIN and HC12_TX_PIN must be unique."
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

#if LOCONET_ENABLED
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

#if LCC_ENABLED
  #if LCC_CAN_RX_PIN != -1
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
  #if LCC_CAN_TX_PIN != -1
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
  #if LCC_CAN_RX_PIN == LCC_CAN_TX_PIN && LCC_CAN_RX_PIN != -1 && LCC_CAN_TX_PIN != -1
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LCC_CAN_TX_PIN must be unique."
  #endif
#endif

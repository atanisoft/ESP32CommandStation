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

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "1.2.0"

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

/////////////////////////////////////////////////////////////////////////////////////
// SET WHETHER TO SHOW PACKETS - DIAGNOSTIC MODE ONLY
/////////////////////////////////////////////////////////////////////////////////////

// If SHOW_DCC_PACKETS is set to 1, the DCC++ BASE STATION will return the
// DCC packet contents in the following format:

//    <* B1 B2 ... Bn CSUM / NUM_BITS / REPEAT>
//
//    B1: the first hexidecimal byte of the DCC packet
//    B2: the second hexidecimal byte of the DCC packet
//    Bn: the nth hexidecimal byte of the DCC packet
//    CSUM: a checksum byte that is required to be the final byte in any DCC packet
//    NUM_BITS: the number of bits in the DCC packet
//    REPEAT: the number of times the DCC packet was re-transmitted to the tracks after its iniital transmission

// Set to one to enable printing of all DCC packets as described above
#define SHOW_DCC_PACKETS  0

// set to zero to disable diagnostic logging of packets in the signal generator queue
// WARNING: Enabling this can cause the ESP32 to crash due to Serial buffer overflow
#define DEBUG_SIGNAL_GENERATOR 0

// set to zero to disable diagnostic logging of packets in the ADC Sampling code
// WARNING: Enabling this can cause the ESP32 to crash due to Serial buffer overflow
#define DEBUG_ADC_SAMPLING 0

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

#include "ConfigurationManager.h"
#include "WiFiInterface.h"
#include "InfoScreen.h"
#include "DCCppProtocol.h"
#include "SignalGenerator.h"
#include "MotorBoard.h"
#include "Sensors.h"
#include "Locomotive.h"
#include "Outputs.h"
#include "NextionInterface.h"
#if defined(LCC_ENABLED) && LCC_ENABLED
#include "LCCInterface.h"
#endif

extern std::vector<uint8_t> restrictedPins;

#if defined(LOCONET_ENABLED) && LOCONET_ENABLED
#include <LocoNet2/LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void esp32_restart();

#define MUTEX_LOCK(mutex)    do {} while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK(mutex)  xSemaphoreGive(mutex)

// Perform some basic configuration validations to prevent common mistakes

#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "Invalid Configuration detected, Config_WiFi.h is a mandatory module."
#endif

#if !defined(MOTORBOARD_NAME_MAIN) || !defined(MOTORBOARD_ENABLE_PIN_MAIN) || \
  !defined(MOTORBOARD_CURRENT_SENSE_MAIN) || !defined(MOTORBOARD_TYPE_MAIN) || \
  !defined(MOTORBOARD_NAME_PROG) || !defined(MOTORBOARD_ENABLE_PIN_PROG) || \
  !defined(MOTORBOARD_CURRENT_SENSE_PROG) || !defined(MOTORBOARD_TYPE_PROG) || \
  !defined(DCC_SIGNAL_PIN_OPERATIONS) || !defined(DCC_SIGNAL_PIN_PROGRAMMING)
#error "Invalid Configuration detected, Config_MotorBoard.h is a mandatory module."
#endif

#if MOTORBOARD_ENABLE_PIN_MAIN == MOTORBOARD_ENABLE_PIN_PROG
#error "Invalid Configuration detected, MOTORBOARD_ENABLE_PIN_MAIN and MOTORBOARD_ENABLE_PIN_PROG must be unique."
#endif

#if DCC_SIGNAL_PIN_OPERATIONS == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

#if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED) && DCC_SIGNAL_PIN_OPERATIONS_INVERTED == DCC_SIGNAL_PIN_OPERATIONS
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and DCC_SIGNAL_PIN_OPERATIONS must be unique."
#endif

#if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED) && DCC_SIGNAL_PIN_OPERATIONS_INVERTED == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

#if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED) && DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == DCC_SIGNAL_PIN_OPERATIONS
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and DCC_SIGNAL_PIN_OPERATIONS must be unique."
#endif

#if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED) && DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED && defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#error "Invalid Configuration detected, it is not supported to include both OLED and LCD support."
#endif

#if defined(NEXTION_ENABLED) && NEXTION_ENABLED
  #if NEXTION_RX_PIN == NEXTION_TX_PIN
  #error "Invalid Configuration detected, NEXTION_RX_PIN and NEXTION_TX_PIN must be unique."
  #endif
  #if defined(HC12_RADIO_ENABLED) && HC12_RADIO_ENABLED
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
  #if defined(LOCONET_ENABLED) && LOCONET_ENABLED
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
  #if defined(S88_ENABLED) && S88_ENABLED
    #if S88_CLOCK_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_CLOCK_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the Nextion TX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the Nextion TX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88 Load Pin must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, the Nextion TX Pin and S88 Load Pin must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == NEXTION_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and NEXTION_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == NEXTION_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and NEXTION_TX_PIN must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == NEXTION_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and NEXTION_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == NEXTION_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and NEXTION_TX_PIN must be unique."
    #endif
  #endif
#endif

#if defined(HC12_RADIO_ENABLED) && HC12_RADIO_ENABLED
  #if HC12_RX_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, HC12_RX_PIN and HC12_TX_PIN must be unique."
  #endif
  #if defined(S88_ENABLED) && S88_ENABLED
    #if S88_CLOCK_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12 RX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_CLOCK_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12 TX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_RESET_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12 RX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_RESET_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12 TX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12 RX Pin and S88 Load Pin must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12 TX Pin and S88 Load Pin must be unique."
    #endif
  #endif
  #if defined(LOCONET_ENABLED) && LOCONET_ENABLED
    #if LOCONET_UART == HC12_UART_NUM
    #error "Invalid Configuration detected, the LocoNet and HC12 can not share the UART interface."
    #endif
    #if LOCONET_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the LocoNet RX Pin and HC12 RX Pin must be unique."
    #endif
    #if LOCONET_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the LocoNet TX Pin and HC12 TX Pin must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == HC12_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and HC12_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == HC12_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and HC12_TX_PIN must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == HC12_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and HC12_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == HC12_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and HC12_TX_PIN must be unique."
    #endif
  #endif
#endif

#if defined(LOCONET_ENABLED) && LOCONET_ENABLED
  #if defined(S88_ENABLED) && S88_ENABLED
    #if S88_CLOCK_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LocoNet RX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_CLOCK_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LocoNet TX Pin and S88 Clock Pin must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LocoNet RX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LocoNet TX Pin and S88 Reset Pin must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LocoNet RX Pin and S88 Load Pin must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LocoNet TX Pin and S88 Load Pin must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == LOCONET_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and LOCONET_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_OPERATIONS_INVERTED == LOCONET_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS_INVERTED and LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == LOCONET_RX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and LOCONET_RX_PIN must be unique."
    #endif
    #if DCC_SIGNAL_PIN_PROGRAMMING_INVERTED == LOCONET_TX_PIN
    #error "Invalid Configuration detected, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED and LOCONET_TX_PIN must be unique."
    #endif
  #endif
#endif

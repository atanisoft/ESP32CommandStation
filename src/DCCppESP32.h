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
#include <SD.h>
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

extern std::vector<uint8_t> restrictedPins;

#if defined(LOCONET_ENABLED) && LOCONET_ENABLED
#include <LocoNetESP32.h>
extern LocoNetESP32 locoNet;
#endif

void esp32_restart();

#define MUTEX_LOCK(mutex)    do {} while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK(mutex)  xSemaphoreGive(mutex)

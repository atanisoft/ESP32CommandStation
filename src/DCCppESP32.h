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

#define VERSION "1.0.7"

/////////////////////////////////////////////////////////////////////////////////////
// PERFORMANCE CONFIGURATION
/////////////////////////////////////////////////////////////////////////////////////

// Maximum clients connected to
#define MAX_DCCPP_CLIENTS 10

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

#include <Arduino.h>
#include <Preferences.h>
#include <algorithm>
#include <functional>
#include <StringArray.h>
#include <esp32-hal-log.h>

#include "Config.h"
#include "WiFiInterface.h"
#include "InfoScreen.h"
#include "DCCppProtocol.h"
#include "SignalGenerator.h"

extern Preferences configStore;
extern WiFiInterface wifiInterface;

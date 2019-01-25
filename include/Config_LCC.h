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

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR LCC INTERFACE
//
// NOTE: The LCC interface supports both WiFi and a hardware CAN connection. By default
// only the WiFi interface is enabled. To enable the CAN interface set the RX and TX
// pin definitions below to valid pin numbers.
//

// This is the ESP32 pin connected to the SN6565HVD23x/MCP2551 R (RX) pin.
// Recommended pin: 4
#define LCC_CAN_RX_PIN 4

// This is the ESP32 pin connected to the SN6565HVD23x/MCP2551 D (TX) pin.
// Recommended pin: 5
#define LCC_CAN_TX_PIN 5

#define LCC_HARDWARE_CAN_ENABLED true
/////////////////////////////////////////////////////////////////////////////////////

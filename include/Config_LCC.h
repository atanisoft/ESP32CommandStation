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
//
// DEFINE WHICH PINS ARE USED FOR LCC INTERFACE
//
// NOTE: The LCC interface supports both WiFi and a hardware CAN connection. By default
// only the WiFi interface is enabled. To enable the CAN interface set the RX and TX
// pin definitions below to valid pin numbers.
//

// This is the LCC node identifier for the DCC++ESP32 Command Station. It is
// recommended, but not required, to request your own ID range via:
// https://registry.openlcb.org/requestuniqueidrange
//
// The format below must start with 0x and have exactly 12 characters
// the first 10 will be the unique ID range and the last two the unique
// node ID in the range. Using 00 or 01 is recommended for the command station
// for easy identification in the JMRI LCC node list.
#define LCC_NODE_ID 0x050101013F00

// This is the ESP32 pin connected to the SN6565HVD23x/MCP2551 R (RX) pin.
// Recommended pin: 4, 16, 21.
// to disable the CAN interface set this to NOT_A_PIN
#define LCC_CAN_RX_PIN NOT_A_PIN

// This is the ESP32 pin connected to the SN6565HVD23x/MCP2551 D (TX) pin.
// Recommended pin: 5, 17, 22.
// to disable the CAN interface set this to NOT_A_PIN
#define LCC_CAN_TX_PIN NOT_A_PIN

// This defines where on the filesystem LCC configuration data will be persisted.
#define LCC_CONFIG_DIR "/LCC"

// This is where the CDI will be persisted on the filesystem and sent from on-demand
// when the NODE CDI information is requested.
#define LCC_CDI_FILE "/LCC/cdi.xml"

// This is where the NODE persistent configuration will be stored on the filesystem.
#define LCC_CONFIG_FILE "/LCC/config"

// This defines which filesystem the LCC interface will use. Currently only SPIFFS is
// supported but an SD card will be supported in the future.
#define LCC_USE_SPIFFS true

/////////////////////////////////////////////////////////////////////////////////////

#define LCC_ENABLED true
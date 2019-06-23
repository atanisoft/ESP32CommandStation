/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 Mike Dunston

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
// The Nextion is an interface to the command station that provides a connected
// throttle, turnout control, programming track and general command station status.

// This is the hardware UART that the Nextion Interface will utilize on the ESP32
//
// Default is to use UART 2 but UART 1 is also available.
#define NEXTION_UART_NUM 2

// This is the BAUD rate which the ESP32 will communicate with the Nextion screen.
//
// This must match the "bauds=XXX" value set in the Nextion HMI initialization event.
#define NEXTION_UART_BAUD 115200

// This is the ESP32 UART RX pin, this needs to be connected to the Nextion TX pin,
// this is typically the blue wire.
#define NEXTION_UART_RX_PIN 14

// This is the ESP32 UART TX pin, this needs to be connected to the Nextion RX pin,
// this is typically the yellow wire.
#define NEXTION_UART_TX_PIN 27

/////////////////////////////////////////////////////////////////////////////////////

#define NEXTION_ENABLED true
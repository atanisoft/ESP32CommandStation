/**********************************************************************
DCC COMMAND STATION FOR ESP32

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

#define NEXTION_UART_NUM 2
#define NEXTION_UART_BAUD 115200
#define NEXTION_RX_PIN 14
#define NEXTION_TX_PIN 27

/////////////////////////////////////////////////////////////////////////////////////

#define NEXTION_ENABLED true
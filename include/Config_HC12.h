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
// The HC12 is a radio receiver that was previously used by some throttles to
// wirelessly send packet data to the command station. Uncomment the define below
// to enable this functionality.

#define HC12_RADIO_BAUD 19200
#define HC12_UART_NUM 1
#define HC12_RX_PIN 16
#define HC12_TX_PIN 17

/////////////////////////////////////////////////////////////////////////////////////

#define HC12_RADIO_ENABLED true

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
// The LocoNet interface allows using many of the available LocoNet sensors, throttles,
// accessories, etc with this base station. It is still under active development, if
// you find a device that does not work please let us know.
//

#define LOCONET_RX_PIN 16
#define LOCONET_TX_PIN 17
#define LOCONET_UART 1

/////////////////////////////////////////////////////////////////////////////////////

#define LOCONET_ENABLED true

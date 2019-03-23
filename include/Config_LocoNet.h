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
// accessories, etc with this command station. It is still under active development, if
// you find a device that does not work please let us know.
//

#define LOCONET_RX_PIN 16
#define LOCONET_TX_PIN 17
#define LOCONET_UART 1

// If you are using the LM311 circuit as defined in the docs/loconet/LoocNetInterface.png
// file it is necessary to invert the logic. If you are using a different circuit you can
// reset this parameter to false or comment the next line which will have the same effect.
#define LOCONET_INVERTED_LOGIC true

// If you are using the LM311 circuit as defined in the docs/loconet/LoocNetInterface.png
// file and do not have a resistor connected between 3v3 (VCC) and the LOCONET_RX_PIN it
// is necessary to enable the internal pull up for the RX pin to ensure proper logic. If
// you have an external pull up resistor comment out this option.
#define LOCONET_ENABLE_RX_PIN_PULLUP true

/////////////////////////////////////////////////////////////////////////////////////

#define LOCONET_ENABLED true

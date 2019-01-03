/**********************************************************************
DCC++ BASE STATION FOR ESP32

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
// Define the pins for an SD Card Module. This is used as part of the Locomotive
// Roster and dynamic configuration support.
//

// if SD_CARD_USE_FSPI is enabled the MOSI, MISO and CLK pins are shared with
// the built in flash, only the SS pin is required to be defined. All others are
// defined as:
// SD_CARD_MOSI_PIN: SD1 (8)
// SD_CARD_MISO_PIN: SD0 (7)
// SD_CARD_CLK_PIN: CLK (6)
#define SD_CARD_USE_FSPI true

// The SD_CARD_SS_PIN should be connected to the SD Card breakout module CS or
// SS pin, whichever is available
#define SD_CARD_SS_PIN 5
#define SD_CARD_MOSI_PIN 8
#define SD_CARD_MISO_PIN 7
#define SD_CARD_CLK_PIN 6
/////////////////////////////////////////////////////////////////////////////////////

#define SD_CARD_ENABLED true
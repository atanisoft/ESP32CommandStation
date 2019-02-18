/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2019 Mike Dunston

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
#pragma once

#include <stdint.h>

enum CV_NAMES {
  SHORT_ADDRESS=1,
  DECODER_VERSION=7,
  DECODER_MANUFACTURER=8,
  ACCESSORY_DECODER_MSB_ADDRESS=9,
  LONG_ADDRESS_MSB_ADDRESS=17,
  LONG_ADDRESS_LSB_ADDRESS=18,
  CONSIST_ADDRESS=19,
  CONSIST_FUNCTION_CONTROL_F1_F8=21,
  CONSIST_FUNCTION_CONTROL_FL_F9_F12=22,
  DECODER_CONFIG=29
};

static constexpr uint8_t CONSIST_ADDRESS_REVERSED_ORIENTATION = 0x80;
static constexpr uint8_t CONSIST_ADDRESS_NO_ADDRESS = 0x00;

enum DECODER_CONFIG_BITS {
  LOCOMOTIVE_DIRECTION=0,
  FL_CONTROLLED_BY_SPEED=1,
  POWER_CONVERSION=2,
  BIDIRECTIONAL_COMMUNICATION=3,
  SPEED_TABLE=4,
  SHORT_OR_LONG_ADDRESS=5,
  ACCESSORY_ADDRESS_MODE=6,
  DECODER_TYPE=7
};

enum CONSIST_FUNCTION_CONTROL_F1_F8_BITS {
  F1_BIT=0,
  F2_BIT=1,
  F3_BIT=2,
  F4_BIT=3,
  F5_BIT=4,
  F6_BIT=5,
  F7_BIT=6,
  F8_BIT=7
};

enum CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS {
  FL_BIT=0,
  F9_BIT=1,
  F10_BIT=2,
  F11_BIT=3,
  F12_BIT=4
};


extern bool progTrackBusy;

bool enterProgrammingMode();
void leaveProgrammingMode();
int16_t readCV(const uint16_t);
bool writeProgCVByte(const uint16_t, const uint8_t);
bool writeProgCVBit(const uint16_t, const uint8_t, const bool);
void writeOpsCVByte(const uint16_t, const uint16_t, const uint8_t);
void writeOpsCVBit(const uint16_t, const uint16_t, const uint8_t, const bool);

/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017,2018 Mike Dunston

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

#include <Arduino.h>
#include <driver/timer.h>
#include <vector>
#include <queue>
#include <stack>
#include <driver/adc.h>

#define MAX_BYTES_IN_PACKET 10

struct Packet {
  uint8_t buffer[MAX_BYTES_IN_PACKET];
  uint8_t numberOfBits;
  uint8_t numberOfRepeats;
  uint8_t currentBit;
}; // Packet

struct SignalGenerator {
  template<int signalGenerator>
  void configureSignal(String, uint8_t, uint16_t);
  template<int signalGenerator>
  void startSignal();
  template<int signalGenerator>
  void stopSignal();

  bool IRAM_ATTR getNextBitToSend();
  void loadPacket(std::vector<uint8_t>, int, bool=false);
  void waitForQueueEmpty();
  bool isQueueEmpty();
  bool isEnabled();

  hw_timer_t *_fullCycleTimer;
  hw_timer_t *_pulseTimer;
  String _name;
  uint8_t _directionPin;
  int _currentMonitorPin;
  bool _enabled;
  std::queue<Packet *> _toSend;
  std::queue<Packet *> _availablePackets;
  Packet *_currentPacket;
  // pre-encoded idle packet that gets sent when the _toSend queue is empty.
  Packet _idlePacket = {
    { 0xFF, 0xFF, 0xFD, 0xFE, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00 }, // packet bytes
    49, // number of bits
    0, // number of repeats
    0 // current bit
  };
  portMUX_TYPE _sendQueueMUX = portMUX_INITIALIZER_UNLOCKED;
};

extern SignalGenerator dccSignal[2];
void configureDCCSignalGenerators();
void startDCCSignalGenerators();
void stopDCCSignalGenerators();
bool isDCCSignalEnabled();
void sendDCCEmergencyStop();

int16_t readCV(const uint16_t);
bool writeProgCVByte(const uint16_t, const uint8_t);
bool writeProgCVBit(const uint16_t, const uint8_t, const bool);
void writeOpsCVByte(const uint16_t, const uint16_t, const uint8_t);
void writeOpsCVBit(const uint16_t, const uint16_t, const uint8_t, const bool);

#define DCC_SIGNAL_OPERATIONS 0
#define DCC_SIGNAL_PROGRAMMING 1
#define MAX_DCC_SIGNAL_GENERATORS 2

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

const uint8_t CONSIST_ADDRESS_REVERSED_ORIENTATION = 0x80;
const uint8_t CONSIST_ADDRESS_NO_ADDRESS = 0x00;

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

/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

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
#ifndef _SIGNALGENERATOR_H_
#define _SIGNALGENERATOR_H_

#include <Arduino.h>
#include <driver/timer.h>
#include <vector>
#include <queue>
#include <stack>

#define MAX_BYTES_IN_PACKET 10

struct Packet {
  uint8_t buffer[MAX_BYTES_IN_PACKET];
  uint8_t numberOfBits;
  uint8_t numberOfRepeats;
  uint8_t currentBit;
}; // Packet

struct SignalGenerator {
  template<int timerIndex>
  void configureSignal(String, uint8_t, uint16_t);
  template<int timerIndex>
  void startSignal();
  template<int timerIndex>
  void stopSignal();

  bool IRAM_ATTR getNextBitToSend();
  void loadPacket(std::vector<uint8_t>, int);
  void waitForQueueEmpty();
  bool isQueueEmpty();

  hw_timer_t *_fullCycleTimer;
  hw_timer_t *_pulseTimer;
  String _name;
  uint8_t _directionPin;
  int _currentMonitorPin;
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
};

extern SignalGenerator dccSignal[2];
void configureDCCSignalGenerators();
void startDCCSignalGenerators();
void stopDCCSignalGenerators();

int16_t readCV(const uint16_t);
bool writeProgCVByte(const uint16_t, const uint8_t);
bool writeProgCVBit(const uint16_t, const uint8_t, const bool);
void writeOpsCVByte(const uint16_t, const uint16_t, const uint8_t);
void writeOpsCVBit(const uint16_t, const uint16_t, const uint8_t, const bool);

#define DCC_SIGNAL_OPERATIONS 0
#define DCC_SIGNAL_PROGRAMMING 1
#define MAX_DCC_SIGNAL_GENERATORS 2

#endif

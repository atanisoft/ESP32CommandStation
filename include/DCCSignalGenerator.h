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
#pragma once

#include <Arduino.h>
#include <queue>
#include <stdint.h>

#define MAX_BYTES_IN_PACKET 10

// standard DCC packet (S-9.2)
// byte #
// 0         1         2         3
// 1111 1111 1111 1111 1111 110X XXXX XXXX
// 0xFF      0xFF      0xFC
// 22 BIT PREAMBLE            ^ marker for END of preamble, must be zero
//   minimum is 14 bits        ^ bit 7 of first byte of payload
//                                       ^ bit 7 of second byte of payload
// 4         5         6         7
// XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX
//         ^ bit 7 of third byte
//                   ^ bit 7 of fourth byte
//                             ^ bit 7 of fifth byte
//                                       ^ bit 7 of sixth byte
// 8         9
// XXXX XXXX XXXX XXXX
//        ^ bit 1 of sixth byte (last bit of packet)
struct Packet {
  uint8_t buffer[MAX_BYTES_IN_PACKET];
  uint8_t numberOfBits;
  uint8_t numberOfRepeats;
  uint8_t currentBit;
};

class SignalGenerator {
public:
  void startSignal(bool=true);
  void stopSignal();
  void loadBytePacket(const uint8_t *, uint8_t, uint8_t, bool=false);
  void loadPacket(std::vector<uint8_t>, int, bool=false);
  void waitForQueueEmpty();
  bool isQueueEmpty();
  bool isEnabled();
  void drainQueue();
  Packet *getPacket();

protected:
  SignalGenerator(String, uint16_t, uint8_t, uint8_t);
  virtual void enable() = 0;
  virtual void disable() = 0;
  void lockSendQueue() {
    portENTER_CRITICAL(&_sendQueueMUX);
  }
  void unlockSendQueue() {
    portEXIT_CRITICAL(&_sendQueueMUX);
  }
  virtual void lockSendQueueISR() {
    lockSendQueue();
  }
  virtual void unlockSendQueueISR() {
    unlockSendQueue();
  }

  const String _name;
  const uint8_t _signalID;
  portMUX_TYPE _sendQueueMUX = portMUX_INITIALIZER_UNLOCKED;
private:
  std::queue<Packet *> _toSend;
  std::queue<Packet *> _availablePackets;
  Packet *_currentPacket;

  // pre-encoded idle packet that gets sent when the _toSend queue is empty.
  Packet _idlePacket {
    { 0xFF, 0xFF, 0xFD, 0xFE, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00 }, // packet bytes
    49, // number of bits
    0, // number of repeats
    0 // current bit
  };

  bool _enabled{false};
};

// S-9.2 baseline packet (idle)
static constexpr DRAM_ATTR uint8_t idlePacket[] = {0xFF, 0x00};
// S-9.2 baseline packet (decoder reset)
static constexpr DRAM_ATTR uint8_t resetPacket[] = {0x00, 0x00};
// S-9.2 baseline packet (eStop, direction bit ignored)
static constexpr DRAM_ATTR uint8_t eStopPacket[] = {0x00, 0x41};

// bitmask used by signal generator when processing DCC packet bytes
static constexpr DRAM_ATTR uint8_t DCC_PACKET_BIT_MASK[] = {
  0x80, 0x40, 0x20, 0x10,
  0x08, 0x04, 0x02, 0x01
};

#define DCC_SIGNAL_OPERATIONS 0
#define DCC_SIGNAL_PROGRAMMING 1
#define MAX_DCC_SIGNAL_GENERATORS 2

extern SignalGenerator *dccSignal[MAX_DCC_SIGNAL_GENERATORS];
void startDCCSignalGenerators();
bool stopDCCSignalGenerators();
bool isDCCSignalEnabled();
void sendDCCEmergencyStop();

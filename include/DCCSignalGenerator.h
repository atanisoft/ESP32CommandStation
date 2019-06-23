/**********************************************************************
ESP32 COMMAND STATION

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
#include <mutex>
#include <queue>
#include <stdint.h>

// Signal generator is for the OPS track
#define DCC_SIGNAL_OPERATIONS 0
// Signal generator is for the PROG track
#define DCC_SIGNAL_PROGRAMMING 1
// MAX number of signal generators
#define MAX_DCC_SIGNAL_GENERATORS 2

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
  int8_t numberOfRepeats;
};

class SignalGenerator {
public:
  void startSignal(bool=true);
  void stopSignal();
  void loadBytePacket(const uint8_t *, uint8_t, uint8_t, bool=false);
  void loadPacket(std::vector<uint8_t>, int=0, bool=false);

  inline void waitForQueueEmpty() {
    while(!isQueueEmpty()) {
      delay(1);
    }
  }

  inline bool isQueueEmpty() {
    std::lock_guard<std::mutex> guard(_toSendMux);
    return _toSend.empty();
  }

  inline Packet *getNextPacket() {
    bool needNewPacket = false;
    if (_currentPacket) {
      _currentPacket->numberOfRepeats--;
      if (_currentPacket->numberOfRepeats <= 0) {
        LOG(VERBOSE, "[%s %d] DCCPacket(%p) sent", getName(), esp_log_timestamp(),
            _currentPacket);
        pushFreePacket(_currentPacket);
        _currentPacket = nullptr;
        needNewPacket = true;
      } else if(_signalID == DCC_SIGNAL_OPERATIONS) {
        // If this is the OPS signal move the packet back to the ready
        // to transmit queue so we can avoid sending back-to-back packets
        // to the same decoder.
        LOG(VERBOSE, "[%s %d] Remaining DCCPacket(%p) repeats deferred", getName(),
            esp_log_timestamp(), _currentPacket);
        pushReadyPacket(_currentPacket);
        // drop reference to the packet so we can send an idle between
        // packets if there are no other packets in the toSend queue.
        _currentPacket = nullptr;
      }
    } else {
      // we don't currently have a packet, check if there is one to send
      needNewPacket = true;
    }
    if (needNewPacket && !isQueueEmpty()) {
      std::lock_guard<std::mutex> guard(_toSendMux);
      LOG(VERBOSE, "[%s %d] DCCPacket send queue size: %d", getName(), esp_log_timestamp(),
          _toSend.size());
      _currentPacket = _toSend.front();
      _toSend.pop();
    }
    if(_currentPacket) {
      LOG(VERBOSE, "[%s %d] Current DCCPacket(%p) (%d bits, %d remaining repeats)", getName(),
          esp_log_timestamp(), _currentPacket, _currentPacket->numberOfBits,
          _currentPacket->numberOfRepeats);
    }
    return _currentPacket;
  }

  inline const char *getName() {
    return _name.c_str();
  }

  inline bool isEnabled() {
    return _enabled;
  }

  inline bool isQueueNearCapacity() {
    std::lock_guard<std::mutex> guard(_toSendMux);
    return (_sendQueueCapacity - _toSend.size()) > _sendQueueThreshold;
  }
  inline size_t sendQueueUtilization() {
    std::lock_guard<std::mutex> guard(_toSendMux);
    return _toSend.size();
  }

protected:
  SignalGenerator(String, uint16_t, uint8_t, uint8_t);
  virtual void enable() = 0;
  virtual void disable() = 0;
  const String _name;
  const uint8_t _signalID;
private:
  inline void drainQueue() {
    // drain any pending packets before we start the signal so we start with an empty queue
    if(!isQueueEmpty()) {
      LOG(INFO, "[%s] Draining packet queue", getName());
      while(!isQueueEmpty()) {
        std::lock_guard<std::mutex> guard(_toSendMux);
        _currentPacket = _toSend.front();
        _toSend.pop();
        pushFreePacket(_currentPacket);
      }
    }
  }

  inline bool isFreePacketQueueEmpty() {
    std::lock_guard<std::mutex> guard(_availablePacketsMux);
    return _availablePackets.empty();
  }

  inline Packet *getFreePacket() {
    while(isFreePacketQueueEmpty()) {
      // delay long enough for at least one packet to be released from the queue,
      // this is calculated as 76 ZERO bits (~152mS).
      LOG(WARNING, "[%s] DCC packet queue full, delaying for 300ms!", getName());
      delay(300);
    }
    std::lock_guard<std::mutex> guard(_availablePacketsMux);
    Packet *packet = _availablePackets.front();
    _availablePackets.pop();
    return packet;
  }

  inline void pushFreePacket(Packet *packet) {
    std::lock_guard<std::mutex> guard(_availablePacketsMux);
    _availablePackets.push(packet);
  }

  inline void pushReadyPacket(Packet *packet) {
    std::lock_guard<std::mutex> guard(_toSendMux);
    LOG(VERBOSE, "[%s] Adding DCC Packet (%d bits, %d repeat)", getName(), packet->numberOfBits, packet->numberOfRepeats);
    _toSend.push(packet);
  }

  std::mutex _toSendMux;
  std::mutex _availablePacketsMux;
  std::queue<Packet *> _toSend;
  std::queue<Packet *> _availablePackets;
  Packet *_currentPacket{nullptr};
  uint16_t _sendQueueCapacity{0};
  uint16_t _sendQueueThreshold{0};

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

extern SignalGenerator *dccSignal[MAX_DCC_SIGNAL_GENERATORS];
void startDCCSignalGenerators();
bool stopDCCSignalGenerators();
bool isDCCSignalEnabled();
void sendDCCEmergencyStop();

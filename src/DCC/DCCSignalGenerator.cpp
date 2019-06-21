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

#include "ESP32CommandStation.h"

SignalGenerator *dccSignal[MAX_DCC_SIGNAL_GENERATORS];
void startDCCSignalGenerators() {
  // NOTE: DCC_SIGNAL_PROGRAMMING is intentionally not started here, it will be managed with
  // the programming track methods below.
  if(!dccSignal[DCC_SIGNAL_OPERATIONS]->isEnabled()) {
    dccSignal[DCC_SIGNAL_OPERATIONS]->startSignal();
  }
}

bool stopDCCSignalGenerators() {
  bool reEnableNeeded = dccSignal[DCC_SIGNAL_OPERATIONS]->isEnabled();
  if(dccSignal[DCC_SIGNAL_OPERATIONS]->isEnabled()) {
    dccSignal[DCC_SIGNAL_OPERATIONS]->stopSignal();
  }
  if(dccSignal[DCC_SIGNAL_PROGRAMMING]->isEnabled()) {
    dccSignal[DCC_SIGNAL_PROGRAMMING]->stopSignal();
  }
  return reEnableNeeded;
}

bool isDCCSignalEnabled() {
  if(dccSignal[DCC_SIGNAL_OPERATIONS]->isEnabled() || dccSignal[DCC_SIGNAL_PROGRAMMING]->isEnabled()) {
    return true;
  }
  return false;
}

void sendDCCEmergencyStop() {
  for(auto generator : dccSignal) {
    if(generator->isEnabled()) {
      generator->loadBytePacket(eStopPacket, 2, 0, true);
    }
  }
}

void SignalGenerator::loadBytePacket(const uint8_t *data, uint8_t length, uint8_t repeatCount, bool drainToSendQueue) {
  std::vector<uint8_t> packet;
  for(int i = 0; i < length; i++) {
    packet.push_back(data[i]);
  }
  loadPacket(packet, repeatCount, drainToSendQueue);
}

void SignalGenerator::loadPacket(std::vector<uint8_t> data, int numberOfRepeats, bool drainToSendQueue) {
  // minimum DCC packet size is 2 bytes (excluding preamble bits and checksum byte)
  if(data.size() < 2) {
    return;
  }
  if(drainToSendQueue) {
    drainQueue();
  }
  Packet *packet = getFreePacket();
  memset(packet, 0, sizeof(Packet));
  packet->numberOfRepeats = numberOfRepeats;

  // calculate checksum (XOR)
  // add first byte as checksum byte
  uint8_t checksum = data[0];
  for(int i = 1; i < data.size(); i++) {
    checksum ^= data[i];
  }
  data.push_back(checksum);

  // 22 bit DCC preamble
  packet->buffer[0] = 0xFF;
  packet->buffer[1] = 0xFF;
  // first bit of actual data at the end of the preamble
  packet->buffer[2] = 0xFC + bitRead(data[0], 7);
  packet->buffer[3] = data[0] << 1;
  packet->buffer[4] = data[1];
  packet->buffer[5] = data[2] >> 1;
  packet->buffer[6] = data[2] << 7;
  packet->numberOfBits = 49;

  if (data.size() >= 4) {
    packet->buffer[6] += data[3] >> 2;
    packet->buffer[7] = data[3] << 6;
    packet->numberOfBits = 58;
  }
  if (data.size() >= 5) {
    packet->buffer[7] += data[4] >> 3;
    packet->buffer[8] = data[4] << 5;
    packet->numberOfBits = 67;
  }
  if (data.size() >= 6) {
    packet->buffer[8] += data[5] >> 4;
    packet->buffer[9] = data[5] << 4;
    packet->numberOfBits = 76;
  }
  pushReadyPacket(packet);
}

SignalGenerator::SignalGenerator(String name, uint16_t maxPackets, uint8_t signalID, uint8_t signalPin) : _name(name), _signalID(signalID) {
  HASSERT(signalID < MAX_DCC_SIGNAL_GENERATORS);

  LOG(INFO, "[%s] Configuring DCC signal generator using pin %d and %d max packets", getName(), signalPin, maxPackets);
  pinMode(signalPin, INPUT);
  digitalWrite(signalPin, LOW);
  pinMode(signalPin, OUTPUT);

  // create packets for this signal generator up front, they will be reused until
  // the command station is shutdown
  for(int index = 0; index < maxPackets; index++) {
    pushFreePacket(new Packet());
  }
}

void SignalGenerator::startSignal(bool sendIdlePackets) {
  if(_enabled) {
    return;
  }

  // drain any pending packets from the queue before starting the signal
  drainQueue();

  // reset to initial state
  _currentPacket = nullptr;

  // inject the required reset and idle packets into the queue
  // this is required as part of S-9.2.4 section A
  // at least 20 reset packets and 10 idle packets must be sent upon initialization
  // of the command station to force decoders to exit service mode.
  LOG(INFO, "[%s] Adding reset packet (25 repeats) to packet queue", getName());
  loadBytePacket(resetPacket, 2, 25);
  if(sendIdlePackets) {
    LOG(INFO, "[%s] Adding idle packet (10 repeats) to packet queue", getName());
    loadBytePacket(idlePacket, 2, 10);
  }
  enable();
  _enabled = true;
}

void SignalGenerator::stopSignal() {
  if(_enabled) {
    disable();

    // if we have a current packet being processed move it to the available
    // queue.
    if(_currentPacket) {
      pushFreePacket(_currentPacket);
    }

    // drain any remaining packets that were not sent back into the available
    // to use packets.
    drainQueue();
  }

  _enabled = false;
}
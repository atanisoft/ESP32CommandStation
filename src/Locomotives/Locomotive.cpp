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

// This controls how often to send a speed update packet to the decoder,
// this is the minimum periodic refresh period. It will always be sent
// when the speed changes.
constexpr uint64_t LOCO_SPEED_PACKET_INTERVAL = MSEC_TO_USEC(100);

// This controls how often to send the locomotive function packets,
// default is approximately every second.
constexpr uint64_t LOCO_FUNCTION_PACKET_INTERVAL = SEC_TO_USEC(60);

Locomotive::Locomotive(uint8_t registerNumber) : _registerNumber(registerNumber) {
  createFunctionPackets();
}

Locomotive::Locomotive(const char *filename) {
  DynamicJsonBuffer buf;
  JsonObject &entry = configStore.load(filename, buf);
  _locoAddress = entry[JSON_ADDRESS_NODE];
  _speed = entry[JSON_SPEED_NODE];
  _direction = entry[JSON_DIRECTION_NODE] == JSON_VALUE_FORWARD;
  _orientation = entry[JSON_ORIENTATION_NODE] == JSON_VALUE_FORWARD;
  // TODO: add function state loading
  createFunctionPackets();
}

Locomotive::Locomotive(JsonObject &json) {
  _locoAddress = json[JSON_ADDRESS_NODE];
  _speed = json[JSON_SPEED_NODE];
  _direction = json[JSON_DIRECTION_NODE] == JSON_VALUE_FORWARD;
  _orientation = json[JSON_ORIENTATION_NODE] == JSON_VALUE_FORWARD;
}

void Locomotive::sendLocoUpdate(bool force) {
  if(force || esp_timer_get_time() > (_lastPacketTime + LOCO_SPEED_PACKET_INTERVAL)) {
    LOG(VERBOSE, "[Loco %d, speed: %d, dir: %s] Building speed packet",
      _locoAddress, _speed, _direction ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE);
    std::vector<uint8_t> packetBuffer;
    if(_locoAddress > 127) {
      packetBuffer.push_back((uint8_t)(0xC0 | highByte(_locoAddress)));
    }
    packetBuffer.push_back(lowByte(_locoAddress));
    // S-9.2.1 Advanced Operations instruction
    // using 128 speed steps
    packetBuffer.push_back(0x3F);
    if(_speed < 0) {
      _speed = 0;
      packetBuffer.push_back(1);
    } else {
      packetBuffer.push_back((uint8_t)(_speed + (_speed > 0) + _direction * 128));
    }
    dccSignal[DCC_SIGNAL_OPERATIONS]->loadPacket(packetBuffer);
    _lastPacketTime = esp_timer_get_time();
  }
  // if we are not sending a forced packet and are not near capacity on the send queue,
  // push function packets that haven't been sent recently. The CS will always send
  // function packets to the queue when a function changes. This check is necessary to
  // ensure we don't flood the outbound queue unnecessarily.
  if(!force && !dccSignal[DCC_SIGNAL_OPERATIONS]->isQueueNearCapacity()) {
    for(uint8_t pkt = 0; pkt < MAX_LOCOMOTIVE_FUNCTION_PACKETS; pkt++) {
      if(esp_timer_get_time() > (_lastFunctionsPacketTime[pkt] + LOCO_FUNCTION_PACKET_INTERVAL)) {
        dccSignal[DCC_SIGNAL_OPERATIONS]->loadPacket(_functionPackets[pkt]);
      }
    }
  }
  // move the last update period forward regardless of sending packets. 
  for(uint8_t pkt = 0; pkt < MAX_LOCOMOTIVE_FUNCTION_PACKETS; pkt++) {
    _lastFunctionsPacketTime[pkt] = esp_timer_get_time();
  }
}

void Locomotive::showStatus() {
  LOG(INFO, "[Loco %d] speed: %d, direction: %s",
    _locoAddress, _speed, _direction ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE);
  wifiInterface.print(F("<T %d %d %d>"), _registerNumber, _speed, _direction);
}

void Locomotive::toJson(JsonObject &jsonObject, bool includeSpeedDir, bool includeFunctions) {
  jsonObject[JSON_ADDRESS_NODE] = _locoAddress;
  if(includeSpeedDir) {
    jsonObject[JSON_SPEED_NODE] = _speed;
    jsonObject[JSON_DIRECTION_NODE] = _direction ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE;
  }
  jsonObject[JSON_ORIENTATION_NODE] = _orientation ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE;
  if(includeFunctions) {
    JsonArray &functions = jsonObject.createNestedArray(JSON_FUNCTIONS_NODE);
    for(uint8_t funcID = 0; funcID < MAX_LOCOMOTIVE_FUNCTIONS; funcID++) {
      JsonObject &node = functions.createNestedObject();
      node[JSON_ID_NODE] = funcID;
      node[JSON_STATE_NODE] = _functionState[funcID];
    }
  }
}

void Locomotive::createFunctionPackets() {
  LOG(VERBOSE, "[Loco %d] Building Function packets", _locoAddress);
  // seed functions packets with locomotive numbers
  for(uint8_t functionPacket = 0; functionPacket < MAX_LOCOMOTIVE_FUNCTION_PACKETS; functionPacket++) {
    _functionPackets[functionPacket].clear();
    if(_locoAddress > 127) {
      // convert train number into a two-byte address
      _functionPackets[functionPacket].push_back((uint8_t)(0xC0 | highByte(_locoAddress)));
    }
    _functionPackets[functionPacket].push_back(lowByte(_locoAddress));
  }

  uint8_t packetByte[2] = {0x80, 0x00};
  // convert functions 0 - 4
  for(uint8_t funcID = 0; funcID <= 4; funcID++) {
    if(funcID && _functionState[funcID]) {
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
      bitSet(packetByte[0], funcID-1);
    } else if(funcID) {
      bitClear(packetByte[0], funcID-1);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    } else if(_functionState[funcID]) {
      bitSet(packetByte[0], 4);
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
    } else {
      bitClear(packetByte[0], 4);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    }
  }
  _functionPackets[0].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 5 - 8
  packetByte[0] = 0xB0;
  for(uint8_t funcID = 5; funcID <= 8; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[0], funcID-5);
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
    } else {
      bitClear(packetByte[0], funcID-5);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    }
  }
  _functionPackets[1].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 9 - 12
  packetByte[0] = 0xA0;
  for(uint8_t funcID = 9; funcID <= 12; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[0], funcID-9);
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
    } else {
      bitClear(packetByte[0], funcID-9);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    }
  }
  _functionPackets[2].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 13 - 20
  packetByte[0] = 0xDE;
  for(uint8_t funcID = 13; funcID <= 20; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[1], funcID-13);
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
    } else {
      bitClear(packetByte[1], funcID-13);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    }
  }
  _functionPackets[3].push_back((packetByte[0] | 0xDE) & 0xDF);
  _functionPackets[3].push_back(packetByte[1]);

  // convert functions 21 - 28
  packetByte[0] = 0xDF;
  for(uint8_t funcID = 21; funcID <= 28; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[1], funcID-21);
      LOG(VERBOSE, "[Loco %d] Function %d ON", _locoAddress, funcID);
    } else {
      bitClear(packetByte[1], funcID-21);
      LOG(VERBOSE, "[Loco %d] Function %d OFF", _locoAddress, funcID);
    }
  }
  _functionPackets[4].push_back((packetByte[0] | 0xDE) & 0xDF);
  _functionPackets[4].push_back(packetByte[1]);
}

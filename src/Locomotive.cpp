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

#include "DCCppESP32.h"

#include "Locomotive.h"

LinkedList<Locomotive *> LocomotiveManager::_locos([](Locomotive *loco) {delete loco; });

Locomotive::Locomotive(uint8_t registerNumber) :
  _registerNumber(registerNumber), _locoAddress(0), _speed(0), _direction(true),
  _lastUpdate(0), _idleOnStartup(false), _defaultOnThrottles(false), _functionsChanged(true) {
  for(uint8_t funcID = 0; funcID < MAX_LOCOMOTIVE_FUNCTIONS; funcID++) {
    _functionState[funcID] = false;
  }
}

void Locomotive::sendLocoUpdate() {
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
  dccSignal[DCC_SIGNAL_OPERATIONS].loadPacket(packetBuffer, 0);
  if(_functionsChanged) {
    _functionsChanged = false;
    createFunctionPackets();
  }
  for(uint8_t functionPacket = 0; functionPacket < MAX_LOCOMOTIVE_FUNCTION_PACKETS; functionPacket++) {
    dccSignal[DCC_SIGNAL_OPERATIONS].loadPacket(_functionPackets[functionPacket], 0);
  }
  _lastUpdate = millis();
}

void Locomotive::showStatus() {
  log_i("Loco(%d) locoNumber: %d, speed: %d, direction: %s",
    _registerNumber, _locoAddress, _speed, _direction ? "FWD" : "REV");
  wifiInterface.printf(F("<T %d %d %d>"), _registerNumber, _speed, _direction);
}

void Locomotive::toJson(JsonObject &jsonObject, bool basic) {
  jsonObject[F("address")] = _locoAddress;
  if(!basic) {
    jsonObject[F("speed")] = _speed;
    jsonObject[F("dir")] = _direction ? "FWD" : "REV";
  }
  jsonObject[F("idleOnStartup")] = _idleOnStartup ? "true" : "false";
  jsonObject[F("defaultOnThrottles")] = _defaultOnThrottles ? "true" : "false";
}

void Locomotive::createFunctionPackets() {
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
      bitSet(packetByte[0], funcID-1);
    } else if(funcID) {
      bitClear(packetByte[0], funcID-1);
    } else if(_functionState[funcID]) {
      bitSet(packetByte[0], 4);
    } else {
      bitClear(packetByte[0], 4);
    }
  }
  _functionPackets[0].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 5 - 8
  packetByte[0] = 0xB0;
  for(uint8_t funcID = 5; funcID <= 8; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[0], funcID-5);
    } else {
      bitClear(packetByte[0], funcID-5);
    }
  }
  _functionPackets[1].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 9 - 12
  packetByte[0] = 0xA0;
  for(uint8_t funcID = 9; funcID <= 12; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[0], funcID-9);
    } else {
      bitClear(packetByte[0], funcID-9);
    }
  }
  _functionPackets[2].push_back((packetByte[0] | 0x80) & 0xBF);

  // convert functions 13 - 20
  packetByte[0] = 0xDE;
  for(uint8_t funcID = 13; funcID <= 20; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[1], funcID-13);
    } else {
      bitClear(packetByte[1], funcID-13);
    }
  }
  _functionPackets[3].push_back((packetByte[0] | 0xDE) & 0xDF);
  _functionPackets[3].push_back(packetByte[1]);

  // convert functions 21 - 28
  packetByte[0] = 0xDF;
  for(uint8_t funcID = 21; funcID <= 28; funcID++) {
    if(_functionState[funcID]) {
      bitSet(packetByte[1], funcID-21);
    } else {
      bitClear(packetByte[1], funcID-21);
    }
  }
  _functionPackets[4].push_back((packetByte[0] | 0xDE) & 0xDF);
  _functionPackets[4].push_back(packetByte[1]);
}

void LocomotiveManager::processThrottle(const std::vector<String> arguments) {
  int registerNumber = arguments[0].toInt();
  Locomotive *instance = nullptr;
  for (const auto& loco : _locos) {
    if(loco->getRegister() == registerNumber) {
      instance = loco;
    }
  }
  if(instance == nullptr) {
    instance = new Locomotive(registerNumber);
    _locos.add(instance);
  }
  instance->setLocoAddress(arguments[1].toInt());
  instance->setSpeed(arguments[2].toInt());
  instance->setDirection(arguments[3].toInt() == 1);
  instance->sendLocoUpdate();
  instance->showStatus();
}

// This method decodes the incoming function packet(s) to update the stored
// functinon states. Loco update will be sent afterwards.
void LocomotiveManager::processFunction(const std::vector<String> arguments) {
  int locoAddress = arguments[0].toInt();
  int functionByte = arguments[1].toInt();
  auto loco = getLocomotive(locoAddress);
  // check this is a request for functions F13-F28
  if(arguments.size() > 2) {
    int secondaryFunctionByte = arguments[2].toInt();
    if(functionByte & 0xDE) {
      for(uint8_t funcID = 13; funcID <= 20; funcID++) {
        loco->setFunction(funcID, bitRead(secondaryFunctionByte, funcID-13));
      }
    } else {
      for(uint8_t funcID = 21; funcID <= 28; funcID++) {
        loco->setFunction(funcID, bitRead(secondaryFunctionByte, funcID-21));
      }
    }
  } else {
    // this is a request for functions FL,F1-F12
    // for safety this guarantees that first nibble of function byte will always
    // be of binary form 10XX which should always be the case for FL,F1-F12
    if(functionByte & 0xA0) {
      for(uint8_t funcID = 9; funcID <= 12; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-9));
      }
    } else if(functionByte & 0xB0) {
      for(uint8_t funcID = 5; funcID <= 8; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-5));
      }
    } else {
      loco->setFunction(0, bitRead(functionByte, 4));
      for(uint8_t funcID = 1; funcID <= 4; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-1));
      }
    }
  }
  loco->sendLocoUpdate();
}

void LocomotiveManager::showStatus() {
  for (const auto& loco : _locos) {
		loco->showStatus();
	}
}

void LocomotiveManager::update() {
  for (const auto& loco : _locos) {
    // if it has been more than 50ms we should send a loco update packet
    if(millis() > loco->getLastUpdate() + 50) {
      loco->sendLocoUpdate();
    }
	}
}

void LocomotiveManager::emergencyStop() {
  for (const auto& loco : _locos) {
    loco->setSpeed(-1);
	}
  sendDCCEmergencyStop();
}

Locomotive *LocomotiveManager::getLocomotive(const uint16_t locoAddress) {
  Locomotive *instance = nullptr;
  for (const auto& loco : _locos) {
    if(loco->getLocoAddress() == locoAddress) {
      instance = loco;
    }
	}
  if(instance == nullptr) {
    instance = new Locomotive(_locos.length());
    instance->setLocoAddress(locoAddress);
    _locos.add(instance);
  }
  return instance;
}

void LocomotiveManager::removeLocomotive(const uint16_t locoAddress) {
  Locomotive *locoToRemove = nullptr;
  for (const auto& loco : _locos) {
    if(loco->getLocoAddress() == locoAddress) {
      locoToRemove = loco;
    }
  }
  if(locoToRemove != nullptr) {
    locoToRemove->setIdle();
    locoToRemove->sendLocoUpdate();
    _locos.remove(locoToRemove);
  }
}

void LocomotiveManager::init() {
  // TODO: load roster from storage and add any idle by default locos
  //getLocomotive(1234)->setDefaultOnThrottles(true);
  //getLocomotive(5678)->setDefaultOnThrottles(true);
  //getLocomotive(9012)->setDefaultOnThrottles(true);
}

uint16_t LocomotiveManager::store() {
  // TODO: persist roster to storage
  return 0;
}

std::vector<Locomotive *> LocomotiveManager::getDefaultLocos(const int8_t maxCount) {
  std::vector<Locomotive *> retval;
  for (const auto& loco : _locos) {
    if(loco->isDefaultOnThrottles()) {
      if(maxCount < 0 || (maxCount > 0 && retval.size() < maxCount)) {
        retval.push_back(loco);
      }
    }
  }
  return retval;
}

void LocomotiveManager::getDefaultLocos(JsonArray &array) {
  for (const auto& loco : _locos) {
    if(loco->isDefaultOnThrottles()) {
      loco->toJson(array.createNestedObject(), true);
    }
  }
}

void LocomotiveManager::getActiveLocos(JsonArray &array) {
  for (const auto& loco : _locos) {
    loco->toJson(array.createNestedObject());
  }
}

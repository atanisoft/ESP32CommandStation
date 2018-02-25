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
  _registerNumber(registerNumber), _locoNumber(0), _speed(0), _direction(0),
  _lastUpdate(0) {
}

void Locomotive::sendLocoUpdate() {
  std::vector<uint8_t> packetBuffer;
  if(_locoNumber > 127) {
    packetBuffer.push_back((uint8_t)(0xC0 | highByte(_locoNumber)));
  }
  packetBuffer.push_back(lowByte(_locoNumber));
  packetBuffer.push_back(0x3F);
  if(_speed < 0) {
    _speed = 0;
    packetBuffer.push_back(1);
  } else {
    packetBuffer.push_back((uint8_t)(_speed + (_speed > 0) + _direction * 128));
  }
  dccSignal[DCC_SIGNAL_OPERATIONS].loadPacket(packetBuffer, 0);
  _lastUpdate = millis();
}

void Locomotive::showStatus() {
  log_i("Loco(%d) locoNumber: %d, speed: %d, direction: %s",
    _registerNumber, _locoNumber, _speed, _direction ? "FWD" : "REV");
  wifiInterface.printf(F("<T %d %d %d>"), _registerNumber, _speed, _direction);
}

void LocomotiveManager::processThrottle(const std::vector<String> arguments) {
  int registerNumber = arguments[0].toInt();
  Locomotive *instance = NULL;
  for (const auto& loco : _locos) {
    if(loco->getRegister() == registerNumber) {
      instance = loco;
    }
  }
  if(instance == NULL) {
    instance = new Locomotive(registerNumber);
    _locos.add(instance);
  }
  instance->setLocoNumber(arguments[1].toInt());
  instance->setSpeed(arguments[2].toInt());
  instance->setDirection(arguments[3].toInt() == 1);
  instance->sendLocoUpdate();
  instance->showStatus();
}

void LocomotiveManager::processFunction(const std::vector<String> arguments) {
  std::vector<uint8_t> packetBuffer;
  int locoNumber = arguments[0].toInt();
  int functionByte = arguments[1].toInt();
  if(locoNumber > 127) {
    // convert train number into a two-byte address
    packetBuffer.push_back(highByte(locoNumber) | 0xC0);
  }
  packetBuffer.push_back(lowByte(locoNumber));
  // check this is a request for functions F13-F28
  if(arguments.size() > 2) {
    int secondaryFunctionByte = arguments[2].toInt();
    // for safety this guarantees that first byte will either be 0xDE (for
    // F13-F20) or 0xDF (for F21-F28)
    packetBuffer.push_back((functionByte | 0xDE) & 0xDF);
    packetBuffer.push_back(secondaryFunctionByte);
  } else {
    // this is a request for functions FL,F1-F12
    // for safety this guarantees that first nibble of function byte will always
    // be of binary form 10XX which should always be the case for FL,F1-F12
    packetBuffer.push_back((functionByte | 0x80) & 0xBF);
  }
  dccSignal[DCC_SIGNAL_OPERATIONS].loadPacket(packetBuffer, 4);
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
    loco->sendLocoUpdate();
	}
}

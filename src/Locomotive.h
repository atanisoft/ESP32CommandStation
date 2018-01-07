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

#ifndef _LOCOMOTIVE_H_
#define _LOCOMOTIVE_H_

#include <functional>
#include <StringArray.h>
#include "DCCppProtocol.h"

class Locomotive {
public:
  Locomotive(uint8_t registerNumber);
  uint8_t getRegister() {
    return _registerNumber;
  }
  void setLocoNumber(uint16_t locoNumber) {
    _locoNumber = locoNumber;
  }
  uint16_t getLocoNumber() {
    return _locoNumber;
  }
  void setSpeed(int8_t speed) {
    _speed = speed;
  }
  int8_t setSpeed() {
    return _speed;
  }
  void setDirection(bool forward) {
    _direction = forward;
  }
  bool isDirectionForward() {
    return _direction;
  }
  uint32_t getLastUpdate() {
    return _lastUpdate;
  }
  void sendLocoUpdate();
  void showStatus();
private:
  uint8_t _registerNumber;
  uint16_t _locoNumber;
  int8_t _speed;
  bool _direction;
  uint32_t _lastUpdate;
};

class LocomotiveManager {
public:
  static void update();
  static void processThrottle(const std::vector<String> arguments);
  static void processFunction(const std::vector<String> arguments);
  static void showStatus();
  static uint8_t getActiveLocoCount() {
    return _locos.length();
  }
private:
  static LinkedList<Locomotive *> _locos;
};

// <t {REGISTER} {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
class ThrottleCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    LocomotiveManager::processThrottle(arguments);
  }
  String getID() {
    return "t";
  }
};

// <f {LOCO} {BYTE} []{BYTE2}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
class FunctionCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    LocomotiveManager::processFunction(arguments);
  }
  String getID() {
    return "f";
  }
};
#endif

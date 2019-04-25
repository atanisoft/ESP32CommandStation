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

#include "DCCppESP32.h"

#define MAX_LOCOMOTIVE_FUNCTIONS 29
#define MAX_LOCOMOTIVE_FUNCTION_PACKETS 5

class Locomotive {
public:
  Locomotive(uint8_t);
  Locomotive(JsonObject &);
  virtual ~Locomotive() {}
  uint8_t getRegister() {
    return _registerNumber;
  }
  void setLocoAddress(uint16_t locoAddress) {
    _locoAddress = locoAddress;
  }
  uint16_t getLocoAddress() {
    return _locoAddress;
  }
  void setSpeed(int8_t speed) {
    if(speed < 0) {
      speed = 0;
    } else if(speed > 128) {
      speed = 128;
    }
    _speed = speed;
  }
  int8_t getSpeed() {
    return _speed;
  }
  void setDirection(bool forward) {
    _direction = forward;
  }
  bool isDirectionForward() {
    return _direction;
  }
  void setOrientationForward(bool forward) {
    _orientation = forward;
  }
  bool isOrientationForward() {
    return _orientation;
  }
  uint32_t getLastUpdate() {
    return _lastUpdate;
  }
  void setIdle() {
    setSpeed(0);
  }
  void sendLocoUpdate();
  void showStatus();
  void toJson(JsonObject &, bool=true, bool=true);
  void setFunction(uint8_t funcID, bool state=false) {
    log_v("[Loco %d] Setting function %d to %d", _locoAddress, funcID, state);
    _functionState[funcID] = state;
    _functionsChanged = true;
  }
  bool isFunctionEnabled(uint8_t funcID) {
    return _functionState[funcID];
  }
private:
  void createFunctionPackets();
  uint8_t _registerNumber;
  uint16_t _locoAddress;
  int8_t _speed;
  bool _direction;
  bool _orientation;
  uint32_t _lastUpdate;
  bool _functionsChanged;
  bool _functionState[MAX_LOCOMOTIVE_FUNCTIONS];
  std::vector<uint8_t> _functionPackets[MAX_LOCOMOTIVE_FUNCTION_PACKETS];
};

class LocomotiveConsist : public Locomotive {
public:
  LocomotiveConsist(uint8_t address, bool decoderAssistedConsist=false) :
    Locomotive(-1), _decoderAssisstedConsist(decoderAssistedConsist) {
    setLocoAddress(address);
  }
  LocomotiveConsist(JsonObject &);
  virtual ~LocomotiveConsist();
  void showStatus();
  void toJson(JsonObject &, bool=true, bool=true);
  bool isAddressInConsist(uint16_t);
  void updateThrottle(uint16_t, int8_t, bool);
  void addLocomotive(uint16_t, bool, uint8_t);
  bool removeLocomotive(uint16_t);
  void releaseLocomotives();
  bool isDecoderAssistedConsist() {
    return _decoderAssisstedConsist;
  }
  void sendLocoUpdate() {
    if (_decoderAssisstedConsist) {
      Locomotive::sendLocoUpdate();
    } else {
      for (const auto& loco : _locos) {
        loco->sendLocoUpdate();
      }
    }
  }
private:
  bool _decoderAssisstedConsist;
  std::vector<Locomotive *> _locos;
};

class RosterEntry {
public:
  RosterEntry(uint16_t address) : _description(""), _address(address), _type(""),
    _idleOnStartup(false), _defaultOnThrottles(false) {}
  RosterEntry(const JsonObject &);
  void toJson(JsonObject &);
  void setDescription(String description) {
    _description = description;
  }
  String getDescription() {
    return _description;
  }
  void setAddress(const uint16_t address) {
    _address = address;
  }
  uint16_t getAddress() {
    return _address;
  }
  void setType(String type) {
    _type = type;
  }
  String getType() {
    return _type;
  }
  void setIdleOnStartup(bool value=false) {
    _idleOnStartup = value;
  }
  bool isIdleOnStartup() {
    return _idleOnStartup;
  }
  void setDefaultOnThrottles(bool value=false) {
    _defaultOnThrottles = value;
  }
  bool isDefaultOnThrottles() {
    return _defaultOnThrottles;
  }

private:
  String _description;
  uint16_t _address;
  String _type;
  bool _idleOnStartup;
  bool _defaultOnThrottles;
};

class LocomotiveManager {
public:
  // gets or creates a new locomotive to be managed
  static Locomotive *getLocomotive(const uint16_t, const bool=true);
  static Locomotive *getLocomotiveByRegister(const uint8_t);
  // removes a locomotive from management, sends speed zero before removal
  static void removeLocomotive(const uint16_t);
  static bool removeLocomotiveConsist(const uint16_t);
  static void processThrottle(const std::vector<String>);
  static void processFunction(const std::vector<String>);
  static void processFunctionEx(const std::vector<String>);
  static void processConsistThrottle(const std::vector<String>);
  static void showStatus();
  static void showConsistStatus();
  static void updateTask(void *param);
  static void emergencyStop();
  static uint8_t getActiveLocoCount() {
    return _locos.length();
  }
  static void init();
  static void clear();
  static uint16_t store();
  static std::vector<RosterEntry *> getDefaultLocos(const int8_t=-1);
  static void getDefaultLocos(JsonArray &);
  static void getActiveLocos(JsonArray &);
  static void getRosterEntries(JsonArray &);
  static bool isConsistAddress(uint16_t);
  static bool isAddressInConsist(uint16_t);
  static LocomotiveConsist *getConsistByID(uint8_t);
  static LocomotiveConsist *getConsistForLoco(uint16_t);
  static LocomotiveConsist *createLocomotiveConsist(int8_t);
  static RosterEntry *getRosterEntry(uint16_t, bool=true);
  static void removeRosterEntry(uint16_t);
private:
  static LinkedList<RosterEntry *> _roster;
  static LinkedList<Locomotive *> _locos;
  static LinkedList<LocomotiveConsist *> _consists;
  static TaskHandle_t _taskHandle;
  static xSemaphoreHandle _lock;
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

// <f {LOCO} {BYTE} [{BYTE2}]> command handler, this command converts a
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

// <fex {LOCO} {FUNC} {STATE}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
class FunctionExCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    LocomotiveManager::processFunctionEx(arguments);
  }
  String getID() {
    return "fex";
  }
};

// wrapper to handle the following command structures:
// CREATE: <C {ID} {LEAD LOCO} {TRAIL LOCO}  [{OTHER LOCO}]>
// DELETE: <C {ID} {LOCO}>
// DELETE: <C {ID}>
// QUERY : <C 0 {LOCO}>
// SHOW  : <C>
class ConsistCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "C";
  }
};

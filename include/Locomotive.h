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

#define MAX_LOCOMOTIVE_FUNCTIONS 29
#define MAX_LOCOMOTIVE_FUNCTION_PACKETS 5

class Locomotive {
public:
  Locomotive(uint8_t);
  Locomotive(JsonObject &);
  Locomotive(const char *);
  virtual ~Locomotive() {}
  int8_t getRegister() {
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
    LOG(INFO, "[Loco %d] speed: %d", _locoAddress, speed);
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
  void setIdle() {
    setSpeed(0);
    sendLocoUpdate(true);
  }
  void sendLocoUpdate(bool=false);
  void showStatus();
  void toJson(JsonObject &, bool=true, bool=true);

#define _LOCO_FUNCTION_UPDATE_IMPL(funcID, pkt, offs, base, limit, sendPacket) \
  if(funcID >= base && funcID <= limit) { \
    if(state) { \
      bitSet(_functionPackets[pkt][offs], funcID - base); \
    } else { \
      bitClear(_functionPackets[pkt][offs], funcID - base); \
    } \
    if(sendPacket) { \
      dccSignal[DCC_SIGNAL_OPERATIONS]->loadPacket(_functionPackets[pkt]); \
      _lastFunctionsPacketTime[pkt] = esp_timer_get_time(); \
    } \
    return; \
  }

  // batch mode function updates (used for protocol reader)
  void setFunctions(uint8_t firstFunction, uint8_t lastFunction, uint8_t mask) {
    for(uint8_t funcID = firstFunction; funcID <= lastFunction; funcID++) {
      setFunction(funcID, bitRead(mask, funcID - firstFunction), funcID < lastFunction);
    }
  }

  void setFunction(uint8_t funcID, bool state=false, bool batch=false) {
    LOG(INFO, "[Loco %d] F%d:%s", _locoAddress, funcID, state ? JSON_VALUE_ON : JSON_VALUE_OFF);
    _functionState[funcID] = state;
    uint8_t offs = 1;
    if(_locoAddress > 127) {
      offs++;
    }
    // handle function zero in special case
    if(!funcID) {
      if(state) {
        bitSet(_functionPackets[0][offs], 4);
      } else {
        bitClear(_functionPackets[0][offs], 4);
      }
      if(!batch) {
        dccSignal[DCC_SIGNAL_OPERATIONS]->loadPacket(_functionPackets[0]);
        _lastFunctionsPacketTime[0] = esp_timer_get_time();
      }
      return;
    }
    _LOCO_FUNCTION_UPDATE_IMPL(funcID, 0, offs, 1, 4, !batch)
    _LOCO_FUNCTION_UPDATE_IMPL(funcID, 1, offs, 5, 8, !batch)
    _LOCO_FUNCTION_UPDATE_IMPL(funcID, 2, offs, 9, 12, !batch)
    _LOCO_FUNCTION_UPDATE_IMPL(funcID, 3, offs + 1, 13, 20, !batch)
    _LOCO_FUNCTION_UPDATE_IMPL(funcID, 4, offs + 1, 21, 28, !batch)
  }
  bool isFunctionEnabled(uint8_t funcID) {
    return _functionState[funcID];
  }
private:
  void createFunctionPackets();
  int8_t _registerNumber{-1};
  uint16_t _locoAddress{0};
  int8_t _speed{0};
  bool _direction{true};
  bool _orientation{true};
  uint64_t _lastPacketTime{0};
  uint64_t _lastFunctionsPacketTime[MAX_LOCOMOTIVE_FUNCTION_PACKETS]{0,0,0,0,0};
  bool _functionState[MAX_LOCOMOTIVE_FUNCTIONS]{false,false,false,false,false,false,false,false,
                                                false,false,false,false,false,false,false,false,
                                                false,false,false,false,false,false,false,false,
                                                false,false,false,false};
  std::vector<uint8_t> _functionPackets[MAX_LOCOMOTIVE_FUNCTION_PACKETS];
};

class LocomotiveConsist : public Locomotive {
public:
  LocomotiveConsist(uint8_t address, bool decoderAssistedConsist=false) :
    Locomotive(-1), _decoderAssisstedConsist(decoderAssistedConsist) {
    setLocoAddress(address);
  }
  LocomotiveConsist(JsonObject &);
  LocomotiveConsist(const char *);
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
  RosterEntry(const char *);
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
  static void processThrottleEx(const std::vector<String>);
  static void processFunction(const std::vector<String>);
  static void processFunctionEx(const std::vector<String>);
  static void processConsistThrottle(const std::vector<String>);
  static void showStatus();
  static void showConsistStatus();
  static void update(void *);
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

// <tex {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
class ThrottleExCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    LocomotiveManager::processThrottleEx(arguments);
  }
  String getID() {
    return "tex";
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

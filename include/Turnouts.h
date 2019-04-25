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

#include <ArduinoJson.h>
#include "DCCppProtocol.h"

enum TurnoutType {
  LEFT=0,
  RIGHT,
  WYE,
  MULTI,
  MAX_TURNOUT_TYPES // NOTE: this must be the last entry in the enum.
};

void calculateTurnoutBoardAddressAndIndex(uint16_t *boardAddress, uint8_t *boardIndex, uint16_t address);

class Turnout {
public:
  Turnout(uint16_t, uint16_t, int8_t, bool=false, TurnoutType=TurnoutType::LEFT);
  Turnout(JsonObject &);
  virtual ~Turnout() {}
  void update(uint16_t, int8_t, TurnoutType);
  void set(bool=false, bool=true);
  void toJson(JsonObject &, bool=false);
  const uint16_t getID() {
    return _turnoutID;
  }
  const uint16_t getAddress() {
    return _address;
  }
  const uint16_t getBoardAddress() {
    return _boardAddress;
  }
  const uint8_t getIndex() {
    return _index;
  }
  const bool isThrown() {
    return _thrown;
  }
  const void toggle() {
    set(!_thrown);
  }
  void showStatus();
  const TurnoutType getType() {
    return _type;
  }
  void setType(const TurnoutType type) {
    _type = type;
  }
private:
  uint16_t _turnoutID;
  uint16_t _address;
  uint8_t _index;
  uint16_t _boardAddress;
  bool _thrown;
  TurnoutType _type;
};

class TurnoutManager {
public:
  static void init();
  static void clear();
  static uint16_t store();
  static bool setByID(uint16_t, bool=false);
  static bool toggleByID(uint16_t);
  static bool toggleByAddress(uint16_t);
  static void getState(JsonArray &, bool=true);
  static void showStatus();
  static Turnout *createOrUpdate(const uint16_t, const uint16_t, const int8_t, const TurnoutType=TurnoutType::LEFT);
  static bool removeByID(const uint16_t);
  static bool removeByAddress(const uint16_t);
  static Turnout *getTurnoutByIndex(const uint16_t);
  static Turnout *getTurnoutByID(const uint16_t);
  static Turnout *getTurnoutByAddress(const uint16_t);
  static uint16_t getTurnoutCount();
};

class TurnoutCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "T";
  }
};

class AccessoryCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "a";
  }
};

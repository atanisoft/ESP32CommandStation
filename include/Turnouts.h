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
#pragma once

#include <ArduinoJson.h>
#include "DCCppProtocol.h"

enum TurnoutOrientation {
  LEFT=0,
  RIGHT=2,
  Y=4,
  MULTI=8
};

class Turnout {
public:
  Turnout(uint16_t, uint16_t, int8_t, bool=false, TurnoutOrientation=LEFT);
  Turnout(JsonObject &);
  virtual ~Turnout() {}
  void update(uint16_t, int8_t);
  void set(bool=false);
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
  void showStatus();
  const TurnoutOrientation getOrientation() {
    return _orientation;
  }
  void setOrientation(const TurnoutOrientation orientation) {
    _orientation = orientation;
  }
private:
  uint16_t _turnoutID;
  uint16_t _address;
  uint8_t _index;
  uint16_t _boardAddress;
  bool _thrown;
  TurnoutOrientation _orientation;
  void calculateBoardAddressAndIndex() {
    _boardAddress = (_address + 3) / 4;
    _index = (_address - (_boardAddress * 4)) + 3;
  }
};

class TurnoutManager {
public:
  static void init();
  static void clear();
  static uint16_t store();
  static bool set(uint16_t, bool=false);
  static bool toggle(uint16_t);
  static void getState(JsonArray &);
  static void showStatus();
  static void createOrUpdate(const uint16_t, const uint16_t, const int8_t);
  static bool remove(const uint16_t);
  static Turnout *getTurnout(const uint16_t);
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

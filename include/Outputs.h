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
#include <ArduinoJson.h>
#include "DCCppProtocol.h"

const uint8_t OUTPUT_IFLAG_INVERT = 0;
const uint8_t OUTPUT_IFLAG_RESTORE_STATE = 1;
const uint8_t OUTPUT_IFLAG_FORCE_STATE = 2;

class Output {
public:
  Output(uint16_t, uint8_t, uint8_t);
  Output(JsonObject &);
  void set(bool=false, bool=true);
  void update(uint8_t, uint8_t);
  void toJson(JsonObject &, bool=false);
  const uint16_t getID() {
    return _id;
  }
  const uint8_t getPin() {
    return _pin;
  }
  const uint8_t getFlags() {
    return _flags;
  }
  const bool isActive() {
    return _active;
  }
  void showStatus();
  const String getFlagsAsString() {
    String flagsString = "";
    if(bitRead(_flags, OUTPUT_IFLAG_INVERT)) {
      flagsString += "activeLow";
    } else {
      flagsString += "activeHigh";
    }
    if(bitRead(_flags, OUTPUT_IFLAG_RESTORE_STATE)) {
      if(bitRead(_flags, OUTPUT_IFLAG_FORCE_STATE)) {
        flagsString += ",force(on)";
      } else {
        flagsString += ",force(off)";
      }
    } else {
      flagsString += ",restoreState";
    }
    return flagsString;
  }
private:
  uint16_t _id;
  uint8_t _pin;
  uint8_t _flags;
  bool _active;
};

class OutputManager {
  public:
    static void init();
    static void clear();
    static uint16_t store();
    static bool set(uint16_t, bool=false);
    static Output *getOutput(uint16_t);
    static bool toggle(uint16_t);
    static void getState(JsonArray &);
    static void showStatus();
    static bool createOrUpdate(const uint16_t, const uint8_t, const uint8_t);
    static bool remove(const uint16_t);
};

class OutputCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments);
  String getID() {
    return "Z";
  }
};

class OutputExCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments);
  String getID() {
    return "Zex";
  }
};
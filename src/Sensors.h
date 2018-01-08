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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <ArduinoJson.h>
#include "DCCppProtocol.h"

class Sensor {
public:
  Sensor(uint16_t, uint8_t, bool=false);
  Sensor(uint16_t);
  void update(uint8_t, bool=false);
  void store(uint16_t);
  const uint16_t getID() {
    return _sensorID;
  }
  const uint8_t getPin() {
    return _pin;
  }
  const bool isPullUp() {
    return _pullUp;
  }
  const bool isActive() {
    return _lastState;
  }
  void check();
  void show();
protected:
  uint16_t _sensorID;
  uint8_t _pin;
  bool _pullUp;
  bool _lastState;
};

class SensorManager {
public:
  static void init();
  static void clear();
  static uint16_t store();
  static void check();
  static void getState(JsonArray &);
  static void createOrUpdate(const uint16_t, const uint8_t, const bool);
  static bool remove(const uint16_t);
};

class SensorCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "S";
  }
};

#if defined(S88_ENABLED) && S88_ENABLED
class S88Sensor : public Sensor {
public:
  S88Sensor(uint16_t id, uint8_t index) : Sensor(id, 0, true), _index(index) {

  }
  void store(uint16_t) {}
  void check() {}
  void setState(bool state) {
    if(_lastState != state) {
      _lastState = state;
      show();
    }
  }
  const uint8_t getIndex() {
    return _index;
  }
private:
  const uint8_t _index;
};

class S88SensorGroup {
public:
  S88SensorGroup(uint8_t, uint8_t=16);
  void store(uint16_t);
  void show();
  const uint8_t getIndex() {
    return _index;
  }
  const uint8_t getPinCount() {
    return _pinCount;
  }
  void read();
private:
  const uint8_t _index;
  const uint8_t _pinCount;
  std::vector<S88Sensor *> _sensors;
};

class S88SensorManager {
public:
  static void init();
  static void clear();
  static uint8_t store();
  static void check();
  static bool create(const uint8_t, const uint8_t);
  static bool remove(const uint8_t);
};

class S88SensorCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "U";
  }
};
#endif

#endif

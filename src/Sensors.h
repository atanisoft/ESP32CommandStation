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

#include "DCCppESP32.h"

class Sensor {
public:
  Sensor(uint16_t, int8_t, bool=false, bool=true);
  Sensor(uint16_t);
  void update(uint8_t, bool=false);
  virtual void store(uint16_t);
  const uint16_t getID() {
    return _sensorID;
  }
  const int8_t getPin() {
    return _pin;
  }
  const bool isPullUp() {
    return _pullUp;
  }
  const bool isActive() {
    return _lastState;
  }
  virtual void check();
  void show();
protected:
  void set(bool state) {
    if(_lastState != state) {
      _lastState = state;
      log_i("Sensor: %d :: %s", _sensorID, _lastState ? "ACTIVE" : "INACTIVE");
      if(state) {
        wifiInterface.printf(F("<Q %d>"), _sensorID);
      } else {
        wifiInterface.printf(F("<q %d>"), _sensorID);
      }
    }
  }
  void setID(uint16_t id) {
    _sensorID = id;
  }
private:
  uint16_t _sensorID;
  int8_t _pin;
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
  static bool createOrUpdate(const uint16_t, const uint8_t, const bool);
  static bool remove(const uint16_t);
  static uint8_t getSensorPin(const uint16_t);
};

class SensorCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "S";
  }
};

#endif

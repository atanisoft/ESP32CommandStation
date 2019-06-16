/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 Mike Dunston
COPYRIGHT (c) 2018 Dan Worth

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
#include "Sensors.h"
#include "DCCppProtocol.h"

class RemoteSensor : public Sensor {
public:
  RemoteSensor(uint16_t, uint16_t=0);
  virtual ~RemoteSensor() {}
  const uint16_t getRawID() {
    return _rawID;
  }
  const uint16_t getSensorValue() {
    return _value;
  }
  void setSensorValue(const uint16_t value) {
    _value = value;
    _lastUpdate = millis();
    set(_value != 0);
  }
  const uint32_t getLastUpdate() {
    return _lastUpdate;
  }
  virtual void check();
  void showSensor();
  virtual void toJson(JsonObject &, bool=false);
private:
  uint16_t _rawID;
  uint16_t _value;
  uint32_t _lastUpdate;
};

class RemoteSensorManager {
public:
  static void init();
  static void show();
  static void createOrUpdate(const uint16_t, const uint16_t=0);
  static bool remove(const uint16_t);
  static void getState(JsonArray &);
};

class RemoteSensorsCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "RS";
  }
};

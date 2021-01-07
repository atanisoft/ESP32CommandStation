/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2021 Mike Dunston
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

#ifndef REMOTE_SENSORS_H_
#define REMOTE_SENSORS_H_

#include <DCCppProtocol.h>

#include "Sensors.h"

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(RemoteSensorsCommandAdapter, "RS", 0)

class RemoteSensor : public Sensor
{
public:
  RemoteSensor(uint16_t, uint16_t=0);
  virtual ~RemoteSensor() {}
  uint16_t getRawID() {
    return _rawID;
  }
  uint16_t getSensorValue()
  {
    return _value;
  }
  void setSensorValue(const uint16_t value)
  {
    _value = value;
    _lastUpdate = esp_timer_get_time() / 1000ULL;
    set(_value != 0);
  }
  uint32_t getLastUpdate()
  {
    return _lastUpdate;
  }
  virtual void check();
  std::string get_state_for_dccpp() override;
  virtual std::string toJson(bool=false) override;
private:
  uint16_t _rawID;
  uint16_t _value;
  uint32_t _lastUpdate;
};

class RemoteSensorManager
{
public:
  static void init();
  static void createOrUpdate(const uint16_t, const uint16_t=0);
  static bool remove(const uint16_t);
  static std::string getStateAsJson();
  static std::string get_state_for_dccpp();
};

#endif // REMOTE_SENSORS_H_
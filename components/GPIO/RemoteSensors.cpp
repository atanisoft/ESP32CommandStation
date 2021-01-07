/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018 Dan Worth
COPYRIGHT (c) 2018-2021 Mike Dunston

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

#include "sdkconfig.h"

#if defined(CONFIG_GPIO_SENSORS)

#include <json.hpp>
#include <DCCppProtocol.h>
#include <JsonConstants.h>
#include <utils/StringPrintf.hxx>

#include "GPIOValidation.h"
#include "RemoteSensors.h"

/**********************************************************************

The ESP32 Command Station supports remote sensor inputs that are connected via a
WiFi connection. Remote Sensors are dynamically created by a remote sensor
reporting its state.

Note: Remote Sensors should not maintain a persistent connection. Instead they
should connect when a change occurs that should be reported. It is not necessary
for Remote Sensors to report when they are INACTIVE. If a Remote Sensor does not
report within REMOTE_SENSORS_DECAY milliseconds the command station will
automatically transition the Remote Sensor to INACTIVE state if it was
previously ACTIVE.

The following varations of the "RS" command :

  <RS ID STATE>:      Informs the command station of the status of a remote sensor.
  <RS ID>:            Deletes remote sensor ID.
  <RS>:               Lists all defined remote sensors.
                      returns: <RS ID STATE> for each defined remote sensor or
                      <X> if no remote sensors have been defined/found.
where

  ID:     the numeric ID (0-32667) of the remote sensor.
  STATE:  State of the sensors, zero is INACTIVE, non-zero is ACTIVE.
          Usage is remote sensor dependent.
**********************************************************************/

// TODO: merge this into the base SensorManager code.

std::vector<std::unique_ptr<RemoteSensor>> remoteSensors;

void RemoteSensorManager::init()
{
}

void RemoteSensorManager::createOrUpdate(const uint16_t id, const uint16_t value) {
  // check for duplicate ID
  for (const auto& sensor : remoteSensors)
  {
    if(sensor->getRawID() == id)
    {
      sensor->setSensorValue(value);
      return;
    }
  }
  remoteSensors.push_back(std::make_unique<RemoteSensor>(id, value));
}

bool RemoteSensorManager::remove(const uint16_t id)
{
  auto ent = std::find_if(remoteSensors.begin(), remoteSensors.end(),
  [id](std::unique_ptr<RemoteSensor> & sensor) -> bool
  {
    return sensor->getID() == id;
  });
  if (ent != remoteSensors.end())
  {
    remoteSensors.erase(ent);
    return true;
  }
  return false;
}

string RemoteSensorManager::getStateAsJson()
{
  string output = "[";
  for (const auto& sensor : remoteSensors)
  {
    if (output.length() > 1)
    {
      output += ",";
    }
    output += sensor->toJson();
  }
  output += "]";
  return output;
}

string RemoteSensorManager::get_state_for_dccpp()
{
  if (remoteSensors.empty())
  {
    return COMMAND_FAILED_RESPONSE;
  }
  string status;
  for (const auto& sensor : remoteSensors)
  {
    status += sensor->get_state_for_dccpp();
  }
  return status;
}

RemoteSensor::RemoteSensor(uint16_t id, uint16_t value) :
  Sensor(id + CONFIG_REMOTE_SENSORS_FIRST_SENSOR, NON_STORED_SENSOR_PIN, false, false), _rawID(id)
{
  setSensorValue(value);
  LOG(CONFIG_GPIO_SENSOR_LOG_LEVEL
    , "[RemoteSensors] RemoteSensor(%d) created with Sensor(%d), active: %s, value: %d"
    , getRawID(), getID(), isActive() ? JSON_VALUE_TRUE : JSON_VALUE_FALSE, value);
}

void RemoteSensor::check()
{
  if(isActive() && (esp_timer_get_time() / 1000ULL) > _lastUpdate + CONFIG_REMOTE_SENSORS_DECAY)
  {
    LOG(INFO, "[RemoteSensors] RemoteSensor(%d) expired, deactivating", getRawID());
    setSensorValue(0);
  }
}

string RemoteSensor::get_state_for_dccpp()
{
  return StringPrintf("<RS %d %d>", getRawID(), _value);
}

string RemoteSensor::toJson(bool includeState)
{
  nlohmann::json object =
  {
    { JSON_ID_NODE, getRawID() },
    { JSON_VALUE_NODE, getSensorValue() },
    { JSON_STATE_NODE, isActive() },
    { JSON_LAST_UPDATE_NODE, getLastUpdate() },
    { JSON_PIN_NODE, getPin() },
    { JSON_PULLUP_NODE, isPullUp() },
  };
  return object.dump();
}

DCC_PROTOCOL_COMMAND_HANDLER(RemoteSensorsCommandAdapter,
[](const vector<string> arguments)
{
  if(arguments.empty())
  {
    // list all sensors
    return RemoteSensorManager::get_state_for_dccpp();
  }
  else
  {
    uint16_t sensorID = std::stoi(arguments[0]);
    if (arguments.size() == 1 && RemoteSensorManager::remove(sensorID))
    {
      // delete remote sensor
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
    else if (arguments.size() == 2)
    {
      // create/update remote sensor
      RemoteSensorManager::createOrUpdate(sensorID, std::stoi(arguments[1]));
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

#endif // CONFIG_GPIO_SENSORS
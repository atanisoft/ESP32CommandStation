/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

#include <ConfigurationManager.h>
#include <DCCppProtocol.h>
#include <driver/gpio.h>
#include <json.hpp>
#include <JsonConstants.h>
#include <StatusDisplay.h>

#include "Sensors.h"
#include "RemoteSensors.h"

std::vector<std::unique_ptr<Sensor>> sensors;

TaskHandle_t SensorManager::_taskHandle;
OSMutex SensorManager::_lock;
static constexpr UBaseType_t SENSOR_TASK_PRIORITY = 1;
static constexpr uint32_t SENSOR_TASK_STACK_SIZE = 2048;

static constexpr const char * SENSORS_JSON_FILE = "sensors.json";

void SensorManager::init()
{
  LOG(INFO, "[Sensors] Initializing sensors");
  nlohmann::json root = nlohmann::json::parse(Singleton<ConfigurationManager>::instance()->load(SENSORS_JSON_FILE));
  if(root.contains(JSON_COUNT_NODE))
  {
    uint16_t sensorCount = root[JSON_COUNT_NODE].get<uint16_t>();
    Singleton<StatusDisplay>::instance()->status("Found %02d Sensors", sensorCount);
    for(auto sensor : root[JSON_SENSORS_NODE])
    {
      string data = sensor.dump();
      sensors.push_back(std::make_unique<Sensor>(data));
    }
  }
  LOG(INFO, "[Sensors] Loaded %d sensors", sensors.size());
  xTaskCreate(sensorTask, "SensorManager", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &_taskHandle);
}

void SensorManager::clear()
{
  sensors.clear();
}

uint16_t SensorManager::store()
{
  nlohmann::json root;
  uint16_t sensorStoredCount = 0;
  for (const auto& sensor : sensors)
  {
    if (sensor->getPin() != NON_STORED_SENSOR_PIN)
    {
      root[JSON_SENSORS_NODE].push_back(sensor->toJson());
      sensorStoredCount++;
    }
  }
  root[JSON_COUNT_NODE] = sensorStoredCount;
  Singleton<ConfigurationManager>::instance()->store(SENSORS_JSON_FILE, root.dump());
  return sensorStoredCount;
}

void SensorManager::sensorTask(void *param)
{
  while(true)
  {
    {
      OSMutexLock l(&_lock);
      for (const auto& sensor : sensors)
      {
        if (sensor->getPin() != NON_STORED_SENSOR_PIN)
        {
          sensor->check();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

string SensorManager::getStateAsJson()
{
  OSMutexLock l(&_lock);
  string status = "[";
  for (const auto& sensor : sensors)
  {
    if (status.length() > 1)
    {
      status += ",";
    }
    status += sensor->toJson(true);
  }
  status += "]";
  return status;
}

Sensor *SensorManager::getSensor(uint16_t id)
{
  OSMutexLock l(&_lock);
  const auto & ent = std::find_if(sensors.begin(), sensors.end(),
  [id](std::unique_ptr<Sensor> & sensor) -> bool
  {
    return sensor->getID() == id;
  });
  if (ent != sensors.end())
  {
    return ent->get();
  }
  return nullptr;
}

bool SensorManager::createOrUpdate(const uint16_t id, const gpio_num_t pin, const bool pullUp)
{
  if(is_restricted_pin(pin))
  {
    return false;
  }
  OSMutexLock l(&_lock);
  auto sens = getSensor(id);
  if (sens)
  {
    sens->update(pin, pullUp);
    return true;
  }
  // add the new sensor
  sensors.push_back(std::make_unique<Sensor>(id, pin, pullUp));
  return true;
}

bool SensorManager::remove(const uint16_t id)
{
  OSMutexLock l(&_lock);
  const auto & ent = std::find_if(sensors.begin(), sensors.end(),
  [id](std::unique_ptr<Sensor> & sensor) -> bool
  {
    return sensor->getID() == id;
  });
  if (ent != sensors.end())
  {
    LOG(INFO, "[Sensors] Removing Sensor(%d)", (*ent)->getID());
    sensors.erase(ent);
    return true;
  }
  return false;
}

gpio_num_t SensorManager::getSensorPin(const uint16_t id)
{
  auto sens = getSensor(id);
  if (sens)
  {
    return sens->getPin();
  }
  return NON_STORED_SENSOR_PIN;
}

string SensorManager::get_state_for_dccpp()
{
  string res;
  for (const auto &sensor : sensors)
  {
    res += sensor->get_state_for_dccpp();
  }
  return res;
}

Sensor::Sensor(uint16_t sensorID, gpio_num_t pin, bool pullUp, bool announce, bool initialState)
  : _sensorID(sensorID), _pin(pin), _pullUp(pullUp), _lastState(initialState)
{
  if (_pin != NON_STORED_SENSOR_PIN)
  {
    if (announce)
    {
      LOG(CONFIG_GPIO_SENSOR_LOG_LEVEL
        , "[Sensors] Sensor(%d) on pin %d created, pullup %s", _sensorID, _pin
        , _pullUp ? "Enabled" : "Disabled");
    }
    gpio_pad_select_gpio(_pin);
    ESP_ERROR_CHECK(gpio_set_direction(_pin, GPIO_MODE_INPUT));
    if (pullUp)
    {
      ESP_ERROR_CHECK(gpio_pullup_en(_pin));
    }
  }
}

Sensor::Sensor(string &data) : _lastState(false)
{
  nlohmann::json object = nlohmann::json::parse(data);
  _sensorID = object[JSON_ID_NODE];
  _pin = (gpio_num_t)object[JSON_PIN_NODE];
  _pullUp = object[JSON_PULLUP_NODE];
  LOG(CONFIG_GPIO_SENSOR_LOG_LEVEL
    , "[Sensors] Sensor(%d) on pin %d loaded, pullup %s", _sensorID, _pin
    , _pullUp ? "Enabled" : "Disabled");
  gpio_pad_select_gpio(_pin);
  ESP_ERROR_CHECK(gpio_set_direction(_pin, GPIO_MODE_INPUT));
  if (_pullUp)
  {
    ESP_ERROR_CHECK(gpio_pullup_en(_pin));
  }
}

string Sensor::toJson(bool includeState)
{
  nlohmann::json object =
  {
    { JSON_ID_NODE, _sensorID },
    { JSON_PIN_NODE, (uint8_t)_pin },
    { JSON_PULLUP_NODE, _pullUp },
  };
  if (includeState)
  {
    object[JSON_STATE_NODE] = _lastState;
  }
  return object.dump();
}

void Sensor::update(gpio_num_t pin, bool pullUp)
{
  ESP_ERROR_CHECK(gpio_reset_pin(_pin));
  _pin = pin;
  _pullUp = pullUp;
  LOG(CONFIG_GPIO_SENSOR_LOG_LEVEL
    , "[Sensors] Sensor(%d) on pin %d updated, pullup %s", _sensorID, _pin
    , _pullUp ? "Enabled" : "Disabled");
  gpio_pad_select_gpio(_pin);
  ESP_ERROR_CHECK(gpio_set_direction(_pin, GPIO_MODE_INPUT));
  if (_pullUp)
  {
    ESP_ERROR_CHECK(gpio_pullup_en(_pin));
  }
}

void Sensor::check()
{
  set(gpio_get_level(_pin));
}

string Sensor::get_state_for_dccpp()
{
  return StringPrintf("<Q %d %d %d>", _sensorID, _pin, _pullUp);
}

string Sensor::set(bool state)
{
  if (_lastState != state)
  {
    _lastState = state;
    LOG(INFO, "Sensor: %d :: %s", _sensorID, _lastState ? "ACTIVE" : "INACTIVE");
    // TODO: find a way to send this out on the JMRI interface
    return StringPrintf("<%c %d>", state ? 'Q' : 'q', _sensorID);
  }
  return COMMAND_NO_RESPONSE;
}


/**********************************************************************
  <S ID PIN PULLUP>:    creates a new sensor ID, with specified PIN and PULLUP
                        if sensor ID already exists, it is updated with
                        specificed PIN and PULLUP.
        returns: <O> if successful and <X> if unsuccessful (e.g. out of memory)

  <S ID>:               deletes definition of sensor ID.
        returns: <O> if successful and <X> if unsuccessful (e.g. ID does not exist)

  <S>:                  lists all defined sensors.
        returns: <Q ID PIN PULLUP> for each defined sensor or <X> if no sensors
        defined

where

  ID:     the numeric ID (0-32767) of the sensor
  PIN:    the pin number the sensor is connected to
  PULLUP: 1=use internal pull-up resistor for PIN, 0=don't use internal pull-up
          resistor for PIN

  <Q ID>     - for transition of Sensor ID from HIGH state to LOW state
               (i.e. the sensor is triggered)
  <q ID>     - for transition of Sensor ID from LOW state to HIGH state
               (i.e. the sensor is no longer triggered)
**********************************************************************/

DCC_PROTOCOL_COMMAND_HANDLER(SensorCommandAdapter,
[](const vector<string> arguments)
{
  if(arguments.empty())
  {
    // list all sensors
    string status = SensorManager::get_state_for_dccpp();
    status += RemoteSensorManager::get_state_for_dccpp();
    return status;
  }
  else
  {
    uint16_t sensorID = std::stoi(arguments[0]);
    if (arguments.size() == 1 && SensorManager::remove(sensorID))
    {
      // delete turnout
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
    else if (arguments.size() == 3)
    {
      // create sensor
      SensorManager::createOrUpdate(sensorID
                                  , (gpio_num_t)std::stoi(arguments[1])
                                  , arguments[2][0] == '1');
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})
#endif // CONFIG_GPIO_SENSORS
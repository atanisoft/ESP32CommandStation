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

/**********************************************************************

The ESP32 Command Station supports multiple S88 Sensor busses.

To have the command station monitor an S88 sensor, first define/edit/delete
an S88 Sensor Bus using the following variations on the "S88" command:
  <S88 ID DATAPIN COUNT> : Creates an S88 Sensor Bus with the specified
                           ID, DATA PIN, SENSOR COUNT.
        returns: <O> if successful and <X> if unsuccessful.
  <S88 ID>               : Deletes definition of S88 Sensor Bus ID and all
                           associated S88 Sensors on the bus.
        returns: <O> if successful and <X> if unsuccessful.
  <S88>                  : Lists all S88 Sensor Busses and state of sensors.
        returns: <S88 ID DATAPIN> for each S88 Sensor Bus or <X>
        if no busses have been defined
Note: S88 Sensor Busses will create individual sensors that report via <S> but
they can not be edited/deleted via <S> commands. Attempts to do that will result
in an <X> being returned.

S88 Sensors are reported in the same manner as generic Sensors:
  <Q ID>     - for activation of S88 Sensor ID.
  <q ID>     - for deactivation of S88 Sensor ID.

**********************************************************************/
#include "sdkconfig.h"

#if defined(CONFIG_GPIO_S88)
#include <FileSystemManager.h>
#include <DCCppProtocol.h>
#include <driver/gpio.h>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <json.hpp>
#include <JsonConstants.h>
#include <os/OS.hxx>
#include <utils/GpioInitializer.hxx>
#include <utils/StringPrintf.hxx>

#include "GPIOValidation.h"
#include "S88Sensors.h"

/////////////////////////////////////////////////////////////////////////////////////
// S88 Timing values (in microseconds)
/////////////////////////////////////////////////////////////////////////////////////
constexpr uint16_t S88_SENSOR_LOAD_PRE_CLOCK_TIME = 50;
constexpr uint16_t S88_SENSOR_LOAD_POST_RESET_TIME = 50;
constexpr uint16_t S88_SENSOR_CLOCK_PULSE_TIME = 50;
constexpr uint16_t S88_SENSOR_CLOCK_PRE_RESET_TIME = 50;
constexpr uint16_t S88_SENSOR_RESET_PULSE_TIME = 50;
constexpr uint16_t S88_SENSOR_READ_TIME = 25;

static constexpr const char * S88_SENSORS_JSON_FILE = "s88.json";

GPIO_PIN(S88_CLOCK, GpioOutputSafeLow, CONFIG_GPIO_S88_CLOCK_PIN);
GPIO_PIN(S88_LOAD, GpioOutputSafeLow, CONFIG_GPIO_S88_LOAD_PIN);
#if CONFIG_GPIO_S88_RESET_PIN >= 0
GPIO_PIN(S88_RESET, GpioOutputSafeLow, CONFIG_GPIO_S88_RESET_PIN);
#else
typedef DummyPin S88_RESET_Pin;
#endif

typedef GpioInitializer<S88_CLOCK_Pin, S88_LOAD_Pin, S88_RESET_Pin> S88PinInit;

void *s88_task(void *param)
{
  S88BusManager *s88 = static_cast<S88BusManager *>(param);
  while (true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    s88->poll();
  }
}

S88BusManager::S88BusManager(openlcb::Node *node) : poller_(node, {this})
{
#if CONFIG_GPIO_S88_RESET_PIN >= 0
  LOG(INFO, "[S88] Configuration (clock: %d, reset: %d, load: %d)"
    , S88_CLOCK_Pin::pin(), S88_RESET_Pin::pin(), S88_LOAD_Pin::pin());
#else
  LOG(INFO, "[S88] Configuration (clock: %d, load: %d)"
    , S88_CLOCK_Pin::pin(), S88_LOAD_Pin::pin());
#endif

  S88PinInit::hw_init();

  LOG(INFO, "[S88] Initializing SensorBus list");
  nlohmann::json root = nlohmann::json::parse(
    Singleton<FileSystemManager>::instance()->load(S88_SENSORS_JSON_FILE));
  for (auto bus : root[JSON_SENSORS_NODE])
  {
    buses_.push_back(
      std::make_unique<S88SensorBus>(bus[JSON_ID_NODE], bus[JSON_PIN_NODE]
                                   , bus[JSON_COUNT_NODE]));
  }
  LOG(INFO, "[S88] Loaded %d Sensor Buses", buses_.size());
  os_thread_create(&taskHandle_, "s88", 1, 2048, s88_task, this);
}

S88BusManager::~S88BusManager()
{
  poller_.stop();
  vTaskDelete(taskHandle_);
}

void S88BusManager::clear()
{
  buses_.clear();
}

uint16_t S88BusManager::store()
{
  AtomicHolder l(this);
  uint16_t count = 0;
  string content = "[";
  for (const auto& bus : buses_)
  {
    // only add the seperator if we have already serialized at least one
    // bus.
    if (content.length() > 1)
    {
      content += ",";
    }
    content += bus->toJson();
    count += bus->getSensorCount();
  }
  content += "]";
  Singleton<FileSystemManager>::instance()->store(S88_SENSORS_JSON_FILE
                                                , content);
  return count;
}

void S88BusManager::poll_33hz(openlcb::WriteHelper *helper, Notifiable *done)
{
  AutoNotify n(done);

  // wake up background task for polling
  xTaskNotifyGive(taskHandle_);
}

void S88BusManager::poll()
{
  AtomicHolder l(this);
  for (const auto& sensorBus : buses_)
  {
    sensorBus->prepForRead();
  }
  S88_LOAD_Pin::set(true);
  ets_delay_us(S88_SENSOR_LOAD_PRE_CLOCK_TIME);
  S88_CLOCK_Pin::set(true);
  ets_delay_us(S88_SENSOR_CLOCK_PULSE_TIME);
  S88_CLOCK_Pin::set(false);
  ets_delay_us(S88_SENSOR_CLOCK_PRE_RESET_TIME);
  S88_RESET_Pin::set(true);
  ets_delay_us(S88_SENSOR_RESET_PULSE_TIME);
  S88_RESET_Pin::set(false);
  ets_delay_us(S88_SENSOR_LOAD_POST_RESET_TIME);
  S88_LOAD_Pin::set(false);

  ets_delay_us(S88_SENSOR_READ_TIME);
  bool keepReading = true;
  while (keepReading)
  {
    keepReading = false;
    for (const auto& sensorBus : buses_)
    {
      if (sensorBus->hasMore())
      {
        keepReading = true;
        sensorBus->readNext();
      }
    }
    S88_CLOCK_Pin::set(true);
    ets_delay_us(S88_SENSOR_CLOCK_PULSE_TIME);
    S88_CLOCK_Pin::set(false);
    ets_delay_us(S88_SENSOR_READ_TIME);
  }
}

bool S88BusManager::createOrUpdateBus(const uint8_t id, const gpio_num_t dataPin, const uint16_t sensorCount)
{
  // check for duplicate data pin
  for (const auto& sensorBus : buses_)
  {
    if (sensorBus->getID() != id && sensorBus->getDataPin() == dataPin)
    {
      LOG_ERROR("[S88] Bus %d is already using data pin %d, rejecting create/update of S88 Bus %d",
        sensorBus->getID(), dataPin, id);
      return false;
    }
  }
  AtomicHolder l(this);
  // check for existing bus to be updated
  for (const auto& sensorBus : buses_)
  {
    if (sensorBus->getID() == id)
    {
      sensorBus->update(dataPin, sensorCount);
      return true;
    }
  }
  if (is_restricted_pin(dataPin))
  {
    LOG_ERROR("[S88] Attempt to use a restricted pin: %d", dataPin);
    return false;
  }
  buses_.push_back(std::make_unique<S88SensorBus>(id, dataPin, sensorCount));
  return true;
}

bool S88BusManager::removeBus(const uint8_t id)
{
  AtomicHolder l(this);
  const auto & ent = std::find_if(buses_.begin(), buses_.end(),
  [id](std::unique_ptr<S88SensorBus> & bus) -> bool
  {
    return bus->getID() == id;
  });
  if (ent != buses_.end())
  {
    buses_.erase(ent);
    return true;
  }
  return false;
}

string S88BusManager::get_state_as_json()
{
  AtomicHolder l(this);
  string state = "[";
  for (const auto& sensorBus : buses_)
  {
    if (state.length() > 1)
    {
      state += ",";
    }
    state += sensorBus->toJson(true);
  }
  state += "]";
  return state;
}

string S88BusManager::get_state_for_dccpp()
{
  string res;
  for (const auto& sensorBus : buses_)
  {
    res += sensorBus->get_state_for_dccpp();
  }
  return res;
}

S88SensorBus::S88SensorBus(const uint8_t id, const gpio_num_t dataPin, const uint16_t sensorCount) :
  _id(id), _dataPin(dataPin), _sensorIDBase((id * CONFIG_GPIO_S88_SENSORS_PER_BUS) + CONFIG_GPIO_S88_FIRST_SENSOR),
  _lastSensorID((id * CONFIG_GPIO_S88_SENSORS_PER_BUS) + CONFIG_GPIO_S88_FIRST_SENSOR)
{
  LOG(INFO, "[S88 Bus-%d] Created using data pin %d with %d sensors starting at id %d",
    _id, _dataPin, sensorCount, _sensorIDBase);
  gpio_pad_select_gpio((gpio_num_t)_dataPin);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)_dataPin, GPIO_MODE_INPUT));
  if (sensorCount > 0)
  {
    addSensors(sensorCount);
  }
}

void S88SensorBus::update(const gpio_num_t dataPin, const uint16_t sensorCount)
{
  _dataPin = dataPin;
  _lastSensorID = _sensorIDBase;
  gpio_pad_select_gpio(_dataPin);
  ESP_ERROR_CHECK(gpio_set_direction(_dataPin, GPIO_MODE_INPUT));  
  for (const auto& sensor : _sensors)
  {
    sensor->updateID(_lastSensorID++);
  }
  if (sensorCount < _sensors.size())
  {
    removeSensors(_sensors.size() - sensorCount);
  }
  else if (sensorCount > 0)
  {
    addSensors(sensorCount - _sensors.size());
  }
  LOG(INFO, "[S88 Bus-%d] Updated to use data pin %d with %d sensors",
    _id, _dataPin, _sensors.size());
}

string S88SensorBus::toJson(bool includeState)
{
  string serialized = StringPrintf(
    "{\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d"
  , JSON_ID_NODE, _id, JSON_PIN_NODE, _dataPin
  , JSON_S88_SENSOR_BASE_NODE, _sensorIDBase, JSON_COUNT_NODE, _sensors.size());
  if (includeState)
  {
    serialized += StringPrintf(",\"%s\":\"%s\"", JSON_STATE_NODE, getStateString().c_str());
  }
  serialized += "}";
  return serialized;
}

void S88SensorBus::addSensors(int16_t sensorCount)
{
  const uint16_t startingIndex = _sensors.size();
  for (uint8_t id = 0; id < sensorCount; id++)
  {
    _sensors.push_back(new S88Sensor(_lastSensorID++, startingIndex + id));
  }
}

void S88SensorBus::removeSensors(int16_t sensorCount)
{
  if (sensorCount < 0)
  {
    for (const auto& sensor : _sensors)
    {
      LOG(CONFIG_GPIO_S88_SENSOR_LOG_LEVEL, "[S88] Sensor(%d) removed"
        , sensor->getID());
    }
    _sensors.clear();
  }
  else
  {
    for (uint8_t id = 0; id < sensorCount; id++)
    {
      LOG(CONFIG_GPIO_S88_SENSOR_LOG_LEVEL, "[S88] Sensor(%d) removed"
        , _sensors.back()->getID());
      _sensors.pop_back();
    }
  }
}

string S88SensorBus::getStateString()
{
  string state = "";
  for (const auto& sensor : _sensors)
  {
    if (sensor->isActive())
    {
      state += "1";
    }
    else
    {
      state += "0";
    }
  }
  return state;
}

void S88SensorBus::readNext()
{
  // sensors need to pull pin LOW for ACTIVE
  _sensors[_nextSensorToRead++]->setState(gpio_get_level(_dataPin));
}

string S88SensorBus::get_state_for_dccpp()
{
  string status = StringPrintf("<S88 %d %d %d>", _id, _dataPin, _sensors.size());
  LOG(CONFIG_GPIO_S88_SENSOR_LOG_LEVEL
    , "[S88 Bus-%d] Data:%d, Base:%d, Count:%d:"
    , _id, _dataPin, _sensorIDBase, _sensors.size());
  for (const auto& sensor : _sensors)
  {
    LOG(CONFIG_GPIO_S88_SENSOR_LOG_LEVEL, "[S88] Input: %d :: %s"
      , sensor->getIndex(), sensor->isActive() ? "ACTIVE" : "INACTIVE");
    status += sensor->get_state_for_dccpp();
  }
  return status;
}

S88Sensor::S88Sensor(uint16_t id, uint16_t index)
  : Sensor(id, NON_STORED_SENSOR_PIN, false, false, true), _index(index)
{
  LOG(CONFIG_GPIO_S88_SENSOR_LOG_LEVEL
    , "[S88] Sensor(%d) created with index %d", id, _index);
  
}

DCC_PROTOCOL_COMMAND_HANDLER(S88BusCommandAdapter,
[](const vector<string> arguments)
{
  auto s88 = S88BusManager::instance();
  if (arguments.empty())
  {
    // list all sensor groups
    return s88->get_state_for_dccpp();
  }
  else
  {
    if (arguments.size() == 1 &&
        s88->removeBus(std::stoi(arguments[0])))
    {
      // delete sensor bus
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
    else if (arguments.size() == 3 &&
             s88->createOrUpdateBus(std::stoi(arguments[0])
                                  , (gpio_num_t)std::stoi(arguments[1])
                                  , std::stoi(arguments[2])))
    {
      // create sensor bus
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

#endif // CONFIG_GPIO_S88

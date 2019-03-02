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

#include "DCCppESP32.h"
#include "S88Sensors.h"

/**********************************************************************

DCC++ESP32 COMMAND STATION supports multiple S88 Sensor busses.

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
#if S88_ENABLED

#ifndef S88_FIRST_SENSOR
#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS
#endif

extern LinkedList<Sensor *> sensors;

TaskHandle_t S88BusManager::_taskHandle;
xSemaphoreHandle S88BusManager::_s88SensorLock;

static constexpr UBaseType_t S88_SENSOR_TASK_PRIORITY = 1;

LinkedList<S88SensorBus *> s88SensorBus([](S88SensorBus *sensorBus) {
  sensorBus->removeSensors(-1);
  log_v("S88SensorBus(%d) removed", sensorBus->getID());
  delete sensorBus;
});

void S88BusManager::init() {
  pinMode(S88_CLOCK_PIN, OUTPUT);
  pinMode(S88_RESET_PIN, OUTPUT);
  pinMode(S88_LOAD_PIN, OUTPUT);

  log_v("Initializing S88 SensorBus list");
  JsonObject &root = configStore.load(S88_SENSORS_JSON_FILE);
  JsonVariant count = root[JSON_COUNT_NODE];
  uint16_t s88BusCount = count.success() ? count.as<int>() : 0;
  log_v("Found %d S88 Busses", s88BusCount);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d S88 Bus"), s88BusCount);
  if(s88BusCount > 0) {
    for(auto bus : root.get<JsonArray>(JSON_SENSORS_NODE)) {
      s88SensorBus.add(new S88SensorBus(bus.as<JsonObject &>()));
    }
  }
  _s88SensorLock = xSemaphoreCreateMutex();
  xTaskCreate(s88SensorTask, "S88SensorManager", DEFAULT_THREAD_STACKSIZE, NULL, S88_SENSOR_TASK_PRIORITY, &_taskHandle);
}

void S88BusManager::clear() {
  MUTEX_LOCK(_s88SensorLock);
  s88SensorBus.free();
  store();
  MUTEX_UNLOCK(_s88SensorLock);
}

uint8_t S88BusManager::store() {
  JsonObject &root = configStore.createRootNode();
  JsonArray &array = root.createNestedArray(JSON_SENSORS_NODE);
  uint8_t sensorBusIndex = 0;
  for (const auto& bus : s88SensorBus) {
    bus->toJson(array.createNestedObject());
    sensorBusIndex++;
  }
  root[JSON_COUNT_NODE] = sensorBusIndex;
  configStore.store(S88_SENSORS_JSON_FILE, root);
  return sensorBusIndex;
}

void S88BusManager::s88SensorTask(void *param) {
  while(true) {
    MUTEX_LOCK(_s88SensorLock);
    for (const auto& sensorBus : s88SensorBus) {
      sensorBus->prepForRead();
    }
    digitalWrite(S88_LOAD_PIN, HIGH);
    delayMicroseconds(S88_SENSOR_LOAD_PRE_CLOCK_TIME);
    digitalWrite(S88_CLOCK_PIN, HIGH);
    delayMicroseconds(S88_SENSOR_CLOCK_PULSE_TIME);
    digitalWrite(S88_CLOCK_PIN, LOW);
    delayMicroseconds(S88_SENSOR_CLOCK_PRE_RESET_TIME);
    digitalWrite(S88_RESET_PIN, HIGH);
    delayMicroseconds(S88_SENSOR_RESET_PULSE_TIME);
    digitalWrite(S88_RESET_PIN, LOW);
    delayMicroseconds(S88_SENSOR_LOAD_POST_RESET_TIME);
    digitalWrite(S88_LOAD_PIN, LOW);

    delayMicroseconds(S88_SENSOR_READ_TIME);
    bool keepReading = true;
    while(keepReading) {
      keepReading = false;
      for (const auto& sensorBus : s88SensorBus) {
        if(sensorBus->hasMore()) {
          keepReading = true;
          sensorBus->readNext();
        }
      }
      digitalWrite(S88_CLOCK_PIN, HIGH);
      delayMicroseconds(S88_SENSOR_CLOCK_PULSE_TIME);
      digitalWrite(S88_CLOCK_PIN, LOW);
      delayMicroseconds(S88_SENSOR_READ_TIME);
    }
    MUTEX_UNLOCK(_s88SensorLock);
  }
}

bool S88BusManager::createOrUpdateBus(const uint8_t id, const uint8_t dataPin, const uint16_t sensorCount) {
  // check for duplicate data pin
  for (const auto& sensorBus : s88SensorBus) {
    if(sensorBus->getID() != id && sensorBus->getDataPin() == dataPin) {
      log_e("S88 Bus %d is already using data pin %d, rejecting create/update of S88 Bus %d",
        sensorBus->getID(), dataPin, id);
      return false;
    }
  }
  MUTEX_LOCK(_s88SensorLock);
  // check for existing bus to be updated
  for (const auto& sensorBus : s88SensorBus) {
    if(sensorBus->getID() == id) {
      sensorBus->update(dataPin, sensorCount);
      MUTEX_UNLOCK(_s88SensorLock);
      return true;
    }
  }
  if(std::find(restrictedPins.begin(), restrictedPins.end(), dataPin) != restrictedPins.end()) {
    MUTEX_UNLOCK(_s88SensorLock);
    return false;
  }
  s88SensorBus.add(new S88SensorBus(id, dataPin, sensorCount));
  MUTEX_UNLOCK(_s88SensorLock);
  return true;
}

bool S88BusManager::removeBus(const uint8_t id) {
  MUTEX_LOCK(_s88SensorLock);
  S88SensorBus *sensorBusToRemove = nullptr;
  for (const auto& sensorBus : s88SensorBus) {
    if(sensorBus->getID() == id) {
      sensorBusToRemove = sensorBus;
    }
  }
  if(sensorBusToRemove != nullptr) {
    log_v("Removing S88 Sensor Bus(%d)", sensorBusToRemove->getID());
    s88SensorBus.remove(sensorBusToRemove);
    MUTEX_UNLOCK(_s88SensorLock);
    return true;
  }
  MUTEX_UNLOCK(_s88SensorLock);
  return false;
}

void S88BusManager::getState(JsonArray &array) {
  for (const auto& sensorBus : s88SensorBus) {
    JsonObject &sensorJson = array.createNestedObject();
    sensorBus->toJson(sensorJson, true);
  }
}

S88SensorBus::S88SensorBus(const uint8_t id, const uint8_t dataPin, const uint16_t sensorCount) :
  _id(id), _dataPin(dataPin), _sensorIDBase((id * S88_MAX_SENSORS_PER_BUS) + S88_FIRST_SENSOR), _lastSensorID((id * S88_MAX_SENSORS_PER_BUS) + S88_FIRST_SENSOR) {
  log_v("S88SensorBus(%d) created on pin %d with %d sensors starting at id %d",
    _id, _dataPin, sensorCount, _sensorIDBase);
  pinMode(_dataPin, INPUT);
  if(sensorCount > 0) {
    addSensors(sensorCount);
  }
}

S88SensorBus::S88SensorBus(JsonObject &json) {
  _id = json[JSON_ID_NODE];
  _dataPin = json[JSON_PIN_NODE];
  _lastSensorID = _sensorIDBase = json[JSON_S88_SENSOR_BASE_NODE];
  uint16_t sensorCount = json[JSON_COUNT_NODE];
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("S88(%d) %02d sensors"), _id, sensorCount);
  for(uint16_t id = 0; id < sensorCount; id++) {
    _sensors.push_back(new S88Sensor(_sensorIDBase + id, id));
  }
  for (const auto& sensor : _sensors) {
    sensors.add(sensor);
  }
}

void S88SensorBus::update(const uint8_t dataPin, const uint16_t sensorCount) {
  _dataPin = dataPin;
  _lastSensorID = _sensorIDBase;
  pinMode(_dataPin, INPUT);
  for (const auto& sensor : _sensors) {
    sensor->updateID(_lastSensorID++);
  }
  if(sensorCount < _sensors.size()) {
    removeSensors(_sensors.size() - sensorCount);
  } else if(sensorCount > 0) {
    addSensors(sensorCount - _sensors.size());
  }
  log_v("S88SensorBus(%d) updated to use data pin %d, updating %d sensors",
    _id, _dataPin, _sensors.size());
  show();
}

void S88SensorBus::toJson(JsonObject &json, bool includeState) {
  json[JSON_ID_NODE] = _id;
  json[JSON_PIN_NODE] = _dataPin;
  json[JSON_S88_SENSOR_BASE_NODE] = _sensorIDBase;
  json[JSON_COUNT_NODE] = _sensors.size();
  if(includeState) {
    json[JSON_STATE_NODE] = getStateString();
  }
}

void S88SensorBus::addSensors(int16_t sensorCount) {
  const uint16_t startingIndex = _sensors.size();
  for(uint8_t id = 0; id < sensorCount; id++) {
    S88Sensor *newSensor = new S88Sensor(_lastSensorID++, startingIndex + id);
    _sensors.push_back(newSensor);
    sensors.add(newSensor);
  }
}

void S88SensorBus::removeSensors(int16_t sensorCount) {
  if(sensorCount < 0) {
    for (const auto& sensor : _sensors) {
      log_v("S88Sensor(%d) removed", sensor->getID());
      sensors.remove(sensor);
    }
    _sensors.clear();
  } else {
    for(uint8_t id = 0; id < sensorCount; id++) {
      S88Sensor *removedSensor = _sensors.back();
      log_v("S88Sensor(%d) removed", removedSensor->getID());
      _sensors.pop_back();
      sensors.remove(removedSensor);
    }
  }
}

String S88SensorBus::getStateString() {
  String state = "";
  for (const auto& sensor : _sensors) {
    if(sensor->isActive()) {
      state += "1";
    } else {
      state += "0";
    }
  }
  return state;
}

void S88SensorBus::readNext() {
  // sensors need to pull pin LOW for ACTIVE
  _sensors[_nextSensorToRead++]->setState(digitalRead(_dataPin) == HIGH);
}

void S88SensorBus::show() {
  wifiInterface.printf(F("<S88 %d %d %d>"), _id, _dataPin, _sensors.size());
  log_v("S88 Bus(%d, %d, %d, %d):", _id, _dataPin, _sensorIDBase, _sensors.size());
  for (const auto& sensor : _sensors) {
    log_v("Input: %d :: %s", sensor->getIndex(), sensor->isActive() ? "ACTIVE" : "INACTIVE");
    sensor->show();
  }
}

void S88BusCommandAdapter::process(const std::vector<String> arguments) {
  if(arguments.empty()) {
    // list all sensor groups
    for (const auto& sensorBus : s88SensorBus) {
      sensorBus->show();
    }
  } else {
    if (arguments.size() == 1 && S88BusManager::removeBus(arguments[0].toInt())) {
      // delete sensor bus
      wifiInterface.send(COMMAND_SUCCESSFUL_RESPONSE);
    } else if (arguments.size() == 3 && S88BusManager::createOrUpdateBus(arguments[0].toInt(), arguments[1].toInt(), arguments[2].toInt())) {
      // create sensor bus
      wifiInterface.send(COMMAND_SUCCESSFUL_RESPONSE);
    } else {
      wifiInterface.send(COMMAND_FAILED_RESPONSE);
    }
  }
}

S88Sensor::S88Sensor(uint16_t id, uint16_t index) : Sensor(id, NON_STORED_SENSOR_PIN, false, false), _index(index) {
  log_v("S88Sensor(%d) created with index %d", id, _index);
}
#endif

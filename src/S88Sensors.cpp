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

#include "DCCppESP32.h"
#include "S88Sensors.h"

/**********************************************************************

DCC++ESP32 BASE STATION supports multiple S88 Sensor busses.

To have the base station monitor an S88 sensor, first define/edit/delete an S88
Sensor Bus using the following variations on the "S88" command:
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
#if defined(S88_ENABLED) && S88_ENABLED
extern LinkedList<Sensor *> sensors;
LinkedList<S88SensorBus *> s88SensorBus([](S88SensorBus *sensorBus) {
  sensorBus->removeSensors(-1);
  log_i("S88SensorBus(%d) removed", sensorBus->getID());
  delete sensorBus;
});

void S88BusManager::init() {
  pinMode(S88_CLOCK_PIN, OUTPUT);
  pinMode(S88_RESET_PIN, OUTPUT);
  pinMode(S88_LOAD_PIN, OUTPUT);

  log_i("Initializing S88 SensorBus list");
  uint16_t s88BusCount = configStore.getUShort("S88BusCount", 0);
  log_i("Found %d S88 Busses", s88BusCount);
  for(int index = 0; index < s88BusCount; index++) {
    s88SensorBus.add(new S88SensorBus(index));
  }
}

void S88BusManager::clear() {
  configStore.putUShort("S88BusCount", 0);
  s88SensorBus.free();
}

uint8_t S88BusManager::store() {
  uint8_t sensorBusIndex = 0;
  for (const auto& bus : s88SensorBus) {
    bus->store(sensorBusIndex++);
  }
  configStore.putUShort("S88BusCount", sensorBusIndex);
  return sensorBusIndex;
}

void S88BusManager::update() {
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
  // check for existing bus to be updated
  for (const auto& sensorBus : s88SensorBus) {
    if(sensorBus->getID() == id) {
      sensorBus->update(dataPin, sensorCount);
      return true;
    }
  }
  s88SensorBus.add(new S88SensorBus(id, dataPin, sensorCount));
  return true;
}

bool S88BusManager::removeBus(const uint8_t id) {
  S88SensorBus *sensorBusToRemove = NULL;
  for (const auto& sensorBus : s88SensorBus) {
    if(sensorBus->getID() == id) {
      sensorBusToRemove = sensorBus;
    }
  }
  if(sensorBusToRemove != NULL) {
    s88SensorBus.remove(sensorBusToRemove);
    return true;
  }
  return false;
}

void S88BusManager::getState(JsonArray &array) {
  for (const auto& sensorBus : s88SensorBus) {
    JsonObject &sensorJson = array.createNestedObject();
    sensorJson[F("id")] = sensorBus->getID();
    sensorJson[F("dataPin")] = sensorBus->getDataPin();
    sensorJson[F("sensorIDBase")] = sensorBus->getSensorIDBase();
    sensorJson[F("sensorCount")] = sensorBus->getSensorCount();
    sensorJson[F("state")] = sensorBus->getStateString();
  }
}

S88SensorBus::S88SensorBus(const uint8_t id, const uint8_t dataPin, const uint16_t sensorCount) :
  _id(id), _dataPin(dataPin), _sensorIDBase((id * S88_SENSOR_ID_OFFSET) + S88_FIRST_SENSOR), _lastSensorID((id * S88_SENSOR_ID_OFFSET) + S88_FIRST_SENSOR) {
  log_i("S88SensorBus(%d) created on pin %d with %d sensors starting at id %d",
    _id, _dataPin, sensorCount, _sensorIDBase);
  pinMode(_dataPin, INPUT);
  if(sensorCount > 0) {
    addSensors(sensorCount);
  }
}

S88SensorBus::S88SensorBus(const uint16_t index) {
  String sensorIDKey = String("S88_") + String(index);
  String dataPinKey = sensorIDKey + String("_p");
  String sensorCountKey = sensorIDKey + String("_s");
  _id = configStore.getUShort(sensorIDKey.c_str(), index);
  _dataPin = configStore.getUChar(dataPinKey.c_str(), 0);
  _sensorIDBase = (_id * S88_SENSOR_ID_OFFSET) + S88_FIRST_SENSOR;
  _lastSensorID = _sensorIDBase;
  uint16_t sensorCount = configStore.getUShort(sensorCountKey.c_str(), 0);
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
  log_i("S88SensorBus(%d) updated to use data pin %d, updating %d sensors",
    _id, _dataPin, _sensors.size());
  show();
}

void S88SensorBus::store(const uint16_t index) {
  String sensorIDKey = String("S88_") + String(index);
  String dataPinKey = sensorIDKey + String("_p");
  String sensorCountKey = sensorIDKey + String("_s");
  configStore.putUShort(sensorIDKey.c_str(), _id);
  configStore.putUChar(dataPinKey.c_str(), _dataPin);
  configStore.putUShort(sensorCountKey.c_str(), _sensors.size());
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
      log_i("S88Sensor(%d) removed", sensor->getID());
      sensors.remove(sensor);
    }
    _sensors.clear();
  } else {
    for(uint8_t id = 0; id < sensorCount; id++) {
      S88Sensor *removedSensor = _sensors.back();
      log_i("S88Sensor(%d) removed", removedSensor->getID());
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
  log_i("S88 Bus(%d, %d, %d, %d):", _id, _dataPin, _sensorIDBase, _sensors.size());
  for (const auto& sensor : _sensors) {
    log_i("Input: %d :: %s", sensor->getIndex(), sensor->isActive() ? "ACTIVE" : "INACTIVE");
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
      wifiInterface.printf(F("<O>"));
    } else if (arguments.size() == 3 && S88BusManager::createOrUpdateBus(arguments[0].toInt(), arguments[1].toInt(), arguments[2].toInt())) {
      // create sensor bus
      wifiInterface.printf(F("<O>"));
    } else {
      wifiInterface.printf(F("<X>"));
    }
  }
}

S88Sensor::S88Sensor(uint16_t id, uint16_t index) : Sensor(id, -1, false, false), _index(index) {
  log_i("S88Sensor(%d) created with index %d", id, _index);
}
#endif

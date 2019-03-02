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

/**********************************************************************

DCC++ESP32 COMMAND STATION supports Sensor inputs that can be connected to any
unused ESP32 pin. Sensors can be of any type (infrared, magentic, mechanical...).
The only requirement is that when "activated" the Sensor must force the
specified pin LOW (i.e. to ground), and when not activated, this pin should
remain HIGH (e.g. 3.3V), or be allowed to float HIGH if use of the pin's
internal pull-up resistor is specified.  In addition to this type of sensor
the command station also supports S88-n connected sensors.

To ensure proper voltage levels, some part of the Sensor circuitry
MUST be tied back to the same ground as used by the ESP32.

To have the command station monitor one or more GPIO pins for sensor triggers, first
define/edit/delete sensor definitions using the following variation of the "S"
command:

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
  PIN:    the arduino pin number the sensor is connected to
  PULLUP: 1=use internal pull-up resistor for PIN, 0=don't use internal pull-up
          resistor for PIN

Once all sensors have been properly defined, use the <E> command to store their
definitions to the ESP32. If you later make edits/additions/deletions to the
sensor definitions, you must invoke the <E> command if you want those new
definitions updated on the ESP32. You can also clear everything stored on the
ESP32 by invoking the <e> command.

All sensors defined as per above are repeatedly and sequentially checked within
the main loop of this sketch. If a Sensor Pin is found to have transitioned from
one state to another, one of the following serial messages are generated:

  <Q ID>     - for transition of Sensor ID from HIGH state to LOW state
               (i.e. the sensor is triggered)
  <q ID>     - for transition of Sensor ID from LOW state to HIGH state
               (i.e. the sensor is no longer triggered)

Depending on whether the physical sensor is acting as an "event-trigger" or a
"detection-sensor," you may decide to ignore the <q ID> return and only react to
<Q ID> triggers.

**********************************************************************/

LinkedList<Sensor *> sensors([](Sensor *sensor) {delete sensor; });

TaskHandle_t SensorManager::_taskHandle;
xSemaphoreHandle SensorManager::_lock;
static constexpr UBaseType_t SENSOR_TASK_PRIORITY = 1;

void SensorManager::init() {
  _lock = xSemaphoreCreateMutex();
  log_v("Initializing sensors list");
  JsonObject &root = configStore.load(SENSORS_JSON_FILE);
  JsonVariant count = root[JSON_COUNT_NODE];
  uint16_t sensorCount = count.success() ? count.as<int>() : 0;
  log_v("Found %d sensors", sensorCount);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Sensors"), sensorCount);
  if(sensorCount > 0) {
    for(auto sensor : root.get<JsonArray>(JSON_SENSORS_NODE)) {
      sensors.add(new Sensor(sensor.as<JsonObject &>()));
    }
  }
  xTaskCreate(sensorTask, "SensorManager", DEFAULT_THREAD_STACKSIZE, NULL, SENSOR_TASK_PRIORITY, &_taskHandle);
}

void SensorManager::clear() {
  MUTEX_LOCK(_lock);
  sensors.free();
  store();
  MUTEX_UNLOCK(_lock);
}

uint16_t SensorManager::store() {
  JsonObject &root = configStore.createRootNode();
  JsonArray &array = root.createNestedArray(JSON_SENSORS_NODE);
  uint16_t sensorStoredCount = 0;
  for (const auto& sensor : sensors) {
    if(sensor->getPin() != NON_STORED_SENSOR_PIN) {
      sensor->toJson(array.createNestedObject());
      sensorStoredCount++;
    }
  }
  root[JSON_COUNT_NODE] = sensorStoredCount;
  configStore.store(SENSORS_JSON_FILE, root);
  return sensorStoredCount;
}

void SensorManager::sensorTask(void *param) {
  while(true) {
    MUTEX_LOCK(_lock);
    for (const auto& sensor : sensors) {
      sensor->check();
    }
    MUTEX_UNLOCK(_lock);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void SensorManager::getState(JsonArray & array) {
  for (const auto& sensor : sensors) {
    JsonObject &sensorJson = array.createNestedObject();
    sensor->toJson(sensorJson, true);
  }
}

Sensor *SensorManager::getSensor(uint16_t id) {
  for (const auto& sensor : sensors) {
    if(sensor->getID() == id && sensor->getPin() != -1) {
      return sensor;
    }
  }
  return nullptr;
}

bool SensorManager::createOrUpdate(const uint16_t id, const uint8_t pin, const bool pullUp) {
  MUTEX_LOCK(_lock);
  // check for duplicate ID or PIN
  for (const auto& sensor : sensors) {
    if(sensor->getID() == id) {
      sensor->update(pin, pullUp);
      MUTEX_UNLOCK(_lock);
      return true;
    }
  }
  if(std::find(restrictedPins.begin(), restrictedPins.end(), pin) != restrictedPins.end()) {
    MUTEX_UNLOCK(_lock);
    return false;
  }
  sensors.add(new Sensor(id, pin, pullUp));
  MUTEX_UNLOCK(_lock);
  return true;
}

bool SensorManager::remove(const uint16_t id) {
  MUTEX_LOCK(_lock);
  Sensor *sensorToRemove = nullptr;
  // check for duplicate ID or PIN
  for (const auto& sensor : sensors) {
    if(sensor->getID() == id) {
      sensorToRemove = sensor;
    }
  }
  if(sensorToRemove != nullptr) {
    log_v("Removing Sensor(%d)", sensorToRemove->getID());
    sensors.remove(sensorToRemove);
    MUTEX_UNLOCK(_lock);
    return true;
  }
  MUTEX_UNLOCK(_lock);
  return false;
}

uint8_t SensorManager::getSensorPin(const uint16_t id) {
  for (const auto& sensor : sensors) {
    if(sensor->getID() == id) {
      return sensor->getPin();
    }
  }
  return -1;
}

Sensor::Sensor(uint16_t sensorID, int8_t pin, bool pullUp, bool announce) : _sensorID(sensorID), _pin(pin), _pullUp(pullUp), _lastState(false) {
  if(announce) {
    log_v("Sensor(%d) on pin %d created, pullup %s", _sensorID, _pin, _pullUp ? "Enabled" : "Disabled");
    if(_pullUp) {
      pinMode(_pin, INPUT_PULLUP);
    } else {
      pinMode(_pin, INPUT);
    }
  }
}

Sensor::Sensor(JsonObject &json) : _lastState(false) {
  _sensorID = json[JSON_ID_NODE];
  _pin = json[JSON_PIN_NODE];
  _pullUp = json[JSON_PULLUP_NODE];
  log_v("Sensor(%d) on pin %d loaded, pullup %s", _sensorID, _pin, _pullUp ? "Enabled" : "Disabled");
  if(_pullUp) {
    pinMode(_pin, INPUT_PULLUP);
  } else {
    pinMode(_pin, INPUT);
  }
}

void Sensor::toJson(JsonObject &json, bool includeState) {
  json[JSON_ID_NODE] = _sensorID;
  json[JSON_PIN_NODE] = _pin;
  json[JSON_PULLUP_NODE] = _pullUp;
  if(includeState) {
    json[JSON_STATE_NODE] = _lastState;
  }
}

void Sensor::update(uint8_t pin, bool pullUp) {
  _pin = pin;
  _pullUp = pullUp;
  log_v("Sensor(%d) on pin %d updated, pullup %s", _sensorID, _pin, _pullUp ? "Enabled" : "Disabled");
  if(_pullUp) {
    pinMode(_pin, INPUT_PULLUP);
  } else {
    pinMode(_pin, INPUT);
  }
}

void Sensor::check() {
  set(digitalRead(_pin) == 1);
}

void Sensor::show() {
  wifiInterface.printf(F("<Q %d %d %d>"), _sensorID, _pin, _pullUp);
}

void SensorCommandAdapter::process(const std::vector<String> arguments) {
  if(arguments.empty()) {
    // list all sensors
    for (const auto& sensor : sensors) {
      sensor->show();
    }
  } else {
    uint16_t sensorID = arguments[0].toInt();
    if (arguments.size() == 1 && SensorManager::remove(sensorID)) {
      // delete turnout
      wifiInterface.send(COMMAND_SUCCESSFUL_RESPONSE);
    } else if (arguments.size() == 3) {
      // create sensor
      SensorManager::createOrUpdate(sensorID, arguments[1].toInt(), arguments[2].toInt() == 1);
      wifiInterface.send(COMMAND_SUCCESSFUL_RESPONSE);
    } else {
      wifiInterface.send(COMMAND_FAILED_RESPONSE);
    }
  }
}

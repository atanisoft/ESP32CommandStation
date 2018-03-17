/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2018 Mike Dunston
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

#include "DCCppESP32.h"
#include "RemoteSensors.h"

/**********************************************************************

DCC++ESP32 BASE STATION supports remote sensor inputs that are connected via a
WiFi connection. Remote Sensors are dynamically created during startup or by a
remote sensor reporting its state.

During startup, the base station scans for Access Points that have a name
starting with REMOTE_SENSORS_PREFIX defined in Config.h, ie: "sensor01". If no
Access Points are found matching this prefix during startup they will be created
automatically when the sensor reports its state to the base station.

Note: Remote Sensors should not maintain a persistent connection. Instead they
should connect when a change occurs that should be reported. It is not necessary
for Remote Sensors to report when they are INACTIVE. If a Remote Sensor does not
report within REMOTE_SENSORS_DECAY milliseconds the base station will
automatically transition the Remote Sensor to INACTIVE state if it was
previously ACTIVE.

The following varations of the "RS" command :

  <RS ID STATE>:      Informs the base station of the status of a remote sensor.
  <RS ID>:            Deletes remote sensor ID.
  <RS>:               Lists all defined remote sensors.
                      returns: <RS ID STATE> for each defined remote sensor or
                      <X> if no remote sensors have been defined/found.
where

  ID:     the numeric ID (0-32667) of the remote sensor.
  STATE:  State of the sensors, zero is INACTIVE, non-zero is ACTIVE.
          Usage is remote sensor dependent.
**********************************************************************/

// sanity check to ensure configuration has been setup correctly, default
// any missing parameters
#ifndef REMOTE_SENSORS_PREFIX
#define REMOTE_SENSORS_PREFIX "sensor"
#endif
#ifndef REMOTE_SENSORS_DECAY
#define REMOTE_SENSORS_DECAY 60000
#endif
#ifndef REMOTE_SENSORS_FIRST_SENSOR
#define REMOTE_SENSORS_FIRST_SENSOR 100
#endif

extern LinkedList<Sensor *> sensors;
LinkedList<RemoteSensor *> remoteSensors([](RemoteSensor *sensor) {
  log_i("RemoteSensor(%d) removed", sensor->getID());
  // NOTE: No delete is being done here as the sensors cleanup handler will
  // handle the actual delete.
});

void RemoteSensorManager::init() {
#if defined(SCAN_REMOTE_SENSORS_ON_STARTUP) && SCAN_REMOTE_SENSORS_ON_STARTUP
  int8_t networksFound;

  log_i("Scanning for RemoteSensors");
  WiFi.scanNetworks(true);
  while((networksFound = WiFi.scanComplete()) < 0) {
    delay(100);
    log_i(".");
  }
  const uint8_t REMOTE_SENSOR_PREFIX_LEN = String(REMOTE_SENSORS_PREFIX).length();
  for (int8_t i = 0; i < networksFound; i++) {
    if(WiFi.SSID(i).startsWith(REMOTE_SENSORS_PREFIX)) {
      const uint16_t sensorID = String(WiFi.SSID(i)).substring(REMOTE_SENSOR_PREFIX_LEN).toInt();
      log_i("Found RemoteSensor(%d): %s", sensorID, WiFi.SSID(i).c_str());
      createOrUpdate(sensorID);
    }
  }
#else
  log_i("Scanning for RemoteSensors DISABLED, remote sensors will only be created after reporting state");
#endif
}

void RemoteSensorManager::createOrUpdate(const uint16_t id, const uint16_t value) {
  // check for duplicate ID
  for (const auto& sensor : remoteSensors) {
    if(sensor->getRawID() == id) {
      sensor->setSensorValue(value);
      return;
    }
  }
  RemoteSensor *newSensor = new RemoteSensor(id, value);
  remoteSensors.add(newSensor);
  sensors.add(newSensor);
}

bool RemoteSensorManager::remove(const uint16_t id) {
  RemoteSensor *sensorToRemove = NULL;
  // check for duplicate ID or PIN
  for (const auto& sensor : remoteSensors) {
    if(sensor->getRawID() == id) {
      sensorToRemove = sensor;
    }
  }
  if(sensorToRemove != NULL) {
    remoteSensors.remove(sensorToRemove);
    sensors.remove(sensorToRemove);
    return true;
  }
  return false;
}

void RemoteSensorManager::getState(JsonArray &array) {
  for (const auto& sensor : remoteSensors) {
    JsonObject &sensorJson = array.createNestedObject();
    sensorJson[F("id")] = sensor->getRawID();
    sensorJson[F("value")] = sensor->getSensorValue();
    sensorJson[F("active")] = sensor->isActive();
    sensorJson[F("lastUpdate")] = sensor->getLastUpdate();
  }
}

void RemoteSensorManager::show() {
  if(remoteSensors.isEmpty()) {
    wifiInterface.printf(F("<X>"));
  } else {
    for (const auto& sensor : remoteSensors) {
      sensor->showSensor();
    }
  }
}

RemoteSensor::RemoteSensor(uint16_t id, uint16_t value) :
  Sensor(id + REMOTE_SENSORS_FIRST_SENSOR, -1, false, false), _rawID(id) {
  setSensorValue(value);
  log_i("RemoteSensor(%d) created with Sensor(%d), active: %s, value: %d",
    _rawID, getID(), isActive() ? "true" : "false", value);
}

void RemoteSensor::check() {
  if(isActive() && millis() > _lastUpdate + REMOTE_SENSORS_DECAY) {
    setSensorValue(0);
  }
}

void RemoteSensor::showSensor() {
  wifiInterface.printf(F("<RS %d %d>"), getRawID(), _value);
}

void RemoteSensorsCommandAdapter::process(const std::vector<String> arguments) {
  if(arguments.empty()) {
    // list all sensors
    RemoteSensorManager::show();
  } else {
    uint16_t sensorID = arguments[0].toInt();
    if (arguments.size() == 1 && RemoteSensorManager::remove(sensorID)) {
      // delete remote sensor
      wifiInterface.printf(F("<O>"));
    } else if (arguments.size() == 2) {
      // create/update remote sensor
      RemoteSensorManager::createOrUpdate(sensorID, arguments[1].toInt());
      wifiInterface.printf(F("<O>"));
    } else {
      wifiInterface.printf(F("<X>"));
    }
  }
}

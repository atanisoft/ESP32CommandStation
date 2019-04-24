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

String JSON_NAME_NODE PROGMEM = "name";
String JSON_STATE_NODE PROGMEM = "state";
String JSON_USAGE_NODE PROGMEM = "usage";

String JSON_COUNT_NODE PROGMEM = "count";

String JSON_ADDRESS_NODE PROGMEM = "address";

String JSON_SUB_ADDRESS_NODE PROGMEM = "subAddress";
String JSON_BOARD_ADDRESS_NODE = "boardAddress";
String JSON_SPEED_NODE PROGMEM = "speed";
String JSON_DIRECTION_NODE PROGMEM = "dir";
String JSON_ORIENTATION_NODE PROGMEM = "orientation";
String JSON_DESCRIPTION_NODE PROGMEM = "description";
String JSON_TYPE_NODE PROGMEM = "type";
String JSON_IDLE_NODE PROGMEM = "idle";

String JSON_IDLE_ON_STARTUP_NODE PROGMEM = "idleOnStartup";
String JSON_DEFAULT_ON_THROTTLE_NODE PROGMEM = "defaultOnThrottles";

String JSON_FUNCTIONS_NODE PROGMEM = "functions";
String JSON_LOCOS_NODE PROGMEM = "locos";
String JSON_LOCO_NODE PROGMEM = "loco";

String JSON_CONSIST_NODE PROGMEM = "consist";
String JSON_CONSISTS_NODE PROGMEM = "consists";
String JSON_DECODER_ASSISTED_NODE PROGMEM = "decoderAssisted";

String JSON_OUTPUTS_NODE PROGMEM = "outputs";
String JSON_ID_NODE PROGMEM = "id";
String JSON_PIN_NODE PROGMEM = "pin";
String JSON_FLAGS_NODE PROGMEM = "flags";
String JSON_INVERTED_NODE PROGMEM = "inverted";
String JSON_FORCE_STATE_NODE PROGMEM = "forceState";
String JSON_DEFAULT_STATE_NODE PROGMEM = "defaultState";

String JSON_SENSORS_NODE PROGMEM = "sensors";
String JSON_PULLUP_NODE PROGMEM = "pullUp";

String JSON_TURNOUTS_NODE PROGMEM = "turnouts";
String JSON_TURNOUTS_READABLE_STRINGS_NODE PROGMEM = "readableStrings";

String JSON_S88_NODE PROGMEM = "s88";
String JSON_S88_SENSOR_BASE_NODE PROGMEM = "sensorIDBase";

String JSON_PROG_ON_MAIN PROGMEM = "pom";
String JSON_CV_NODE PROGMEM = "cv";
String JSON_VALUE_NODE PROGMEM = "value";
String JSON_CV_BIT_NODE PROGMEM = "bit";
String JSON_IDENTIFY_NODE PROGMEM = "identify";
String JSON_ADDRESS_MODE_NODE PROGMEM = "addressMode";
String JSON_SPEED_TABLE_NODE PROGMEM = "speedTable";
String JSON_DECODER_VERSION_NODE PROGMEM = "version";
String JSON_DECODER_MANUFACTURER_NODE PROGMEM = "manufacturer";
String JSON_CREATE_NODE PROGMEM = "create";
String JSON_OVERALL_STATE_NODE PROGMEM = "overallState";
String JSON_LAST_UPDATE_NODE PROGMEM = "lastUpdate";

String JSON_VALUE_FORWARD PROGMEM = "FWD";
String JSON_VALUE_REVERSE PROGMEM = "REV";
String JSON_VALUE_TRUE PROGMEM = "true";
String JSON_VALUE_FALSE PROGMEM = "false";
String JSON_VALUE_NORMAL PROGMEM = "Normal";
String JSON_VALUE_OFF PROGMEM = "Off";
String JSON_VALUE_ON PROGMEM = "On";
String JSON_VALUE_FAULT PROGMEM = "Fault";
String JSON_VALUE_THROWN PROGMEM = "Thrown";
String JSON_VALUE_CLOSED PROGMEM = "Closed";
String JSON_VALUE_LONG_ADDRESS PROGMEM = "Long Address";
String JSON_VALUE_SHORT_ADDRESS PROGMEM = "Short Address";
String JSON_VALUE_MOBILE_DECODER PROGMEM = "Mobile Decoder";
String JSON_VALUE_STATIONARY_DECODER PROGMEM = "Stationary Decoder";

String ROSTER_JSON_FILE PROGMEM = "roster.json";
String CONSISTS_JSON_FILE PROGMEM = "consists.json";
String OUTPUTS_JSON_FILE PROGMEM = "outputs.json";
String S88_SENSORS_JSON_FILE PROGMEM = "s88.json";
String SENSORS_JSON_FILE PROGMEM = "sensors.json";
String TURNOUTS_JSON_FILE PROGMEM = "turnouts.json";
ConfigurationManager configStore;

StaticJsonBuffer<20480> jsonConfigBuffer;

ConfigurationManager::ConfigurationManager() {
}

ConfigurationManager::~ConfigurationManager() {
  SPIFFS.end();
}

void ConfigurationManager::init() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Loading Config"));
  if(!SPIFFS.begin()) {
    LOG(INFO, "[Config] SPIFFS mount failed, formatting SPIFFS and retrying");
    SPIFFS.begin(true);
  }
  SPIFFS.mkdir("/DCCppESP32");
}

void ConfigurationManager::clear() {
  SPIFFS.rmdir("/DCCppESP32");
  SPIFFS.mkdir("/DCCppESP32");
}

JsonObject &ConfigurationManager::load(const String &name) {
  LOG(INFO, "[Config] Loading /DCCppESP32/%s", name.c_str());
  File configFile = SPIFFS.open("/DCCppESP32/" + name, FILE_READ);
  jsonConfigBuffer.clear();
  JsonObject &root = jsonConfigBuffer.parseObject(configFile);
  configFile.close();
  return root;
}

void ConfigurationManager::store(const String &name, const JsonObject &json) {
  LOG(INFO, "[Config] Storing /DCCppESP32/%s", name.c_str());
  File configFile = SPIFFS.open("/DCCppESP32/" + name, FILE_WRITE);
  if(!configFile) {
    LOG_ERROR("[Config] Failed to open /DCCppESP32/%s", name.c_str());
    return;
  }
  json.printTo(configFile);
  configFile.close();
}

JsonObject &ConfigurationManager::createRootNode(bool clearBuffer) {
  if(clearBuffer) {
    jsonConfigBuffer.clear();
  }
  return jsonConfigBuffer.createObject();
}
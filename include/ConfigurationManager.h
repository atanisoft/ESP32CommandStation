/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2018-2019 Mike Dunston

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

#include <WString.h>
#include <FS.h>
#include <ArduinoJson.h>

// Class definition for the Configuration Management system in DCC++ESP32
class ConfigurationManager {
public:
  ConfigurationManager();
  virtual ~ConfigurationManager();
  void init();
  void clear();

  JsonObject &load(const String &);
  void store(const String &, const JsonObject &);
  JsonObject &createRootNode(bool=true);
};

extern ConfigurationManager configStore;

extern String JSON_NAME_NODE;
extern String JSON_STATE_NODE;
extern String JSON_USAGE_NODE;
extern String JSON_COUNT_NODE;
extern String JSON_ADDRESS_NODE;
extern String JSON_SUB_ADDRESS_NODE;
extern String JSON_BOARD_ADDRESS_NODE;
extern String JSON_SPEED_NODE;
extern String JSON_DIRECTION_NODE;
extern String JSON_ORIENTATION_NODE;
extern String JSON_DESCRIPTION_NODE;
extern String JSON_TYPE_NODE;
extern String JSON_IDLE_NODE;
extern String JSON_IDLE_ON_STARTUP_NODE;
extern String JSON_DEFAULT_ON_THROTTLE_NODE;
extern String JSON_FUNCTIONS_NODE;
extern String JSON_LOCOS_NODE;
extern String JSON_LOCO_NODE;
extern String JSON_CONSIST_NODE;
extern String JSON_CONSISTS_NODE;
extern String JSON_DECODER_ASSISTED_NODE;
extern String JSON_OUTPUTS_NODE;
extern String JSON_ID_NODE;
extern String JSON_PIN_NODE;
extern String JSON_FLAGS_NODE;
extern String JSON_INVERTED_NODE;
extern String JSON_FORCE_STATE_NODE;
extern String JSON_DEFAULT_STATE_NODE;
extern String JSON_SENSORS_NODE;
extern String JSON_PULLUP_NODE;
extern String JSON_TURNOUTS_NODE;
extern String JSON_TURNOUTS_READABLE_STRINGS_NODE;
extern String JSON_S88_NODE;
extern String JSON_S88_SENSOR_BASE_NODE;
extern String JSON_PROG_ON_MAIN;
extern String JSON_CV_NODE;
extern String JSON_VALUE_NODE;
extern String JSON_CV_BIT_NODE;
extern String JSON_IDENTIFY_NODE;
extern String JSON_ADDRESS_MODE_NODE;
extern String JSON_SPEED_TABLE_NODE;
extern String JSON_DECODER_VERSION_NODE;
extern String JSON_DECODER_MANUFACTURER_NODE;
extern String JSON_CREATE_NODE;
extern String JSON_OVERALL_STATE_NODE;
extern String JSON_LAST_UPDATE_NODE;

extern String JSON_VALUE_FORWARD;
extern String JSON_VALUE_REVERSE;
extern String JSON_VALUE_TRUE;
extern String JSON_VALUE_FALSE;
extern String JSON_VALUE_NORMAL;
extern String JSON_VALUE_OFF;
extern String JSON_VALUE_ON;
extern String JSON_VALUE_FAULT;
extern String JSON_VALUE_THROWN;
extern String JSON_VALUE_CLOSED;
extern String JSON_VALUE_LONG_ADDRESS;
extern String JSON_VALUE_SHORT_ADDRESS;
extern String JSON_VALUE_MOBILE_DECODER;
extern String JSON_VALUE_STATIONARY_DECODER;

extern String ROSTER_JSON_FILE;
extern String CONSISTS_JSON_FILE;
extern String OUTPUTS_JSON_FILE;
extern String SENSORS_JSON_FILE;
extern String S88_SENSORS_JSON_FILE;
extern String TURNOUTS_JSON_FILE;

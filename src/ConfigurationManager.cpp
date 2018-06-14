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
#include <Preferences.h>

ConfigurationManager configStore;

ConfigurationManager::ConfigurationManager() :
#if defined(SD_CARD_ENABLED) && SD_CARD_ENABLED
  _fs(SD), _usingSpiffs(false)
#else
  _fs(SPIFFS), _usingSpiffs(true)
#endif
  {
}

ConfigurationManager::~ConfigurationManager() {
#if defined(SD_CARD_ENABLED) && SD_CARD_ENABLED
  SD.end();
#endif
  if(_usingSpiffs) {
    SPIFFS.end();
  }
}

void ConfigurationManager::init() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Loading Config"));
#if defined(SD_CARD_ENABLED) && SD_CARD_ENABLED
  SPI.begin(SD_CARD_CLK_PIN, SD_CARD_MISO_PIN, SD_CARD_MOSI_PIN, SD_CARD_CS_PIN);
  SD.begin(SD_CARD_CS_PIN);
  if(SD.cardType() == CARD_NONE) {
    log_w("SD Card not found, switching to SPIFFS storage");
    _fs = SPIFFS;
    _usingSpiffs = true;
  }
#else
  if(!SPIFFS.begin()) {
    log_i("SPIFFS mount failed, formatting SPIFFS and retrying");
    SPIFFS.begin(true);
  }
#endif
  _fs.mkdir("/DCCppESP32");
}

void ConfigurationManager::clear() {
}

JsonObject &ConfigurationManager::load(const String &name, DynamicJsonBuffer &jsonBuffer) {
  File configFile = _fs.open("/DCCppESP32/" + name, FILE_READ);
  JsonObject &root = jsonBuffer.parseObject(configFile);
  configFile.close();
  return root;
}

void ConfigurationManager::store(const String &name, const JsonObject &json) {
  File configFile = _fs.open("/DCCppESP32/" + name, FILE_WRITE);
  if(!configFile) {
    log_e("Failed to open /DCCppESP32/%s", name.c_str());
    return;
  }
  json.printTo(configFile);
  configFile.close();
}

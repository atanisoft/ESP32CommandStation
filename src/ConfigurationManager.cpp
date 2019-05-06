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


ConfigurationManager configStore;

StaticJsonBuffer<20480> jsonConfigBuffer;

#if !defined(CONFIG_USE_SPIFFS) && !defined(COFNIG_USE_SD)
#define CONFIG_USE_SPIFFS true
#endif

#if CONFIG_USE_SPIFFS
#define CONFIG_FS SPIFFS
#elif COFNIG_USE_SD
#define CONFIG_FS SD_MMC
#endif

static constexpr const char *DCCPPESP32_CONFIG_DIR = "/DCCppESP32";

ConfigurationManager::ConfigurationManager() {
}

ConfigurationManager::~ConfigurationManager() {
  CONFIG_FS.end();
}

void ConfigurationManager::init() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Loading Config"));
#if CONFIG_USE_SPIFFS
  if(!SPIFFS.begin()) {
    LOG(INFO, "[Config] SPIFFS mount failed, formatting SPIFFS and retrying");
    if(!SPIFFS.begin(true)) {
      LOG(INFO, "[Config] SPIFFS mount failed even after formatting!");
      delay(10000);
      LOG(FATAL, "[Config] Aborting due to SPIFFS mount failure.");
    }
  }
#elif COFNIG_USE_SD
  if(!SD_MMC.begin()) {
    LOG(INFO, "[Config] SD_MMC mount failed, is there an SD card inserted?");
    delay(10000);
    LOG(FATAL, "[Config] Aborting due to SD_MMC mount failure.");
  }
#endif
  CONFIG_FS.mkdir(DCCPPESP32_CONFIG_DIR);
}

void ConfigurationManager::clear() {
  CONFIG_FS.rmdir(DCCPPESP32_CONFIG_DIR);
  CONFIG_FS.mkdir(DCCPPESP32_CONFIG_DIR);
}

JsonObject &ConfigurationManager::load(const char *name) {
  std::string configFilePath = StringPrintf("%s/%s", DCCPPESP32_CONFIG_DIR, name);
  LOG(INFO, "[Config] Loading %s", configFilePath.c_str());
  File configFile = CONFIG_FS.open(configFilePath.c_str(), FILE_READ);
  jsonConfigBuffer.clear();
  JsonObject &root = jsonConfigBuffer.parseObject(configFile);
  configFile.close();
  return root;
}

void ConfigurationManager::store(const char *name, const JsonObject &json) {
  std::string configFilePath = StringPrintf("%s/%s", DCCPPESP32_CONFIG_DIR, name);
  LOG(INFO, "[Config] Storing %s", configFilePath.c_str());
  File configFile = CONFIG_FS.open(configFilePath.c_str(), FILE_WRITE);
  if(!configFile) {
    LOG_ERROR("[Config] Failed to open %s", configFilePath.c_str());
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
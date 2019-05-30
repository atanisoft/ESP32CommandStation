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

LinkedList<RosterEntry *> LocomotiveManager::_roster([](RosterEntry *entry) {delete entry;});
LinkedList<Locomotive *> LocomotiveManager::_locos([](Locomotive *loco) {delete loco; });
LinkedList<LocomotiveConsist *> LocomotiveManager::_consists([](LocomotiveConsist *consist) {delete consist; });

static constexpr const char * OLD_ROSTER_JSON_FILE = "roster.json";
static constexpr const char * ROSTER_JSON_FILE = "locoroster.json";
static constexpr const char * ROSTER_ENTRY_JSON_FILE = "roster-%d.json";

static constexpr const char * OLD_CONSISTS_JSON_FILE = "consists.json";
static constexpr const char * CONSISTS_JSON_FILE = "lococonsists.json";
static constexpr const char * CONSIST_ENTRY_JSON_FILE = "consist-%d.json";

void LocomotiveManager::processThrottle(const std::vector<String> arguments) {
  int registerNumber = arguments[0].toInt();
  uint16_t locoAddress = arguments[1].toInt();
  if(isConsistAddress(locoAddress) || isAddressInConsist(locoAddress)) {
    processConsistThrottle(arguments);
    return;
  }
  Locomotive *instance = getLocomotiveByRegister(registerNumber);
  if(instance == nullptr) {
    instance = new Locomotive(registerNumber);
    _locos.add(instance);
  }
  instance->setLocoAddress(locoAddress);
  instance->setSpeed(arguments[2].toInt());
  instance->setDirection(arguments[3].toInt() == 1);
  instance->sendLocoUpdate();
  instance->showStatus();
}

void LocomotiveManager::processThrottleEx(const std::vector<String> arguments) {
  uint16_t locoAddress = arguments[0].toInt();
  int8_t speed = arguments[1].toInt();
  int8_t dir = arguments[2].toInt();
  auto instance = getLocomotive(locoAddress);
  if(speed >= 0) {
    instance->setSpeed(speed);
  }
  if(dir >= 0) {
    instance->setDirection(dir == 1);
  }
  instance->sendLocoUpdate();
  instance->showStatus();
}

// This method decodes the incoming function packet(s) to update the stored
// functinon states. Loco update will be sent afterwards.
void LocomotiveManager::processFunction(const std::vector<String> arguments) {
  int locoAddress = arguments[0].toInt();
  int functionByte = arguments[1].toInt();
  if(isConsistAddress(locoAddress)) {
    return;
  }
  auto loco = getLocomotive(locoAddress);
  // check this is a request for functions F13-F28
  if(arguments.size() > 2) {
    int secondaryFunctionByte = arguments[2].toInt();
    if((functionByte & 0xDE) == 0xDE) {
      for(uint8_t funcID = 13; funcID <= 20; funcID++) {
        loco->setFunction(funcID, bitRead(secondaryFunctionByte, funcID-13));
      }
    } else {
      for(uint8_t funcID = 21; funcID <= 28; funcID++) {
        loco->setFunction(funcID, bitRead(secondaryFunctionByte, funcID-21));
      }
    }
  } else {
    // this is a request for functions FL,F1-F12
    // for safety this guarantees that first nibble of function byte will always
    // be of binary form 10XX which should always be the case for FL,F1-F12
    if((functionByte & 0xB0) == 0xB0) {
      for(uint8_t funcID = 5; funcID <= 8; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-5));
      }
    } else if((functionByte & 0xA0) == 0xA0) {
      for(uint8_t funcID = 9; funcID <= 12; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-9));
      }
    } else {
      loco->setFunction(0, bitRead(functionByte, 4));
      for(uint8_t funcID = 1; funcID <= 4; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-1));
      }
    }
  }
  loco->sendLocoUpdate();
}

void LocomotiveManager::processFunctionEx(const std::vector<String> arguments) {
  int locoAddress = arguments[0].toInt();
  int function = arguments[1].toInt();
  int state = arguments[2].toInt();
  if(isConsistAddress(locoAddress)) {
    return;
  }
  auto loco = getLocomotive(locoAddress);
  loco->setFunction(function, state);
  loco->sendLocoUpdate();
}

void LocomotiveManager::processConsistThrottle(const std::vector<String> arguments) {
  uint16_t locoAddress = arguments[1].toInt();
  int8_t speed = arguments[2].toInt();
  bool forward = arguments[3].toInt() == 1;
  for (const auto& consist : _consists) {
    if (consist->getLocoAddress() == locoAddress || consist->isAddressInConsist(locoAddress)) {
      consist->updateThrottle(locoAddress, speed, forward);
      return;
    }
  }
}

void LocomotiveManager::showStatus() {
  for (const auto& loco : _locos) {
		loco->showStatus();
	}
  showConsistStatus();
}

void LocomotiveManager::showConsistStatus() {
  for (const auto& consist : _consists) {
		consist->showStatus();
	}
}

void LocomotiveManager::update() {
  for (const auto& loco : _locos) {
    loco->sendLocoUpdate();
  }
  for (const auto& loco : _consists) {
    loco->sendLocoUpdate();
  }
}

void LocomotiveManager::emergencyStop() {
  for (const auto& loco : _locos) {
    loco->setSpeed(-1);
	}
  sendDCCEmergencyStop();
}

Locomotive *LocomotiveManager::getLocomotive(const uint16_t locoAddress, const bool managed) {
  Locomotive *instance = nullptr;
  if(locoAddress) {
    for (const auto& loco : _locos) {
      if(loco->getLocoAddress() == locoAddress) {
        instance = loco;
      }
    }
    if(instance == nullptr) {
      instance = new Locomotive(_locos.length() + 1);
      instance->setLocoAddress(locoAddress);
      if(managed) {
        _locos.add(instance);
      }
    }
  }
  return instance;
}

Locomotive *LocomotiveManager::getLocomotiveByRegister(const uint8_t registerNumber) {
  for (const auto& loco : _locos) {
    if(loco->getRegister() == registerNumber) {
      return loco;
    }
  }
  return nullptr;
}

void LocomotiveManager::removeLocomotive(const uint16_t locoAddress) {
  Locomotive *locoToRemove = nullptr;
  for (const auto& loco : _locos) {
    if(loco->getLocoAddress() == locoAddress) {
      locoToRemove = loco;
    }
  }
  if(locoToRemove != nullptr) {
    locoToRemove->setIdle();
    locoToRemove->sendLocoUpdate();
    _locos.remove(locoToRemove);
  }
}

bool LocomotiveManager::removeLocomotiveConsist(const uint16_t consistAddress) {
  LocomotiveConsist *consistToRemove = nullptr;
  for (const auto& consist : _consists) {
    if(consist->getLocoAddress() == consistAddress) {
      consistToRemove = consist;
    }
  }
  if (consistToRemove != nullptr) {
    consistToRemove->releaseLocomotives();
    _consists.remove(consistToRemove);
    return true;
  }
  return false;
}

void LocomotiveManager::init() {
  bool persistNeeded = false;
  if (configStore.exists(ROSTER_JSON_FILE)) {
    JsonObject &root = configStore.load(ROSTER_JSON_FILE);
    JsonVariant count = root[JSON_COUNT_NODE];
    uint16_t locoCount = count.success() ? count.as<int>() : 0;
    LOG(INFO, "[Roster] Found %d RosterEntries", locoCount);
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Locos"), locoCount);
    if (locoCount > 0) {
      JsonArray &rosterEntries = root.get<JsonArray>(JSON_LOCOS_NODE);
      for (auto entry : rosterEntries) {
        JsonObject &rosterEntry = entry.as<JsonObject &>();
        if (configStore.exists(rosterEntry.get<char *>(JSON_FILE_NODE))) {
          _roster.add(new RosterEntry(rosterEntry.get<char *>(JSON_FILE_NODE)));
        } else {
          LOG_ERROR("[Roster] Unable to locate RosterEntry %s!", rosterEntry.get<char *>(JSON_FILE_NODE));
        }
      }
    }
  }

  if (configStore.exists(OLD_ROSTER_JSON_FILE)) {
    JsonObject &root = configStore.load(OLD_ROSTER_JSON_FILE);
    JsonVariant count = root[JSON_COUNT_NODE];
    uint16_t locoCount = count.success() ? count.as<int>() : 0;
    LOG(INFO, "[Roster] Found %d RosterEntries", locoCount);
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Locos"), locoCount);
    if (locoCount > 0) {
      JsonArray &rosterEntries = root.get<JsonArray>(JSON_LOCOS_NODE);
      for (auto entry : rosterEntries) {
        _roster.add(new RosterEntry(entry.as<JsonObject &>()));
      }
    }
    configStore.remove(OLD_ROSTER_JSON_FILE);
    persistNeeded = true;
  }

  if (configStore.exists(CONSISTS_JSON_FILE)) {
    JsonObject &consistRoot = configStore.load(CONSISTS_JSON_FILE);
    JsonVariant count = consistRoot[JSON_COUNT_NODE];
    uint16_t consistCount = count.success() ? count.as<int>() : 0;
    LOG(INFO, "[Consist] Found %d Consists", consistCount);
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Consists"), consistCount);
    if (consistCount > 0) {
      JsonArray &consists = consistRoot.get<JsonArray>(JSON_CONSISTS_NODE);
      for(auto entry : consists) {
        JsonObject &consistEntry = entry.as<JsonObject &>();
        if (configStore.exists(consistEntry.get<char *>(JSON_FILE_NODE))) {
          _consists.add(new LocomotiveConsist(consistEntry.get<char *>(JSON_FILE_NODE)));
        } else {
          LOG_ERROR("[Roster] Unable to locate ConsistEntry %s!", consistEntry.get<char *>(JSON_FILE_NODE));
        }
      }
    }
  }

  if(configStore.exists(OLD_CONSISTS_JSON_FILE)) {
    JsonObject &consistRoot = configStore.load(OLD_CONSISTS_JSON_FILE);
    JsonVariant count = consistRoot[JSON_COUNT_NODE];
    uint16_t consistCount = count.success() ? count.as<int>() : 0;
    LOG(INFO, "[Consist] Found %d Consists", consistCount);
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Consists"), consistCount);
    if (consistCount > 0) {
      JsonArray &consists = consistRoot.get<JsonArray>(JSON_CONSISTS_NODE);
      for (auto entry : consists) {
        _consists.add(new LocomotiveConsist(entry.as<JsonObject &>()));
      }
    }
    configStore.remove(OLD_CONSISTS_JSON_FILE);
    persistNeeded = true;
  }
  if (persistNeeded) {
    store();
  }
}

void LocomotiveManager::clear() {
  _locos.free();
  _consists.free();
  _roster.free();
  store();
}

uint16_t LocomotiveManager::store() {
  StaticJsonBuffer<1024> buf;
  JsonObject &root = configStore.createRootNode();
  JsonArray &locoArray = root.createNestedArray(JSON_LOCOS_NODE);
  uint16_t locoStoredCount = 0;
  for (const auto& entry : _roster) {
    std::string filename = StringPrintf(ROSTER_ENTRY_JSON_FILE, entry->getAddress());
    locoArray.createNestedObject()[JSON_FILE_NODE] = filename.c_str();
    buf.clear();
    JsonObject &entryRoot = buf.createObject();
    entry->toJson(entryRoot);
    configStore.store(filename.c_str(), entryRoot);
    locoStoredCount++;
  }
  root[JSON_COUNT_NODE] = locoStoredCount;
  configStore.store(ROSTER_JSON_FILE, root);

  JsonObject &consistRoot = configStore.createRootNode();
  JsonArray &consistArray = consistRoot.createNestedArray(JSON_CONSISTS_NODE);
  uint16_t consistStoredCount = 0;
  for (const auto& consist : _consists) {
    std::string filename = StringPrintf(CONSIST_ENTRY_JSON_FILE, consist->getLocoAddress());
    consistArray.createNestedObject()[JSON_FILE_NODE] = filename.c_str();
    buf.clear();
    JsonObject &entryRoot = buf.createObject();
    consist->toJson(entryRoot);
    configStore.store(filename.c_str(), entryRoot);
    consistStoredCount++;
  }
  consistRoot[JSON_COUNT_NODE] = consistStoredCount;
  configStore.store(CONSISTS_JSON_FILE, consistRoot);
  return locoStoredCount + consistStoredCount;
}

std::vector<RosterEntry *> LocomotiveManager::getDefaultLocos(const int8_t maxCount) {
  std::vector<RosterEntry *> retval;
  for (const auto& entry : _roster) {
    if(entry->isDefaultOnThrottles()) {
      if(maxCount < 0 || (maxCount > 0 && retval.size() < maxCount)) {
        retval.push_back(entry);
      }
    }
  }
  return retval;
}

void LocomotiveManager::getDefaultLocos(JsonArray &array) {
  for (const auto& entry : _roster) {
    if(entry->isDefaultOnThrottles()) {
      entry->toJson(array.createNestedObject());
    }
  }
}

void LocomotiveManager::getActiveLocos(JsonArray &array) {
  for (const auto& loco : _locos) {
    loco->toJson(array.createNestedObject());
  }
  for (const auto& consist : _consists) {
    consist->toJson(array.createNestedObject());
  }
}

void LocomotiveManager::getRosterEntries(JsonArray &array) {
  for (const auto& entry : _roster) {
    entry->toJson(array.createNestedObject());
  }
}

bool LocomotiveManager::isConsistAddress(uint16_t address) {
  for (const auto& consist : _consists) {
    if(consist->getLocoAddress() == address) {
      return true;
    }
  }
  return false;
}

bool LocomotiveManager::isAddressInConsist(uint16_t address) {
  for (const auto& consist : _consists) {
    if(consist->isAddressInConsist(address)) {
      return true;
    }
  }
  return false;
}

LocomotiveConsist *LocomotiveManager::getConsistByID(uint8_t consistAddress) {
  for (const auto& consist : _consists) {
    if(consist->getLocoAddress() == consistAddress) {
      return consist;
    }
  }
  return nullptr;
}

LocomotiveConsist *LocomotiveManager::getConsistForLoco(uint16_t locomotiveAddress) {
  for (const auto& consist : _consists) {
    if(consist->isAddressInConsist(locomotiveAddress)) {
      return consist;
    }
  }
  return nullptr;
}

LocomotiveConsist *LocomotiveManager::createLocomotiveConsist(int8_t consistAddress) {
  if(consistAddress == 0) {
    LOG(INFO, "[Consist] Creating new Loco Consist, automatic address selection...");
    uint8_t newConsistAddress = 127;
    for (const auto& consist : _consists) {
      if(newConsistAddress > consist->getLocoAddress() - 1 && !isConsistAddress(consist->getLocoAddress() - 1)) {
        newConsistAddress = consist->getLocoAddress() - 1;
        LOG(INFO, "[Consist] Found free address for new Loco Consist: %d", newConsistAddress);
        break;
      }
    }
    if(newConsistAddress > 0) {
      LOG(INFO, "[Consist] Adding new Loco Consist %d", newConsistAddress);
      _consists.add(new LocomotiveConsist(newConsistAddress, true));
      return getConsistByID(newConsistAddress);
    } else {
      LOG(INFO, "[Consist] Unable to locate free address for new Loco Consist, giving up.");
    }
  } else {
    LOG(INFO, "[Consist] Adding new Loco Consist %d", consistAddress);
    _consists.add(new LocomotiveConsist(abs(consistAddress), consistAddress < 0));
    return getConsistByID(abs(consistAddress));
  }
  return nullptr;
}

RosterEntry *LocomotiveManager::getRosterEntry(uint16_t address, bool create) {
  RosterEntry *instance = nullptr;
  for (const auto& entry : _roster) {
    if(entry->getAddress() == address) {
      instance = entry;
    }
  }
  if(instance == nullptr && create) {
    LOG(VERBOSE, "[Roster] No roster entry for address %d, creating", address);
    instance = new RosterEntry(address);
    _roster.add(instance);
  }
  return instance;
}

void LocomotiveManager::removeRosterEntry(uint16_t address) {
  RosterEntry *entryToRemove = nullptr;
  for (const auto& entry : _roster) {
    if(entry->getAddress() == address) {
      entryToRemove = entry;
    }
  }
  if(entryToRemove != nullptr) {
    LOG(VERBOSE, "[Roster] Removing roster entry for address %d", address);
    _roster.remove(entryToRemove);
  } else {
    LOG(WARNING, "[Roster] Roster entry for address %d doesn't exist, ignoring delete request", address);
  }
}

RosterEntry::RosterEntry(const char *filename) {
  DynamicJsonBuffer buf;
  JsonObject &entry = configStore.load(filename, buf);
  _description = entry[JSON_DESCRIPTION_NODE].as<String>();
  _address = entry[JSON_ADDRESS_NODE];
  _type = entry[JSON_TYPE_NODE].as<String>();
  _idleOnStartup = entry[JSON_IDLE_ON_STARTUP_NODE] == JSON_VALUE_TRUE;
  _defaultOnThrottles = entry[JSON_DEFAULT_ON_THROTTLE_NODE] == JSON_VALUE_TRUE;
}

RosterEntry::RosterEntry(const JsonObject &json) {
  _description = json[JSON_DESCRIPTION_NODE].as<String>();
  _address = json[JSON_ADDRESS_NODE];
  _type = json[JSON_TYPE_NODE].as<String>();
  _idleOnStartup = json[JSON_IDLE_ON_STARTUP_NODE] == JSON_VALUE_TRUE;
  _defaultOnThrottles = json[JSON_DEFAULT_ON_THROTTLE_NODE] == JSON_VALUE_TRUE;
}

void RosterEntry::toJson(JsonObject &json) {
  json[JSON_DESCRIPTION_NODE] = _description;
  json[JSON_ADDRESS_NODE] = _address;
  json[JSON_TYPE_NODE] = _type;
  json[JSON_IDLE_ON_STARTUP_NODE] = _idleOnStartup ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
  json[JSON_DEFAULT_ON_THROTTLE_NODE] = _defaultOnThrottles ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
}

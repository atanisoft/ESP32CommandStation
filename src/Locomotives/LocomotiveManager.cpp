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

TaskHandle_t LocomotiveManager::_taskHandle;
xSemaphoreHandle LocomotiveManager::_lock;
static constexpr UBaseType_t LOCOMOTIVE_MANAGER_TASK_PRIORITY = 3;

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
    if(functionByte & 0xDE) {
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
    if(functionByte & 0xA0) {
      for(uint8_t funcID = 9; funcID <= 12; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-9));
      }
    } else if(functionByte & 0xB0) {
      for(uint8_t funcID = 5; funcID <= 8; funcID++) {
        loco->setFunction(funcID, bitRead(functionByte, funcID-5));
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

void LocomotiveManager::updateTask(void *param) {
  while(true) {
    if(dccSignal[DCC_SIGNAL_OPERATIONS]->isEnabled()) {
      MUTEX_LOCK(_lock);
      for (const auto& loco : _locos) {
        // if it has been more than 50ms we should send a loco update packet
        if(millis() > loco->getLastUpdate() + 50) {
          loco->sendLocoUpdate();
        }
      }
      for (const auto& loco : _consists) {
        // if it has been more than 50ms we should send a loco update packet
        if(millis() > loco->getLastUpdate() + 50) {
          loco->sendLocoUpdate();
        }
      }
      MUTEX_UNLOCK(_lock);
    }
    vTaskDelay(pdMS_TO_TICKS(25));
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
      instance = new Locomotive(_locos.length());
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
  MUTEX_LOCK(_lock);
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
  MUTEX_UNLOCK(_lock);
}

bool LocomotiveManager::removeLocomotiveConsist(const uint16_t consistAddress) {
  MUTEX_LOCK(_lock);
  LocomotiveConsist *consistToRemove = nullptr;
  for (const auto& consist : _consists) {
    if(consist->getLocoAddress() == consistAddress) {
      consistToRemove = consist;
    }
  }
  if (consistToRemove != nullptr) {
    consistToRemove->releaseLocomotives();
    _consists.remove(consistToRemove);
    MUTEX_UNLOCK(_lock);
    return true;
  }
  MUTEX_UNLOCK(_lock);
  return false;
}

void LocomotiveManager::init() {
  _lock = xSemaphoreCreateMutex();
  JsonObject &root = configStore.load(ROSTER_JSON_FILE);
  JsonVariant count = root[JSON_COUNT_NODE];
  uint16_t locoCount = count.success() ? count.as<int>() : 0;
  log_v("Found %d RosterEntries", locoCount);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Locos"), locoCount);
  if(locoCount > 0) {
    for(auto loco : root.get<JsonArray>(JSON_LOCOS_NODE)) {
      _roster.add(new RosterEntry(loco.as<JsonObject &>()));
    }
  }
  JsonObject &consistRoot = configStore.load(CONSISTS_JSON_FILE);
  count = consistRoot[JSON_COUNT_NODE];
  uint16_t consistCount = count.success() ? count.as<int>() : 0;
  log_v("Found %d Consists", consistCount);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Found %02d Consists"), consistCount);
  if(locoCount > 0) {
    for(auto consist : consistRoot.get<JsonArray>(JSON_CONSISTS_NODE)) {
      _consists.add(new LocomotiveConsist(consist.as<JsonObject &>()));
    }
  }
  xTaskCreate(updateTask, "LocomotiveManager", DEFAULT_THREAD_STACKSIZE, NULL, LOCOMOTIVE_MANAGER_TASK_PRIORITY, &_taskHandle);
}

void LocomotiveManager::clear() {
  MUTEX_LOCK(_lock);
  _locos.free();
  _consists.free();
  _roster.free();
  store();
  MUTEX_UNLOCK(_lock);
}

uint16_t LocomotiveManager::store() {
  JsonObject &root = configStore.createRootNode();
  JsonArray &locoArray = root.createNestedArray(JSON_LOCOS_NODE);
  uint16_t locoStoredCount = 0;
  for (const auto& entry : _roster) {
    entry->toJson(locoArray.createNestedObject());
    locoStoredCount++;
  }
  root[JSON_COUNT_NODE] = locoStoredCount;
  configStore.store(ROSTER_JSON_FILE, root);

  JsonObject &consistRoot = configStore.createRootNode();
  JsonArray &consistArray = consistRoot.createNestedArray(JSON_CONSISTS_NODE);
  uint16_t consistStoredCount = 0;
  for (const auto& consist : _consists) {
    consist->toJson(consistArray.createNestedObject(), false, false);
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
    log_i("Creating new Loco Consist, automatic address selection...");
    uint8_t newConsistAddress = 127;
    for (const auto& consist : _consists) {
      if(newConsistAddress > consist->getLocoAddress() - 1 && !isConsistAddress(consist->getLocoAddress() - 1)) {
        newConsistAddress = consist->getLocoAddress() - 1;
        log_i("Found free address for new Loco Consist: %d", newConsistAddress);
        break;
      }
    }
    if(newConsistAddress > 0) {
      log_i("Adding new Loco Consist %d", newConsistAddress);
      _consists.add(new LocomotiveConsist(newConsistAddress, true));
      return getConsistByID(newConsistAddress);
    } else {
      log_i("Unable to locate free address for new Loco Consist, giving up.");
    }
  } else {
    log_i("Adding new Loco Consist %d", consistAddress);
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
    log_v("No roster entry for address %d, creating", address);
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
    log_v("Removing roster entry for address %d", address);
    _roster.remove(entryToRemove);
  } else {
    log_w("Roster entry for address %d doesn't exist, ignoring delete request", address);
  }
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

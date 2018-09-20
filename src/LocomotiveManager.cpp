/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017,2018 Mike Dunston

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

LinkedList<Locomotive *> LocomotiveManager::_locos([](Locomotive *loco) {delete loco; });
LinkedList<LocomotiveConsist *> LocomotiveManager::_consists([](LocomotiveConsist *consist) {delete consist; });

void LocomotiveManager::processThrottle(const std::vector<String> arguments) {
  int registerNumber = arguments[0].toInt();
  uint16_t locoAddress = arguments[1].toInt();
  if(isConsistAddress(locoAddress) || isAddressInConsist(locoAddress)) {
    processConsistThrottle(arguments);
    return;
  }
  Locomotive *instance = nullptr;
  for (const auto& loco : _locos) {
    if(loco->getRegister() == registerNumber) {
      instance = loco;
    }
  }
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

void LocomotiveManager::update() {
  for (const auto& loco : _locos) {
    // if it has been more than 50ms we should send a loco update packet
    if(millis() > loco->getLastUpdate() + 50) {
      loco->sendLocoUpdate();
    }
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
  return instance;
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
  // TODO: load roster from storage and add any idle by default locos
  //getLocomotive(1234)->setDefaultOnThrottles(true);
  //getLocomotive(5678)->setDefaultOnThrottles(true);
  //getLocomotive(9012)->setDefaultOnThrottles(true);
}

uint16_t LocomotiveManager::store() {
  // TODO: persist roster to storage
  return 0;
}

std::vector<Locomotive *> LocomotiveManager::getDefaultLocos(const int8_t maxCount) {
  std::vector<Locomotive *> retval;
  for (const auto& loco : _locos) {
    if(loco->isDefaultOnThrottles()) {
      if(maxCount < 0 || (maxCount > 0 && retval.size() < maxCount)) {
        retval.push_back(loco);
      }
    }
  }
  return retval;
}

void LocomotiveManager::getDefaultLocos(JsonArray &array) {
  for (const auto& loco : _locos) {
    if(loco->isDefaultOnThrottles()) {
      loco->toJson(array.createNestedObject(), true);
    }
  }
}

void LocomotiveManager::getActiveLocos(JsonArray &array) {
  for (const auto& loco : _locos) {
    loco->toJson(array.createNestedObject());
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
    uint8_t newConsistAddress = 127;
    for (const auto& consist : _consists) {
      if(newConsistAddress > consist->getLocoAddress() - 1 && !isConsistAddress(consist->getLocoAddress() - 1)) {
        newConsistAddress = consist->getLocoAddress() - 1;
        break;
      }
    }
    if(newConsistAddress > 0) {
      _consists.add(new LocomotiveConsist(newConsistAddress, true));
      return getConsistByID(newConsistAddress);
    }
  } else {
    _consists.add(new LocomotiveConsist(abs(consistAddress), consistAddress < 0));
    return getConsistByID(abs(consistAddress));
  }
  return nullptr;
}

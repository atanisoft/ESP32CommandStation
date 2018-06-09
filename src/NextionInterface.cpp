/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2018 NormHal
COPYRIGHT (c) 2018 Mike Dunston

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
#include "NextionInterface.h"

#ifndef NEXTION_UART_NUM
#define NEXTION_UART_NUM 2
#endif
#ifndef NEXTION_UART_BAUD
#define NEXTION_UART_BAUD 115200
#endif
#ifndef NEXTION_RX_PIN
#define NEXTION_RX_PIN 14
#endif
#ifndef NEXTION_TX_PIN
#define NEXTION_TX_PIN 27
#endif

#if defined(NEXTION_ENABLED) && NEXTION_ENABLED

HardwareSerial nextionSerial(NEXTION_UART_NUM);
Nextion nextion(nextionSerial);
bool NextionInterface::_initializing;
uint64_t NextionInterface::_startupTransitionTimer;

const uint8_t SELECTED_IMAGE_OFF = 3;
const uint8_t SELECTED_IMAGE_ON = 4;

const uint8_t SPEED_INCREMENT = 5;

DCCPPNextionPage *nextionPages[MAX_PAGES] = {
  new NextionTitlePage(nextion),
  new NextionAddressPage(nextion),
  new NextionThrottlePage(nextion),
  nullptr,
  nullptr,
  nullptr
};

NextionAddressPage::NextionAddressPage(Nextion &nextion) :
  DCCPPNextionPage(nextion, ADDRESS_PAGE, "1"),
  _buttons {
    NextionButton(nextion, ADDRESS_PAGE, 6, "b0"),
    NextionButton(nextion, ADDRESS_PAGE, 7, "b1"),
    NextionButton(nextion, ADDRESS_PAGE, 8, "b2"),
    NextionButton(nextion, ADDRESS_PAGE, 9, "b3"),
    NextionButton(nextion, ADDRESS_PAGE, 10, "b4"),
    NextionButton(nextion, ADDRESS_PAGE, 11, "b5"),
    NextionButton(nextion, ADDRESS_PAGE, 12, "b6"),
    NextionButton(nextion, ADDRESS_PAGE, 13, "b7"),
    NextionButton(nextion, ADDRESS_PAGE, 14, "b8"),
    NextionButton(nextion, ADDRESS_PAGE, 15, "b9")
  },
  _doneButton(nextion, ADDRESS_PAGE, 1, "done"),
  _abortButton(nextion, ADDRESS_PAGE, 2, "abort"),
  _currentAddress(nextion, ADDRESS_PAGE, 4, "t0"),
  _newAddress(nextion, ADDRESS_PAGE, 3, "DCCnew"),
  _address(0),
  _returnPageID(0),
  _newAddressString("") {
  for(int index = 0; index < 10; index++) {
    _buttons[index].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE])->addNumber(static_cast<NextionButton *>(widget));
      }
    });
  }
  _doneButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      NextionAddressPage *addressPage = static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE]);
      switch(addressPage->getReturnPage()) {
        case THROTTLE_PAGE:
          NextionThrottlePage *throttle = static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE]);
          throttle->changeLocoAddress(addressPage->getNewAddress());
          nextionPages[THROTTLE_PAGE]->display();
        break;
      }
    }
  });
  _abortButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      NextionAddressPage *addressPage = static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE]);
      nextionPages[addressPage->getReturnPage()]->display();
    }
  });
}

void NextionAddressPage::setCurrentAddress(uint32_t address) {
  _address = address;
}

void NextionAddressPage::addNumber(const NextionButton *button) {
  const uint8_t buttonMap[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
  for(int index = 0; index < 10; index++) {
    if(button == &_buttons[index]) {
      _newAddressString += String(buttonMap[index]);
      _newAddress.setValue(_newAddressString.toInt());
    }
  }
}

void NextionAddressPage::displayPage() {
  _newAddressString = "";
  _currentAddress.setTextAsNumber(_address);
  _newAddress.setValue(0);
}

NextionThrottlePage::NextionThrottlePage(Nextion &nextion) : DCCPPNextionPage(nextion, THROTTLE_PAGE, "2"),
  _activeLoco(0),
  _activeFunctionGroup(0),
  _locoNumbers {0, 0, 0},
  _locomotives {nullptr, nullptr, nullptr},
  _locoButtons {
    NextionButton(nextion, THROTTLE_PAGE, 14, "l0"),
    NextionButton(nextion, THROTTLE_PAGE, 15, "l1"),
    NextionButton(nextion, THROTTLE_PAGE, 16, "l2")
  },
  _fgroupButtons {
    NextionButton(nextion, THROTTLE_PAGE, 27, "fa"),
    NextionButton(nextion, THROTTLE_PAGE, 28, "fb"),
    NextionButton(nextion, THROTTLE_PAGE, 29, "fc")
  },
  _fwdButton(nextion, THROTTLE_PAGE, 3, "forward"),
  _revButton(nextion, THROTTLE_PAGE, 2, "reverse"),
  _onButton(nextion, THROTTLE_PAGE, 25, "don"),
  _offButton(nextion, THROTTLE_PAGE, 26, "doff"),
  _stopButton(nextion, THROTTLE_PAGE, 33, "allStop"),
  _functionButtons {
    NextionButton(nextion, THROTTLE_PAGE, 4, "f1"),
    NextionButton(nextion, THROTTLE_PAGE, 5, "f2"),
    NextionButton(nextion, THROTTLE_PAGE, 6, "f3"),
    NextionButton(nextion, THROTTLE_PAGE, 7, "f4"),
    NextionButton(nextion, THROTTLE_PAGE, 9, "f5"),
    NextionButton(nextion, THROTTLE_PAGE, 10, "f6"),
    NextionButton(nextion, THROTTLE_PAGE, 11, "f7"),
    NextionButton(nextion, THROTTLE_PAGE, 12, "f8"),
    NextionButton(nextion, THROTTLE_PAGE, 8, "f0"),
    NextionButton(nextion, THROTTLE_PAGE, 13, "Clear")
  },
  _locoAddress(nextion, THROTTLE_PAGE, 30, "b0"),
  _routes(nextion, THROTTLE_PAGE, 34, "b2"),
  _accessories(nextion, THROTTLE_PAGE, 31, "b1"),
  _downButton(nextion, THROTTLE_PAGE, 18, "dn"),
  _upButton(nextion, THROTTLE_PAGE, 19, "up"),
  _speedSlider(nextion, THROTTLE_PAGE, 17, "throttle"),
  _speedNumber(nextion, THROTTLE_PAGE, 1, "throttleNum") {
  for(int index = 0; index < 3; index++) {
    _locoButtons[index].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->activateLoco(static_cast<NextionButton *>(widget));
      }
    });
  }
  for(int index = 0; index < 3; index++) {
    _fgroupButtons[index].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->activateFunctionGroup(static_cast<NextionButton *>(widget));
      }
    });
  }
  _fwdButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setLocoDirection(true);
    }
  });
  _revButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setLocoDirection(false);
    }
  });
  _stopButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      LocomotiveManager::emergencyStop();
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->refresh();
    }
  });
  _onButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setTrackPower(true);
    }
  });
  _offButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setTrackPower(false);
    }
  });
  for(int index = 0; index < 10; index++) {
    _functionButtons[index].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->toggleFunction(static_cast<NextionButton *>(widget));
      }
    });
  }
  _locoAddress.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      NextionAddressPage *addressPage = static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE]);
      NextionThrottlePage *throttlePage = static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE]);
      addressPage->setCurrentAddress(throttlePage->getCurrentLocoAddress());
      addressPage->setReturnPage(THROTTLE_PAGE);
      addressPage->display();
    }
  });
  _downButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->decreaseLocoSpeed();
  });
  _upButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->increaseLocoSpeed();
  });
  _speedSlider.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_POP) {
      NextionSlider *slider = static_cast<NextionSlider *>(widget);
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setLocoSpeed(slider->getValue());
    }
  });
}

void NextionThrottlePage::activateLoco(const NextionButton *button) {
  // turn off all buttons
  for(int index = 0; index < 3; index++) {
    _locoButtons[index].setNumberProperty("picc", SELECTED_IMAGE_OFF);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if(button == &_locoButtons[index]) {
      _locoButtons[index].setNumberProperty("picc", SELECTED_IMAGE_ON);
      _activeLoco = index;
    }
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::activateFunctionGroup(const NextionButton *button) {
  // turn off all buttons
  for(int index = 0; index < 3; index++) {
    _fgroupButtons[index].setNumberProperty("picc", SELECTED_IMAGE_OFF);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if (button == &_fgroupButtons[index]) {
      _fgroupButtons[index].setNumberProperty("picc", SELECTED_IMAGE_ON);
      _activeFunctionGroup = index;
    }
  }
  refreshFunctionButtons();
}

void NextionThrottlePage::setLocoDirection(bool direction) {
  if(_locomotives[_activeLoco] != nullptr) {
    _locomotives[_activeLoco]->setDirection(direction);
  }
  refreshLocomotiveDirection();
}

void NextionThrottlePage::setTrackPower(bool on) {
  if(on) {
    MotorBoardManager::powerOnAll();
  } else {
    MotorBoardManager::powerOffAll();
  }
  refreshPowerButtons();
}

void NextionThrottlePage::toggleFunction(const NextionButton *button) {
  if(_locomotives[_activeLoco] != nullptr) {
    for(uint8_t function = 0; function < 10; function++) {
      if(&_functionButtons[function] == button) {
        if(function == 8) { // Front Light
          _locomotives[_activeLoco]->setFunction(0, !_locomotives[_activeLoco]->isFunctionEnabled(0));
        } else if(function == 9) { // Clear
          for(uint8_t index = 0; index < 28; index++) {
            _locomotives[_activeLoco]->setFunction(index, false);
          }
        } else {
          _locomotives[_activeLoco]->setFunction(_activeFunctionGroup * 8 + function + 1,
            !_locomotives[_activeLoco]->isFunctionEnabled(_activeFunctionGroup * 8 + function + 1));
        }
      }
    }
    refreshFunctionButtons();
  }
}

void NextionThrottlePage::changeLocoAddress(uint32_t newAddress) {
  _locoNumbers[_activeLoco] = newAddress;
  _locomotives[_activeLoco] = LocomotiveManager::getLocomotive(newAddress);
  refreshLocomotiveDetails();
}

uint32_t NextionThrottlePage::getCurrentLocoAddress() {
  if(_locomotives[_activeLoco] != nullptr) {
    return _locoNumbers[_activeLoco];
  }
  return 0;
}

void NextionThrottlePage::decreaseLocoSpeed() {
  if(_locomotives[_activeLoco] != nullptr) {
    _locomotives[_activeLoco]->setSpeed(_locomotives[_activeLoco]->getSpeed() - SPEED_INCREMENT);
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::increaseLocoSpeed() {
  if(_locomotives[_activeLoco] != nullptr) {
    _locomotives[_activeLoco]->setSpeed(_locomotives[_activeLoco]->getSpeed() + SPEED_INCREMENT);
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::setLocoSpeed(uint8_t speed) {
  if(_locomotives[_activeLoco] != nullptr) {
    _locomotives[_activeLoco]->setSpeed(speed);
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::invalidateLocomotive(uint32_t address) {
  for(int index = 0; index < 3; index++) {
    if(_locoNumbers[index] == address) {
      _locoNumbers[index] = 0;
      _locomotives[index] = nullptr;
    }
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::init() {
  uint8_t index = 0;
  for(auto loco : LocomotiveManager::getDefaultLocos(3)) {
    _locomotives[index++] = loco;
  }
  activateLoco(&_locoButtons[0]);
  activateFunctionGroup(&_fgroupButtons[0]);
}

void NextionThrottlePage::displayPage() {
  _locoButtons[_activeLoco].setNumberProperty("picc", SELECTED_IMAGE_ON);
  _fgroupButtons[_activeFunctionGroup].setNumberProperty("picc", SELECTED_IMAGE_ON);
  refreshPowerButtons();
  refreshLocomotiveDetails();
}

void NextionThrottlePage::refreshLocomotiveDetails() {
  for(int index = 0; index < 3; index++) {
    if(_locomotives[index] != nullptr) {
      _locoButtons[index].setTextAsNumber(_locoNumbers[index]);
    } else {
      _locoButtons[index].setText("");
    }
  }
  if(_locomotives[_activeLoco] != nullptr) {
    _speedSlider.setValue(_locomotives[_activeLoco]->getSpeed());
    _speedNumber.setValue(_locomotives[_activeLoco]->getSpeed());
  } else {
    _speedSlider.setValue(0);
    _speedNumber.setValue(0);
  }
  refreshLocomotiveDirection();
  refreshFunctionButtons();
}

void NextionThrottlePage::refreshFunctionButtons() {
  const uint8_t functionButtonImagesOff[3] = {3, 5, 7};
  const uint8_t functionButtonImagesOn[3] = {4, 6, 8};
  const uint8_t functionOffsets[3] = {1, 9, 17};
  for(int index = 0; index < 8; index++) {
    if(_locomotives[_activeLoco] != nullptr && _locomotives[_activeLoco]->isFunctionEnabled(_activeFunctionGroup * 8 + index + 1)) {
      _functionButtons[index].setNumberProperty("picc", functionButtonImagesOn[_activeFunctionGroup]);
    } else {
      _functionButtons[index].setNumberProperty("picc", functionButtonImagesOff[_activeFunctionGroup]);
    }
  }
  if(_locomotives[_activeLoco] != nullptr && _locomotives[_activeLoco]->isFunctionEnabled(0)) {
    _functionButtons[8].setNumberProperty("picc", functionButtonImagesOn[_activeFunctionGroup]);
  } else {
    _functionButtons[8].setNumberProperty("picc", functionButtonImagesOff[_activeFunctionGroup]);
  }
}

void NextionThrottlePage::refreshLocomotiveDirection() {
  if(_locomotives[_activeLoco] != nullptr) {
    if(_locomotives[_activeLoco]->isDirectionForward()) {
      _fwdButton.setNumberProperty("picc", SELECTED_IMAGE_ON);
      _revButton.setNumberProperty("picc", SELECTED_IMAGE_OFF);
    } else {
      _fwdButton.setNumberProperty("picc", SELECTED_IMAGE_OFF);
      _revButton.setNumberProperty("picc", SELECTED_IMAGE_ON);
    }
  } else {
    _fwdButton.setNumberProperty("picc", SELECTED_IMAGE_ON);
    _revButton.setNumberProperty("picc", SELECTED_IMAGE_OFF);
  }
}

void NextionThrottlePage::refreshPowerButtons() {
  if(MotorBoardManager::isTrackPowerOn()) {
    _onButton.setNumberProperty("picc", SELECTED_IMAGE_ON);
    _offButton.setNumberProperty("picc", SELECTED_IMAGE_OFF);
  } else {
    _onButton.setNumberProperty("picc", SELECTED_IMAGE_OFF);
    _offButton.setNumberProperty("picc", SELECTED_IMAGE_ON);
  }
}

void NextionInterface::init() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init Nextion"));
  nextionSerial.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
  nextion.init();

  nextionPages[TITLE_PAGE]->display();
  _initializing = true;
  _startupTransitionTimer = millis() + 2500;
}

void NextionInterface::update() {
  if(_initializing && _startupTransitionTimer <= millis()) {
    _initializing = false;
    nextionPages[THROTTLE_PAGE]->display();
  }
  nextion.poll();
}

#endif

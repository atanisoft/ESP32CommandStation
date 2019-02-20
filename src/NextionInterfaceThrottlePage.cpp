/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2018-2019 NormHal
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

#include "DCCppESP32.h"

#if NEXTION_ENABLED

const uint8_t SPEED_INCREMENT = 5;

const uint8_t dec=4;            //Dec
const uint8_t throttlenum=5;    //ThrottleNum
const uint8_t inc=6;            //Inc
const uint8_t throttleslider=7; //Throttle
const uint8_t rev=8;            //Rev
const uint8_t smalllogo=9;      // ??
const uint8_t fwd=10;           //Fwd
const uint8_t loco1=11;         //Loco1 inactive images
const uint8_t loco2=12;         //Loco2
const uint8_t loco3=13;         //Loco3

const uint8_t locoaddr=14;      //LocoAddr
const uint8_t acc=15;           //Turnouts

const uint8_t f1=16;            //F1 PIC 4/
const uint8_t f2=17;            //F2 PIC 5/
const uint8_t f3=18;            //F3 PIC 6/
const uint8_t f4=19;            //F4 PIC 7/
const uint8_t f0=20;            //F0 PIC 12/
const uint8_t f5=21;            //F5 PIC 8/
const uint8_t f6=22;            //F6 PIC 9/
const uint8_t f7=23;            //F7 PIC 10/
const uint8_t f8=24;            //F8 PIC 11/
const uint8_t clearFunctions=25;//Clear PIC 13/

const uint8_t fgp1=26;          //F18
const uint8_t fgp2=27;          //F916
const uint8_t fgp3=28;          //F1724

const uint8_t FUNC_LIGHT_INDEX = 8;
const uint8_t FUNC_CLEAR_INDEX = 9;

const uint8_t F0_PIC_OFF=28;
const uint8_t F0_PIC_ON=53;
const uint8_t F1_PIC_OFF=4;
const uint8_t F1_PIC_ON=29;

const uint8_t LOCO_PIC_OFF=58;
const uint8_t LOCO_PIC_ON=59;

const uint8_t LOCOADDR_PIC_OFF=60;
const uint8_t LOCOADDR_PIC_ON=61;
const uint8_t ACC_PIC_OFF=62;
const uint8_t ACC_PIC_ON=63;

const uint8_t DOWN_PIC_OFF=64;
const uint8_t DOWN_PIC_ON=65;
const uint8_t UP_PIC_OFF=66;
const uint8_t UP_PIC_ON=67;

const uint8_t FWD_PIC_OFF=68;
const uint8_t FWD_PIC_ON=69;
const uint8_t REV_PIC_OFF=70;
const uint8_t REV_PIC_ON=71;

const uint8_t CLEAR_PIC_OFF=72;
const uint8_t CLEAR_PIC_ON=73;

const uint8_t FG1_PIC_OFF=77;
const uint8_t FG1_PIC_ON=74;
const uint8_t FG2_PIC_OFF=78;
const uint8_t FG2_PIC_ON=75;
const uint8_t FG3_PIC_OFF=79;
const uint8_t FG3_PIC_ON=76;

//
/************************************************************************************************************/
// Throttle Page
/************************************************************************************************************/
//
NextionThrottlePage::NextionThrottlePage(Nextion &nextion) : DCCPPNextionPage(nextion, THROTTLE_PAGE, "2"),
  _activeLoco(0),
  _activeFunctionGroup(0),
  _locoNumbers {0, 0, 0},
  _locoButtons {
    NextionButton(nextion, THROTTLE_PAGE, loco1, "Loco1"),
    NextionButton(nextion, THROTTLE_PAGE, loco2, "Loco2"),
    NextionButton(nextion, THROTTLE_PAGE, loco3, "Loco3")
  },
  _fgroupButtons {
    NextionButton(nextion, THROTTLE_PAGE, fgp1, "F18"),
    NextionButton(nextion, THROTTLE_PAGE, fgp2, "F916"),
    NextionButton(nextion, THROTTLE_PAGE, fgp3, "F1724")
  },
  _functionButtons {
    NextionButton(nextion, THROTTLE_PAGE, f1, "F1"),
    NextionButton(nextion, THROTTLE_PAGE, f2, "F2"),
    NextionButton(nextion, THROTTLE_PAGE, f3, "F3"),
    NextionButton(nextion, THROTTLE_PAGE, f4, "F4"),
    NextionButton(nextion, THROTTLE_PAGE, f5, "F5"),
    NextionButton(nextion, THROTTLE_PAGE, f6, "F6"),
    NextionButton(nextion, THROTTLE_PAGE, f7, "F7"),
    NextionButton(nextion, THROTTLE_PAGE, f8, "F8"),
    NextionButton(nextion, THROTTLE_PAGE, f0, "F0"),
    NextionButton(nextion, THROTTLE_PAGE, clearFunctions, "Clear")
  },
  _fwdButton(nextion, THROTTLE_PAGE, fwd, "Fwd"),
  _revButton(nextion, THROTTLE_PAGE, rev, "Rev"),
  _locoAddress(nextion, THROTTLE_PAGE, locoaddr, "LocoAddr"),
  _setup(nextion, THROTTLE_PAGE, 34, "b2"),
  _accessories(nextion, THROTTLE_PAGE, acc, "Acc"),
  _downButton(nextion, THROTTLE_PAGE, dec, "Dec"),
  _upButton(nextion, THROTTLE_PAGE, inc, "Inc"),
  _speedSlider(nextion, THROTTLE_PAGE, throttleslider, "Throttle"),
  _speedNumber(nextion, THROTTLE_PAGE, throttlenum, "ThrottleNum") {
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
      addressPage->setPreviousPage(THROTTLE_PAGE);
      addressPage->display();
    }
  });
  _accessories.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      NextionTurnoutPage *turnoutPage = static_cast<NextionTurnoutPage *>(nextionPages[TURNOUT_PAGE]);
      turnoutPage->setPreviousPage(THROTTLE_PAGE);
      turnoutPage->display();
    }
  });
  _downButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->decreaseLocoSpeed();
  });
  _upButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->increaseLocoSpeed();
  });
  _speedSlider.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      NextionSlider *slider = static_cast<NextionSlider *>(widget);
      static_cast<NextionThrottlePage*>(nextionPages[THROTTLE_PAGE])->setLocoSpeed(slider->getValue());
    }
  });
}

void NextionThrottlePage::activateLoco(const NextionButton *button) {
  // turn off all buttons
  for(int index = 0; index < 3; index++) {
    _locoButtons[index].setPictureID(LOCO_PIC_OFF);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if(button == &_locoButtons[index]) {
      _locoButtons[index].setPictureID(LOCO_PIC_ON);
      _activeLoco = index;
    }
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::activateFunctionGroup(const NextionButton *button) {
  // turn off all buttons
  uint8_t fgrp_buts[6] = {FG1_PIC_OFF, FG2_PIC_OFF, FG3_PIC_OFF, FG1_PIC_ON, FG2_PIC_ON, FG3_PIC_ON};
  for(int index = 0; index < 3; index++) {
    _fgroupButtons[index].setPictureID(fgrp_buts[index]);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if (button == &_fgroupButtons[index]) {
      _fgroupButtons[index].setPictureID(fgrp_buts[index + 3]);
      _activeFunctionGroup = index;
    }
  }
  refreshFunctionButtons();
}

void NextionThrottlePage::setLocoDirection(bool direction) {
  if(_locoNumbers[_activeLoco]) {
    LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setDirection(direction);
    if(direction) {
      _fwdButton.setPictureID(FWD_PIC_ON);
      _revButton.setPictureID(REV_PIC_OFF);
    } else {
      _fwdButton.setPictureID(FWD_PIC_OFF);
      _revButton.setPictureID(REV_PIC_ON);
    }
  }
}

void NextionThrottlePage::toggleFunction(const NextionButton *button) {
  if(_locoNumbers[_activeLoco]) {
    for(uint8_t function = 0; function < 10; function++) {
      uint16_t functionPicOff = _activeFunctionGroup * 8 + function + F1_PIC_OFF;
      uint16_t functionPicOn = _activeFunctionGroup * 8 + function + F1_PIC_ON;
      if(&_functionButtons[function] == button) {
        if(function == FUNC_LIGHT_INDEX) { // Front Light
          if(_functionButtons[FUNC_LIGHT_INDEX].getPictureID() == F0_PIC_OFF) {
            _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_ON);
            LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setFunction(0, true);
          } else {
            _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_OFF);
            LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setFunction(0, false);
          }
        } else if(function == FUNC_CLEAR_INDEX) { // Clear all 28 functions... 29?
          for(uint8_t index = 0; index < 28; index++) {
            LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setFunction(index, false);
          }
        } else {
          if(_functionButtons[function].getPictureID() == functionPicOff) {
            _functionButtons[function].setPictureID(functionPicOn);
            LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setFunction(_activeFunctionGroup * 8 + function + 1, true);
          } else {
            _functionButtons[function].setPictureID(functionPicOff);
            LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setFunction(_activeFunctionGroup * 8 + function + 1, false);
          }
        }
      }
    }
    refreshFunctionButtons();
  }
}

void NextionThrottlePage::changeLocoAddress(uint32_t newAddress) {
  _locoNumbers[_activeLoco] = newAddress;
  refreshLocomotiveDetails();
}

uint32_t NextionThrottlePage::getCurrentLocoAddress() {
  return _locoNumbers[_activeLoco];
}

void NextionThrottlePage::decreaseLocoSpeed() {
  if(_locoNumbers[_activeLoco]) {
    int8_t speed = _speedNumber.getTextAsNumber() - SPEED_INCREMENT;
    if(speed < 0) {
      speed = 0;
    }
    LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setSpeed(speed);
    _speedNumber.setTextAsNumber(speed);
    _speedSlider.setValue(speed);
  }
}

void NextionThrottlePage::increaseLocoSpeed() {
  if(_locoNumbers[_activeLoco]) {
    int8_t speed = _speedNumber.getTextAsNumber() + SPEED_INCREMENT;
    if(speed < 0) {
      speed = 0;
    }
    LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setSpeed(speed);
    _speedNumber.setTextAsNumber(speed);
    _speedSlider.setValue(speed);
  }
}

void NextionThrottlePage::setLocoSpeed(uint8_t speed) {
  if(_locoNumbers[_activeLoco]) {
    LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco])->setSpeed(speed);
    _speedNumber.setTextAsNumber(speed);
    _speedSlider.setValue(speed);
  }
}

void NextionThrottlePage::invalidateLocomotive(uint32_t address) {
  for(int index = 0; index < 3; index++) {
    if(_locoNumbers[index] == address) {
      _locoNumbers[index] = 0;
    }
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::init() {
  uint8_t locoCount = 0;
  for(auto loco : LocomotiveManager::getDefaultLocos(3)) {
    _locoNumbers[locoCount] = loco->getAddress();
    _locoButtons[locoCount++].setTextAsNumber(loco->getAddress());
  }
  if(locoCount) {
    activateLoco(&_locoButtons[0]);
    activateFunctionGroup(&_fgroupButtons[0]);
  }
}

void NextionThrottlePage::displayPage() {
  _locoButtons[_activeLoco].setPictureID(LOCO_PIC_ON);
  _fgroupButtons[_activeFunctionGroup].setPictureID(FG1_PIC_ON);
  refreshLocomotiveDetails();
}

void NextionThrottlePage::previousPageCallback(DCCPPNextionPage *previousPage) {
  changeLocoAddress(static_cast<NextionAddressPage *>(previousPage)->getNewAddress());
}

void NextionThrottlePage::refreshLocomotiveDetails()
{
  for(int index = 0; index < 3; index++) {
    if(_locoNumbers[index]) {
      _locoButtons[index].setTextAsNumber(_locoNumbers[index]);
    } else {
      _locoButtons[index].setText("");
    }
  }
  if(_locoNumbers[_activeLoco]) {
    auto loco = LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco]);
    _speedSlider.setValue(loco->getSpeed());
    _speedNumber.setTextAsNumber(loco->getSpeed());
    if(loco->isDirectionForward()) {
      _fwdButton.setPictureID(FWD_PIC_ON);
      _revButton.setPictureID(REV_PIC_OFF);
    } else {
      _fwdButton.setPictureID(FWD_PIC_OFF);
      _revButton.setPictureID(REV_PIC_ON);
    }
    refreshFunctionButtons();
  } else {
    _speedSlider.setValue(0);
    _speedNumber.setTextAsNumber(0);
    _fwdButton.setPictureID(FWD_PIC_ON);
    _revButton.setPictureID(REV_PIC_OFF);
    for(int index = 0; index < FUNC_LIGHT_INDEX; index++) {
      _functionButtons[index].setPictureID(_activeFunctionGroup * 8 + index + F1_PIC_OFF);
    }
    _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_OFF);
  }
}

void NextionThrottlePage::refreshFunctionButtons() {
  auto loco = LocomotiveManager::getLocomotive(_locoNumbers[_activeLoco]);
  if(loco) {
    for(int index = 0; index < FUNC_LIGHT_INDEX; index++) {
      uint8_t functionIndex = _activeFunctionGroup * 8 + index;
      if(loco->isFunctionEnabled(functionIndex + 1)) {
        _functionButtons[index].setPictureID(functionIndex + F1_PIC_ON);
      } else {
        _functionButtons[index].setPictureID(functionIndex + F1_PIC_OFF);
      }
    }
    if(loco->isFunctionEnabled(0)) {
      _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_ON);
    } else {
      _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_OFF);
    }
  } else {
    for(int index = 0; index < FUNC_LIGHT_INDEX; index++) {
      uint8_t functionIndex = _activeFunctionGroup * 8 + index;
      _functionButtons[index].setPictureID(functionIndex + F1_PIC_OFF);
    }
    _functionButtons[FUNC_LIGHT_INDEX].setPictureID(F0_PIC_OFF);
  }
}

#endif
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

// Nextion Objects
// Page 1 Address

#define on1 1 //On PIC 44/45
#define stop1 2 //Stop PIC 70/71
#define off1 3 //Off PIC 46/47

#define mainlogo 4

#define baseaddr 5
#define subaddr 6

#define oldaddr 7 //OldAddr
#define newaddr 8 //NewAddr

#define num1 9 //b1 PIC 72/82
#define num2 10 //b2 PIC 73/83
#define num3 11 //b3 PIC 74/84
#define num4 12 //b4 PIC 75/85
#define num5 13 //b5 PIC 76/86
#define num6 14 //b6 PIC 77/87
#define num7 15 //b7 PIC 78/88
#define num8 16 //b8 PIC 79/89
#define num9 17 //b9 PIC 80/90
#define num0 18 //b0 PIC 81/91

#define save1 19 //save PIC 92/93
#define erase 20
#define quit1 21 //quit PIC 94/95

//Page 2 - Throttle

#define dec 4 //Dec
#define throttlenum 5 //ThrottleNum
#define inc 6 //Inc
#define throttleslider 7 //Throttle
#define rev 8 //Rev
#define smalllogo 9
#define fwd 10 //Fwd
#define loco1 11 //Loco1 inactive images
#define loco2 12 //Loco2
#define loco3 13 //Loco3

#define locoaddr 14 //LocoAddr
#define acc 15 //Turnouts

#define f1 16 //F1 PIC 4/
#define f2 17 //F2 PIC 5/
#define f3 18 //F3 PIC 6/
#define f4 19 //F4 PIC 7/
#define f0 20 //F0 PIC 12/
#define f5 21 //F5 PIC 8/
#define f6 22 //F6 PIC 9/
#define f7 23 //F7 PIC 10/
#define f8 24 //F8 PIC 11/
#define clear 25 //Clear PIC 13/

#define fgp1 26 //F18
#define fgp2 27 //F916
#define fgp3 28 //F1724

// Nextion images
//
#define F1_PIC_OFF 4
#define F2_PIC_OFF 5
#define F3_PIC_OFF 6
#define F4_PIC_OFF 7
#define F5_PIC_OFF 8
#define F6_PIC_OFF 9
#define F7_PIC_OFF 10
#define F8_PIC_OFF 11

#define F9_PIC_OFF 12
#define F10_PIC_OFF 13
#define F11_PIC_OFF 14
#define F12_PIC_OFF 15
#define F13_PIC_OFF 16
#define F14_PIC_OFF 17
#define F15_PIC_OFF 18
#define F16_PIC_OFF 19

#define F17_PIC_OFF 20
#define F18_PIC_OFF 21
#define F19_PIC_OFF 22
#define F20_PIC_OFF 23
#define F21_PIC_OFF 24
#define F22_PIC_OFF 25
#define F23_PIC_OFF 26
#define F24_PIC_OFF 27

#define F0_PIC_OFF 28

#define F1_PIC_ON 29
#define F2_PIC_ON 30
#define F3_PIC_ON 31
#define F4_PIC_ON 32
#define F5_PIC_ON 33
#define F6_PIC_ON 34
#define F7_PIC_ON 35
#define F8_PIC_ON 36

#define F9_PIC_ON 37
#define F10_PIC_ON 38
#define F11_PIC_ON 39
#define F12_PIC_ON 40
#define F13_PIC_ON 41
#define F14_PIC_ON 42
#define F15_PIC_ON 43
#define F16_PIC_ON 44

#define F17_PIC_ON 45
#define F18_PIC_ON 46
#define F19_PIC_ON 47
#define F20_PIC_ON 48
#define F21_PIC_ON 49
#define F22_PIC_ON 50
#define F23_PIC_ON 51
#define F24_PIC_ON 52

#define F0_PIC_ON 53

#define ON_PIC_OFF 54
#define ON_PIC_ON 55
#define OFF_PIC_OFF 56
#define OFF_PIC_ON 57

#define LOCO_PIC_OFF 58
#define LOCO_PIC_ON 59

#define LOCOADDR_PIC_OFF 60
#define LOCOADDR_PIC_ON 61
#define ACC_PIC_OFF 62
#define ACC_PIC_ON 63

#define DOWN_PIC_OFF 64
#define DOWN_PIC_ON 65
#define UP_PIC_OFF 66
#define UP_PIC_ON 67

#define FWD_PIC_OFF 68
#define FWD_PIC_ON 69
#define REV_PIC_OFF 70
#define REV_PIC_ON 71

#define CLEAR_PIC_OFF 72
#define CLEAR_PIC_ON 73

#define FG1_PIC_OFF 77
#define FG1_PIC_ON 74
#define FG2_PIC_OFF 78
#define FG2_PIC_ON 75
#define FG3_PIC_OFF 79
#define FG3_PIC_ON 76

//Page 3

#define RH 108
#define LH 106

#define TO_RH_CLOSED 108
#define TO_RH_THROWN 109
#define TO_LH_CLOSED 106
#define TO_LH_THROWN 107

#define slot0 4
#define slot1 5
#define slot2 6
#define slot3 7
#define slot4 8

#define slot5 9
#define slot6 10
#define slot7 11
#define slot8 12
#define slot9 13

#define slot10 14
#define slot11 15
#define slot12 16
#define slot13 17
#define slot14 18

#define ad0 25
#define ad1 26
#define ad2 27
#define ad3 28
#define ad4 29

#define ad5 30
#define ad6 31
#define ad7 32
#define ad8 33
#define ad9 34

#define ad10 35
#define ad11 36
#define ad12 37
#define ad13 38
#define ad14 39

#define prev 19
#define addto 20
#define next 21
#define back 22
#define del 23
#define routes 24

HardwareSerial nextionSerial(NEXTION_UART_NUM);
// if using UART2 we need to turn off flushing as it corrupts the stream
#if NEXTION_UART_NUM == 2
Nextion nextion(nextionSerial, false);
#else
Nextion nextion(nextionSerial);
#endif
bool NextionInterface::_initializing;
uint64_t NextionInterface::_startupTransitionTimer;

const uint8_t SPEED_INCREMENT = 5;

DCCPPNextionPage *nextionPages[MAX_PAGES] = {
  new NextionTitlePage(nextion),
  new NextionAddressPage(nextion),
  new NextionThrottlePage(nextion),
  new NextionTurnoutPage(nextion),
  nullptr,
  nullptr
};

DCCPPNextionPage::DCCPPNextionPage(Nextion &nextion, const uint8_t pageID, const char *pageName) :
  NextionPage(nextion, pageID, 0, pageName),
  _onButton(nextion, pageID, on1, "On"),
  _offButton(nextion, pageID, off1, "Off"),
  _stopButton(nextion, pageID, stop1, "Stop"),
  _pageInitialized(false),
  _returnPageID(0) {
  _stopButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      LocomotiveManager::emergencyStop();
      nextionPages[widget->getPageID()]->refresh();
    }
  });
  _onButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->setTrackPower(true);
    }
  });
  _offButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->setTrackPower(false);
    }
  });
}

void DCCPPNextionPage::display() {
#if NEXTION_UART_NUM == 2
  // manual flush since we are on UART2
  while(nextionSerial.available()) {
    nextionSerial.read();
  }
#endif
  String cmd = String("page ") + String(m_name);
  m_nextion.sendCommand((char *)cmd.c_str());
  if(!m_nextion.checkCommandComplete()) {
    log_e("display of page %s was not successful.", m_name);
  } else {
    log_i("displayed page %s", m_name);
  }
  if(!_pageInitialized) {
    init();
    _pageInitialized = true;
  }
  refreshPowerButtons();
  displayPage();
}

void DCCPPNextionPage::refresh() {
  refreshPowerButtons();
  refreshPage();
}

void DCCPPNextionPage::setTrackPower(bool on) {
  if(on) {
    MotorBoardManager::powerOnAll();
  } else {
    MotorBoardManager::powerOffAll();
  }
  refreshPowerButtons();
}

void DCCPPNextionPage::displayPreviousPage(bool invokeCallback) {
  if(_returnPageID > 0) {
    nextionPages[_returnPageID]->display();
    if(invokeCallback) {
      nextionPages[_returnPageID]->previousPageCallback(this);
    }
  }
}

void DCCPPNextionPage::refreshPowerButtons() {
  if(MotorBoardManager::isTrackPowerOn()) {
    _onButton.setNumberProperty("pic", ON_PIC_ON);
    _offButton.setNumberProperty("pic", OFF_PIC_OFF);
  } else {
    _onButton.setNumberProperty("pic", ON_PIC_OFF);
    _offButton.setNumberProperty("pic", OFF_PIC_ON);
  }
}

//
/************************************************************************************************************/
// Address Page
/************************************************************************************************************/
//
NextionAddressPage::NextionAddressPage(Nextion &nextion) :
  DCCPPNextionPage(nextion, ADDRESS_PAGE, "1"),
  _buttons {
    NextionButton(nextion, ADDRESS_PAGE, num1, "b1"),
    NextionButton(nextion, ADDRESS_PAGE, num2, "b2"),
    NextionButton(nextion, ADDRESS_PAGE, num3, "b3"),
    NextionButton(nextion, ADDRESS_PAGE, num4, "b4"),
    NextionButton(nextion, ADDRESS_PAGE, num5, "b5"),
    NextionButton(nextion, ADDRESS_PAGE, num6, "b6"),
    NextionButton(nextion, ADDRESS_PAGE, num7, "b7"),
    NextionButton(nextion, ADDRESS_PAGE, num8, "b8"),
    NextionButton(nextion, ADDRESS_PAGE, num9, "b9"),
    NextionButton(nextion, ADDRESS_PAGE, num0, "b0")
  },
  _doneButton(nextion, ADDRESS_PAGE, save1, "Save"),
  _abortButton(nextion, ADDRESS_PAGE, quit1, "Quit"),
  _eraseButton(nextion, ADDRESS_PAGE, erase, "Erase"),
  _currentAddress(nextion, ADDRESS_PAGE, oldaddr, "OldAddr"),
  _newAddress(nextion, ADDRESS_PAGE, newaddr, "NewAddr"),
  _address(0),
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
      nextionPages[widget->getPageID()]->displayPreviousPage();
    }
  });
  _abortButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
  _eraseButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE])->removeNumber(static_cast<NextionButton *>(widget));
    }
  });
}

void NextionAddressPage::addNumber(const NextionButton *button) {
  const uint8_t buttonMap[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
  for(int index = 0; index < 10; index++) {
    if(button == &_buttons[index]) {
      _newAddressString += String(buttonMap[index]);
      _newAddress.setTextAsNumber(_newAddressString.toInt());
    }
  }
}

void NextionAddressPage::removeNumber(const NextionButton *button) {
  if(_newAddressString.length() > 0) {
    // remove last character
    _newAddressString.remove(_newAddressString.length() - 1);
  }
  if(_newAddressString.length() == 0) {
    // if our string is empty, default to zero
    _newAddressString.concat("0");
  }
  _newAddress.setTextAsNumber(_newAddressString.toInt());
}

void NextionAddressPage::displayPage() {
  _newAddressString = "";
  _currentAddress.setTextAsNumber(_address);
  _newAddress.setTextAsNumber(0);
}

//
/************************************************************************************************************/
// Throttle Page
/************************************************************************************************************/
//
NextionThrottlePage::NextionThrottlePage(Nextion &nextion) : DCCPPNextionPage(nextion, THROTTLE_PAGE, "2"),
  _activeLoco(0),
  _activeFunctionGroup(0),
  _locoNumbers {3, 4, 5},
  _locomotives {nullptr, nullptr, nullptr},
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
  _fwdButton(nextion, THROTTLE_PAGE, fwd, "Fwd"),
  _revButton(nextion, THROTTLE_PAGE, rev, "Rev"),
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
    NextionButton(nextion, THROTTLE_PAGE, clear, "Clear")
  },
  _locoAddress(nextion, THROTTLE_PAGE, locoaddr, "LocoAddr"),
  _routes(nextion, THROTTLE_PAGE, 34, "b2"),
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
    _locoButtons[index].setNumberProperty("pic", LOCO_PIC_OFF);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if(button == &_locoButtons[index]) {
      _locoButtons[index].setNumberProperty("pic", LOCO_PIC_ON);
      _activeLoco = index;
    }
  }
  refreshLocomotiveDetails();
}

void NextionThrottlePage::activateFunctionGroup(const NextionButton *button) {
  // turn off all buttons
  uint8_t fgrp_buts[6] = {FG1_PIC_OFF, FG2_PIC_OFF, FG3_PIC_OFF, FG1_PIC_ON, FG2_PIC_ON, FG3_PIC_ON};
  for(int index = 0; index < 3; index++) {
    _fgroupButtons[index].setNumberProperty("pic", fgrp_buts[index]);
  }
  // find and activate the selected button
  for(int index = 0; index < 3; index++) {
    if (button == &_fgroupButtons[index]) {
      _fgroupButtons[index].setNumberProperty("pic", fgrp_buts[index + 3]);
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

void NextionThrottlePage::toggleFunction(const NextionButton *button) {
  if(_locomotives[_activeLoco] != nullptr) {
    for(uint8_t function = 0; function < 10; function++) {
      if(&_functionButtons[function] == button) {
        if(function == 8) { // Front Light
          _locomotives[_activeLoco]->setFunction(0, !_locomotives[_activeLoco]->isFunctionEnabled(0));
        } else if(function == 9) { // Clear all 28 functions... 29?
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
  for(auto entry : LocomotiveManager::getDefaultLocos(3)) {
    _locomotives[index++] = LocomotiveManager::getLocomotive(entry->getAddress());
  }
  activateLoco(&_locoButtons[0]);
  activateFunctionGroup(&_fgroupButtons[0]);
}

void NextionThrottlePage::displayPage() {
  _locoButtons[_activeLoco].setNumberProperty("pic", LOCO_PIC_ON);
  _fgroupButtons[_activeFunctionGroup].setNumberProperty("pic", FG1_PIC_ON);
  refreshLocomotiveDetails();
}

void NextionThrottlePage::previousPageCallback(DCCPPNextionPage *previousPage) {
  changeLocoAddress(static_cast<NextionAddressPage *>(previousPage)->getNewAddress());
}

void NextionThrottlePage::refreshLocomotiveDetails()
{
  for(int index = 0; index < 3; index++) {
    if(_locomotives[index] != nullptr) {
      _locoButtons[index].setTextAsNumber(_locoNumbers[index]);
    } else {
      _locoButtons[index].setText("");
    }
  }
  if(_locomotives[_activeLoco] != nullptr) {
    _speedSlider.setValue(_locomotives[_activeLoco]->getSpeed());
    _speedNumber.setTextAsNumber(_locomotives[_activeLoco]->getSpeed());
  } else {
    _speedSlider.setValue(0);
    _speedNumber.setTextAsNumber(0);
  }
  refreshLocomotiveDirection();
  refreshFunctionButtons();
}

void NextionThrottlePage::refreshFunctionButtons() {
  for(int index = 0; index < 8; index++) {
    if(_locomotives[_activeLoco] != nullptr && _locomotives[_activeLoco]->isFunctionEnabled(_activeFunctionGroup * 8 + index + 1)) {
      _functionButtons[index].setNumberProperty("pic", (_activeFunctionGroup * 8 + index + F1_PIC_ON));
    } else {
      _functionButtons[index].setNumberProperty("pic", _activeFunctionGroup * 8 + index + F1_PIC_OFF);
    }
  }
}

void NextionThrottlePage::refreshLocomotiveDirection() {
  if(_locomotives[_activeLoco] != nullptr) {
    if(_locomotives[_activeLoco]->isDirectionForward()) {
      _fwdButton.setNumberProperty("pic", FWD_PIC_ON);
      _revButton.setNumberProperty("pic", REV_PIC_OFF);
    } else {
      _fwdButton.setNumberProperty("pic", FWD_PIC_OFF);
      _revButton.setNumberProperty("pic", REV_PIC_ON);
    }
  } else {
    _fwdButton.setNumberProperty("pic", FWD_PIC_ON);
    _revButton.setNumberProperty("pic", REV_PIC_OFF);
  }
}
//
/************************************************************************************************************/
// Turnout Page
/************************************************************************************************************/
//
NextionTurnoutPage::NextionTurnoutPage(Nextion &nextion) :
  DCCPPNextionPage(nextion, TURNOUT_PAGE, "3"),
  _turnoutButtons {
    NextionButton(nextion, TURNOUT_PAGE, slot0, "To0"),
    NextionButton(nextion, TURNOUT_PAGE, slot1, "To1"),
    NextionButton(nextion, TURNOUT_PAGE, slot2, "To2"),
    NextionButton(nextion, TURNOUT_PAGE, slot3, "To3"),
    NextionButton(nextion, TURNOUT_PAGE, slot4, "To4"),
    NextionButton(nextion, TURNOUT_PAGE, slot5, "To5"),
    NextionButton(nextion, TURNOUT_PAGE, slot6, "To6"),
    NextionButton(nextion, TURNOUT_PAGE, slot7, "To7"),
    NextionButton(nextion, TURNOUT_PAGE, slot8, "To8"),
    NextionButton(nextion, TURNOUT_PAGE, slot9, "To9"),
    NextionButton(nextion, TURNOUT_PAGE, slot10, "To10"),
    NextionButton(nextion, TURNOUT_PAGE, slot11, "To11"),
    NextionButton(nextion, TURNOUT_PAGE, slot12, "To12"),
    NextionButton(nextion, TURNOUT_PAGE, slot13, "To13"),
    NextionButton(nextion, TURNOUT_PAGE, slot14, "To14")
  },
  _backButton(nextion, TURNOUT_PAGE, back, "Back"),
  _prevButton(nextion, TURNOUT_PAGE, prev, "Prev"),
  _nextButton(nextion, TURNOUT_PAGE, next, "Next"),
  _addButton(nextion, TURNOUT_PAGE, addto, "Add"),
  _delButton(nextion, TURNOUT_PAGE, del, "Del"),
  _routesButton(nextion, TURNOUT_PAGE, routes, "Routes"),
  _toAddress {
    NextionButton(nextion, TURNOUT_PAGE, slot0, "To0"),
    NextionButton(nextion, TURNOUT_PAGE, slot1, "To1"),
    NextionButton(nextion, TURNOUT_PAGE, slot2, "To2"),
    NextionButton(nextion, TURNOUT_PAGE, slot3, "To3"),
    NextionButton(nextion, TURNOUT_PAGE, slot4, "To4"),
    NextionButton(nextion, TURNOUT_PAGE, slot5, "To5"),
    NextionButton(nextion, TURNOUT_PAGE, slot6, "To6"),
    NextionButton(nextion, TURNOUT_PAGE, slot7, "To7"),
    NextionButton(nextion, TURNOUT_PAGE, slot8, "To8"),
    NextionButton(nextion, TURNOUT_PAGE, slot9, "To9"),
    NextionButton(nextion, TURNOUT_PAGE, slot10, "To10"),
    NextionButton(nextion, TURNOUT_PAGE, slot11, "To11"),
    NextionButton(nextion, TURNOUT_PAGE, slot12, "To12"),
    NextionButton(nextion, TURNOUT_PAGE, slot13, "To13"),
    NextionButton(nextion, TURNOUT_PAGE, slot14, "To14")
  } {
  for(int slot = 0; slot < 15; slot++) {
    _turnoutButtons[slot].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->toggleTurnout(static_cast<NextionButton *>(widget));
      }
    });
  };

  _routesButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      log_i("Routes Button Pressed");
    }
  });

  _prevButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      log_i("Prev Button Pressed");
    }
  });

  _nextButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      log_i("Next Button Pressed");
    }
  });

  _addButton.attachCallback([](NextionEventType type, INextionTouchable *widget)
  {
    if(type == NEX_EVENT_PUSH) {
      log_i("Add Button Pressed");
    }
  });

  _delButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      log_i("Del Button Pressed");
    }
  });

  _backButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
}

void NextionTurnoutPage::displayPage() {
  int startIndex = 0;
  uint16_t turnoutsToDisplay = TurnoutManager::getTurnoutCount() - startIndex;
  // make sure that we only ever display a maximum of 15 turnouts per page
  if(turnoutsToDisplay > 15) {
    turnoutsToDisplay = 15;
  }
  for(uint8_t componentIndex = 0; componentIndex < turnoutsToDisplay; componentIndex++) {
    Turnout *turnout = TurnoutManager::getTurnout(startIndex + componentIndex);
    _turnoutButtons[componentIndex].setNumberProperty("pic", (RH + (turnout->getOrientation()) + (turnout->isThrown())));
    _toAddress[componentIndex].setTextAsNumber(turnout->getID());
  }
}

void NextionTurnoutPage::previousPageCallback(DCCPPNextionPage *previousPage) {
  // static_cast<NextionAddressPage *>(previousPage)->getNewAddress();
}

void NextionTurnoutPage::toggleTurnout(const NextionButton *button) {
  for(uint8_t slot = 0; slot < 15; slot++) {
    if(&_turnoutButtons[slot] == button) {
      log_i("Toggle slot %d activated", slot);
      //This is where logic to activate the turnout needs to go
      _turnoutButtons[slot].setNumberProperty("pic", LH); //temporary wrong image
      // toggle the turnout state
      TurnoutManager::toggle(_toAddress[slot].getTextAsNumber());
    }
  }
}

/*
void NextionTurnoutPage::setReturnPage()
{
}
*/
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

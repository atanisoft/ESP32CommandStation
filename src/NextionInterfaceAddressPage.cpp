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

constexpr uint8_t oldaddr=7; //OldAddr
constexpr uint8_t newaddr=8; //NewAddr
constexpr uint8_t num1=9;    //b1 PIC 72/82
constexpr uint8_t num2=10;   //b2 PIC 73/83
constexpr uint8_t num3=11;   //b3 PIC 74/84
constexpr uint8_t num4=12;   //b4 PIC 75/85
constexpr uint8_t num5=13;   //b5 PIC 76/86
constexpr uint8_t num6=14;   //b6 PIC 77/87
constexpr uint8_t num7=15;   //b7 PIC 78/88
constexpr uint8_t num8=16;   //b8 PIC 79/89
constexpr uint8_t num9=17;   //b9 PIC 80/90
constexpr uint8_t num0=18;   //b0 PIC 81/91
constexpr uint8_t save1=19;  //save PIC 92/93
constexpr uint8_t erase=20;
constexpr uint8_t quit1=21;  //quit PIC 94/95
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
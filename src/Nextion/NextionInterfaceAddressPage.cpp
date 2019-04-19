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
constexpr uint8_t undo=20;
constexpr uint8_t quit1=21;  //quit PIC 94/95
constexpr uint8_t addrtype=22;  //AddrType PIC129/130
constexpr uint8_t boardaddress=5;
constexpr uint8_t indexaddress=6;
constexpr uint8_t orientation=23;

constexpr uint8_t LOCO_PIC=140;
constexpr uint8_t TURNOUT_PIC=141;
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
  _addressPic(nextion, ADDRESS_PAGE, addrtype, "AddrType"),
  _boardAddress(nextion, ADDRESS_PAGE, boardaddress, "boardAddress"),
  _indexAddress(nextion, ADDRESS_PAGE, indexaddress, "indexAddress"),
  _turnoutTypeButton(nextion, ADDRESS_PAGE, orientation, "Orientation"),
  _saveButton(nextion, ADDRESS_PAGE, save1, "Save"),
  _quitButton(nextion, ADDRESS_PAGE, quit1, "Quit"),
  _undoButton(nextion, ADDRESS_PAGE, undo, "Undo"),
  _currentAddress(nextion, ADDRESS_PAGE, oldaddr, "OldAddr"),
  _newAddress(nextion, ADDRESS_PAGE, newaddr, "NewAddr") {
  for(int index = 0; index < 10; index++) {
    _buttons[index].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE])->addNumber(static_cast<NextionButton *>(widget));
      }
    });
  }
  _saveButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->displayPreviousPage();
    }
  });
  _quitButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
  _undoButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE])->removeNumber(static_cast<NextionButton *>(widget));
    }
  });
  _turnoutTypeButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if (type == NEX_EVENT_PUSH) {
      static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE])->changeTurnoutType(static_cast<NextionButton *>(widget));
    }
  });
}

void NextionAddressPage::addNumber(const NextionButton *button) {
  static const uint8_t buttonMap[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
  for(int index = 0; index < 10; index++) {
    if(button == &_buttons[index]) {
      _newAddressString += String(buttonMap[index]);
      _newAddress.setTextAsNumber(_newAddressString.toInt());
      // if it is a turnout, calculate and display the board address and index
      if(_addressPic.getPictureID() == TURNOUT_PIC) {
        uint16_t boardAddress = 0;
        uint8_t boardIndex = 0;
        calculateTurnoutBoardAddressAndIndex(&boardAddress, &boardIndex, _newAddressString.toInt());
        _boardAddress.setTextAsNumber(boardAddress);
        _indexAddress.setTextAsNumber(boardIndex);
      }
    }
  }
}

void NextionAddressPage::changeTurnoutType(const NextionButton *button) {
  _turnoutType++;
  _turnoutType %= TurnoutType::MAX_TURNOUT_TYPES;
  refreshTurnoutTypeButton();
  LOG(VERBOSE, "[Nextion] TurnoutType set to %d", _turnoutType);
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

  if(_addressPic.getPictureID() == TURNOUT_PIC) {
    uint16_t boardAddress = 0;
    uint8_t boardIndex = 0;
    calculateTurnoutBoardAddressAndIndex(&boardAddress, &boardIndex, _newAddressString.toInt());
    _boardAddress.setTextAsNumber(boardAddress);
    _indexAddress.setTextAsNumber(boardIndex);
  }
}

void NextionAddressPage::displayPage() {
  if(_address) {
    _currentAddress.setTextAsNumber(_address);
    _currentAddress.show();
  }
  _newAddressString = "";
  _newAddress.setTextAsNumber(0);
  if (getReturnPage() == THROTTLE_PAGE) {
    _addressPic.setPictureID(LOCO_PIC);
    _boardAddress.hide();
    _indexAddress.hide();
    _turnoutTypeButton.hide();
  } else {
    _addressPic.setPictureID(TURNOUT_PIC);
    _boardAddress.setTextAsNumber(0);
    _indexAddress.setTextAsNumber(0);
    _boardAddress.show();
    _indexAddress.show();
    refreshTurnoutTypeButton();
  }
}

void NextionAddressPage::refreshTurnoutTypeButton() {
  switch(_turnoutType) {
    case TurnoutType::LEFT:
      _turnoutTypeButton.setPictureID(TURNOUT_IMAGE_IDS::LEFT_HAND_THROWN);
      break;
    case TurnoutType::RIGHT:
      _turnoutTypeButton.setPictureID(TURNOUT_IMAGE_IDS::RIGHT_HAND_THROWN);
      break;
    case TurnoutType::WYE:
      _turnoutTypeButton.setPictureID(TURNOUT_IMAGE_IDS::WYE_THROWN_LEFT);
      break;
    case TurnoutType::MULTI:
      _turnoutTypeButton.setPictureID(TURNOUT_IMAGE_IDS::MULTI_STRAIGHT);
      break;
  }
  _turnoutTypeButton.show();
}

#endif

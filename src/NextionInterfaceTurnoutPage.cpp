/**********************************************************************
DCC++ BASE STATION FOR ESP32

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

constexpr uint8_t LH=106;
constexpr uint8_t RH=108;

constexpr uint8_t TO_LH_CLOSED=106;
constexpr uint8_t TO_LH_THROWN=107;
constexpr uint8_t TO_RH_CLOSED=108;
constexpr uint8_t TO_RH_THROWN=109;

constexpr uint8_t NEXT_BUTTON_ENABLED=110;
constexpr uint8_t NEXT_BUTTON_DISABLED=126;
constexpr uint8_t PREV_BUTTON_ENABLED=111;
constexpr uint8_t PREV_BUTTON_DISABLED=125;

constexpr uint8_t slot0=4;
constexpr uint8_t slot1=5;
constexpr uint8_t slot2=6;
constexpr uint8_t slot3=7;
constexpr uint8_t slot4=8;

constexpr uint8_t slot5=9;
constexpr uint8_t slot6=10;
constexpr uint8_t slot7=11;
constexpr uint8_t slot8=12;
constexpr uint8_t slot9=13;

constexpr uint8_t slot10=14;
constexpr uint8_t slot11=15;
constexpr uint8_t slot12=16;
constexpr uint8_t slot13=17;
constexpr uint8_t slot14=18;

constexpr uint8_t ad0=25;
constexpr uint8_t ad1=26;
constexpr uint8_t ad2=27;
constexpr uint8_t ad3=28;
constexpr uint8_t ad4=29;

constexpr uint8_t ad5=30;
constexpr uint8_t ad6=31;
constexpr uint8_t ad7=32;
constexpr uint8_t ad8=33;
constexpr uint8_t ad9=34;

constexpr uint8_t ad10=35;
constexpr uint8_t ad11=36;
constexpr uint8_t ad12=37;
constexpr uint8_t ad13=38;
constexpr uint8_t ad14=39;

constexpr uint8_t prev=19;
constexpr uint8_t addto=20;
constexpr uint8_t next=21;
constexpr uint8_t back=22;
constexpr uint8_t del=23;
constexpr uint8_t routes=24;

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
    NextionButton(nextion, TURNOUT_PAGE, slot0, "Ad0"),
    NextionButton(nextion, TURNOUT_PAGE, slot1, "Ad1"),
    NextionButton(nextion, TURNOUT_PAGE, slot2, "Ad2"),
    NextionButton(nextion, TURNOUT_PAGE, slot3, "Ad3"),
    NextionButton(nextion, TURNOUT_PAGE, slot4, "Ad4"),
    NextionButton(nextion, TURNOUT_PAGE, slot5, "Ad5"),
    NextionButton(nextion, TURNOUT_PAGE, slot6, "Ad6"),
    NextionButton(nextion, TURNOUT_PAGE, slot7, "Ad7"),
    NextionButton(nextion, TURNOUT_PAGE, slot8, "Ad8"),
    NextionButton(nextion, TURNOUT_PAGE, slot9, "Ad9"),
    NextionButton(nextion, TURNOUT_PAGE, slot10, "Ad10"),
    NextionButton(nextion, TURNOUT_PAGE, slot11, "Ad11"),
    NextionButton(nextion, TURNOUT_PAGE, slot12, "Ad12"),
    NextionButton(nextion, TURNOUT_PAGE, slot13, "Ad13"),
    NextionButton(nextion, TURNOUT_PAGE, slot14, "Ad14")
  }, _turnoutStartIndex(0) {
  for(int slot = 0; slot < 15; slot++) {
    _turnoutButtons[slot].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->toggleTurnout(static_cast<NextionButton *>(widget));
      }
    });
  };

  _routesButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      printf("Routes Button Pressed\n");
    }
  });

  _prevButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->decrementTurnoutPage();
    }
  });

  _nextButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->incrementTurnoutPage();
    }
  });

  _addButton.attachCallback([](NextionEventType type, INextionTouchable *widget)
  {
    if(type == NEX_EVENT_PUSH) {
      NextionAddressPage *addressPage = static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE]);
      addressPage->setPreviousPage(TURNOUT_PAGE);
      addressPage->display();
    }
  });

  _delButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      printf("Del Button Pressed\n");
      //auto turnout = TurnoutManager::getTurnoutByAddress(1);
      //turnout->setOrientation(LEFT);
    }
  });

  _backButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
}

void NextionTurnoutPage::displayPage() {
  // make sure that we only ever display a maximum of TURNOUTS_PER_PAGE turnouts per page
  uint16_t turnoutsToDisplay = min(TurnoutManager::getTurnoutCount() - _turnoutStartIndex, TURNOUTS_PER_PAGE);
  // update the number of turnouts we can display on the page
  for(uint8_t componentIndex = 0; componentIndex < turnoutsToDisplay; componentIndex++) {
    auto turnout = TurnoutManager::getTurnoutByID(_turnoutStartIndex + componentIndex);
    if(turnout) {
      _turnoutButtons[componentIndex].setPictureID(LH + (turnout->getOrientation() * 2) + (turnout->isThrown()));
      _turnoutButtons[componentIndex].show();
      _toAddress[componentIndex].setTextAsNumber(turnout->getAddress());
      _toAddress[componentIndex].show();
    } 
  }
  // check if we need to show/hide any components on the page
  if(turnoutsToDisplay < TURNOUTS_PER_PAGE) {
    // we are displaying a partial page of turnouts, so hide the remaining
    // buttons and the next page button
    for (; turnoutsToDisplay < TURNOUTS_PER_PAGE; turnoutsToDisplay++)
    {
      _turnoutButtons[turnoutsToDisplay].hide();
      _toAddress[turnoutsToDisplay].hide();
    }
    _nextButton.setPictureID(NEXT_BUTTON_DISABLED);
    _nextButton.disable();
  } else if(turnoutsToDisplay == TURNOUTS_PER_PAGE && 
     TurnoutManager::getTurnoutCount() > (turnoutsToDisplay + _turnoutStartIndex)) {
    // we are displaying a full page of turnouts and there is at least one
    // additional turnout to display so show the next page button
    _nextButton.setPictureID(NEXT_BUTTON_ENABLED);
    _nextButton.enable();
  }
  // if we are not on the first page of turnouts display the previous page button
  if(_turnoutStartIndex) {
    _prevButton.setPictureID(PREV_BUTTON_ENABLED);
    _prevButton.enable();
  } else {
    // first page of turnouts, hide the previous page button
    _prevButton.setPictureID(PREV_BUTTON_DISABLED);
    _prevButton.disable();
  }
}

void NextionTurnoutPage::previousPageCallback(DCCPPNextionPage *previousPage) {
  addNewTurnoutAddress(static_cast<NextionAddressPage *>(previousPage)->getNewAddress());
}

void NextionTurnoutPage::addNewTurnoutAddress(uint32_t newAddress) {
  log_i("and the received address is:- %d", newAddress);
}

void NextionTurnoutPage::toggleTurnout(const NextionButton *button) {
  for(uint8_t slot = 0; slot < TURNOUTS_PER_PAGE; slot++) {
    if(&_turnoutButtons[slot] == button) {
      auto turnoutAddress = _toAddress[slot].getTextAsNumber();
      auto turnout = TurnoutManager::getTurnoutByAddress(turnoutAddress);
      if(turnout) {
        turnout->set(!turnout->isThrown());
        _turnoutButtons[slot].setPictureID(LH + (turnout->getOrientation() * 2) + (turnout->isThrown()));
      } else {
        log_w("Turnout not found");
      }
    }
  }
}

#endif
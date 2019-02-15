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

constexpr uint8_t NEXT_BUTTON_DISABLED=122;
constexpr uint8_t NEXT_BUTTON_ENABLED=123;
constexpr uint8_t PREV_BUTTON_DISABLED=125;
constexpr uint8_t PREV_BUTTON_ENABLED=126;

constexpr uint8_t DELETE_INACTIVE=136;
constexpr uint8_t DELETE_ACTIVE=137;

constexpr uint8_t slot0ButtonID=4;
constexpr uint8_t slot1ButtonID=5;
constexpr uint8_t slot2ButtonID=6;
constexpr uint8_t slot3ButtonID=7;
constexpr uint8_t slot4ButtonID=8;

constexpr uint8_t slot5ButtonID=9;
constexpr uint8_t slot6ButtonID=10;
constexpr uint8_t slot7ButtonID=11;
constexpr uint8_t slot8ButtonID=12;
constexpr uint8_t slot9ButtonID=13;

constexpr uint8_t slot10ButtonID=14;
constexpr uint8_t slot11ButtonID=15;
constexpr uint8_t slot12ButtonID=16;
constexpr uint8_t slot13ButtonID=17;
constexpr uint8_t slot14ButtonID=18;

constexpr uint8_t ad0ButtonID=24;
constexpr uint8_t ad1ButtonID=25;
constexpr uint8_t ad2ButtonID=26;
constexpr uint8_t ad3ButtonID=27;
constexpr uint8_t ad4ButtonID=28;

constexpr uint8_t ad5ButtonID=29;
constexpr uint8_t ad6ButtonID=30;
constexpr uint8_t ad7ButtonID=31;
constexpr uint8_t ad8ButtonID=32;
constexpr uint8_t ad9ButtonID=33;

constexpr uint8_t ad10ButtonID=34;
constexpr uint8_t ad11ButtonID=35;
constexpr uint8_t ad12ButtonID=36;
constexpr uint8_t ad13ButtonID=37;
constexpr uint8_t ad14ButtonID=38;

constexpr uint8_t prevButtonID=19;
constexpr uint8_t addtoButtonID=20;
constexpr uint8_t nextButtonID=21;
constexpr uint8_t backButtonID=22;
constexpr uint8_t delButtonID=39;
constexpr uint8_t setupButtonID=23;

//
/************************************************************************************************************/
// Turnout Page
/************************************************************************************************************/
//
NextionTurnoutPage::NextionTurnoutPage(Nextion &nextion) :
  DCCPPNextionPage(nextion, TURNOUT_PAGE, "3"),
  _turnoutButtons {
    NextionButton(nextion, TURNOUT_PAGE, slot0ButtonID, "To0"),
    NextionButton(nextion, TURNOUT_PAGE, slot1ButtonID, "To1"),
    NextionButton(nextion, TURNOUT_PAGE, slot2ButtonID, "To2"),
    NextionButton(nextion, TURNOUT_PAGE, slot3ButtonID, "To3"),
    NextionButton(nextion, TURNOUT_PAGE, slot4ButtonID, "To4"),
    NextionButton(nextion, TURNOUT_PAGE, slot5ButtonID, "To5"),
    NextionButton(nextion, TURNOUT_PAGE, slot6ButtonID, "To6"),
    NextionButton(nextion, TURNOUT_PAGE, slot7ButtonID, "To7"),
    NextionButton(nextion, TURNOUT_PAGE, slot8ButtonID, "To8"),
    NextionButton(nextion, TURNOUT_PAGE, slot9ButtonID, "To9"),
    NextionButton(nextion, TURNOUT_PAGE, slot10ButtonID, "To10"),
    NextionButton(nextion, TURNOUT_PAGE, slot11ButtonID, "To11"),
    NextionButton(nextion, TURNOUT_PAGE, slot12ButtonID, "To12"),
    NextionButton(nextion, TURNOUT_PAGE, slot13ButtonID, "To13"),
    NextionButton(nextion, TURNOUT_PAGE, slot14ButtonID, "To14")
  },
  _toAddress {
    NextionButton(nextion, TURNOUT_PAGE, ad0ButtonID, "Ad0"),
    NextionButton(nextion, TURNOUT_PAGE, ad1ButtonID, "Ad1"),
    NextionButton(nextion, TURNOUT_PAGE, ad2ButtonID, "Ad2"),
    NextionButton(nextion, TURNOUT_PAGE, ad3ButtonID, "Ad3"),
    NextionButton(nextion, TURNOUT_PAGE, ad4ButtonID, "Ad4"),
    NextionButton(nextion, TURNOUT_PAGE, ad5ButtonID, "Ad5"),
    NextionButton(nextion, TURNOUT_PAGE, ad6ButtonID, "Ad6"),
    NextionButton(nextion, TURNOUT_PAGE, ad7ButtonID, "Ad7"),
    NextionButton(nextion, TURNOUT_PAGE, ad8ButtonID, "Ad8"),
    NextionButton(nextion, TURNOUT_PAGE, ad9ButtonID, "Ad9"),
    NextionButton(nextion, TURNOUT_PAGE, ad10ButtonID, "Ad10"),
    NextionButton(nextion, TURNOUT_PAGE, ad11ButtonID, "Ad11"),
    NextionButton(nextion, TURNOUT_PAGE, ad12ButtonID, "Ad12"),
    NextionButton(nextion, TURNOUT_PAGE, ad13ButtonID, "Ad13"),
    NextionButton(nextion, TURNOUT_PAGE, ad14ButtonID, "Ad14")
  },
  _backButton(nextion, TURNOUT_PAGE, backButtonID, "Back"),
  _prevButton(nextion, TURNOUT_PAGE, prevButtonID, "Prev"),
  _nextButton(nextion, TURNOUT_PAGE, nextButtonID, "Next"),
  _addButton(nextion, TURNOUT_PAGE, addtoButtonID, "Add"),
  _delButton(nextion, TURNOUT_PAGE, delButtonID, "Del"),
  _setupButton(nextion, TURNOUT_PAGE, setupButtonID, "Setup") {
  for(int slot = 0; slot < 15; slot++) {
    _turnoutButtons[slot].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->toggleTurnout(static_cast<NextionButton *>(widget));
      }
    });
  };

  _setupButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      printf("Setup Button Pressed\n");
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
      static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->addNewTurnout();
    }
  });

  _delButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->deleteButtonHandler();
    }
  });

  _backButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
}

void NextionTurnoutPage::deleteButtonHandler() {
  if(_pageMode == PAGE_MODE::DELETION) {
    for(uint8_t slot = 0; slot < TURNOUTS_PER_PAGE; slot++) {
      if(_turnoutButtons[slot].getPictureID() == TURNOUT_IMAGE_IDS::TURNOUT_DELETED) {
        TurnoutManager::removeByAddress(_toAddress[slot].getTextAsNumber());
        log_i("Turnout(%d) deleted from slot(%d)", _toAddress[slot].getTextAsNumber(), slot);
      }
    }
    _delButton.setPictureID(DELETE_INACTIVE);
    _pageMode = PAGE_MODE::NORMAL;
  } else {
    _delButton.setPictureID(DELETE_ACTIVE);
    _pageMode = PAGE_MODE::DELETION;
  }
  refresh();
}

void NextionTurnoutPage::refreshPage() {
  // make sure that we only ever display a maximum of TURNOUTS_PER_PAGE turnouts per page
  uint16_t turnoutsToDisplay = min(TurnoutManager::getTurnoutCount() - _turnoutStartIndex, TURNOUTS_PER_PAGE);

  while(turnoutsToDisplay == 0 && _turnoutStartIndex >= TURNOUTS_PER_PAGE) {
    // we have overrun the list of turnouts (possibly from deletion)
    _turnoutStartIndex -= TURNOUTS_PER_PAGE;
    if(_turnoutStartIndex < 0) {
      _turnoutStartIndex = 0;
    }
    // recalcuate the number of turnouts to display
    turnoutsToDisplay = min(TurnoutManager::getTurnoutCount() - _turnoutStartIndex, TURNOUTS_PER_PAGE);
  }

  // update the number of turnouts we can display on the page
  for(uint8_t componentIndex = 0; componentIndex < turnoutsToDisplay; componentIndex++) {
    auto turnout = TurnoutManager::getTurnoutByIndex(_turnoutStartIndex + componentIndex);
    if(turnout) {
      _turnoutButtons[componentIndex].setPictureID(getDefaultTurnoutPictureID(turnout));
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
  NextionAddressPage *addressPage = static_cast<NextionAddressPage *>(previousPage);
  TurnoutManager::createOrUpdate(TurnoutManager::getTurnoutCount(),
    addressPage->getNewAddress(), -1, addressPage->getOrientation());
  // reset page mode for normal operations
  _pageMode = PAGE_MODE::NORMAL;
  refresh();
}

void NextionTurnoutPage::toggleTurnout(const NextionButton *button) {
  for(uint8_t slot = 0; slot < TURNOUTS_PER_PAGE; slot++) {
    if(&_turnoutButtons[slot] == button) {
      auto turnoutAddress = _toAddress[slot].getTextAsNumber();
      auto turnout = TurnoutManager::getTurnoutByAddress(turnoutAddress);
      if(turnout) {
        if(_pageMode == PAGE_MODE::DELETION) {
          if(_turnoutButtons[slot].getPictureID() == TURNOUT_IMAGE_IDS::TURNOUT_DELETED) {
            _turnoutButtons[slot].setPictureID(getDefaultTurnoutPictureID(turnout));
          } else {
            _turnoutButtons[slot].setPictureID(TURNOUT_IMAGE_IDS::TURNOUT_DELETED);
          }
        } else if(_pageMode == PAGE_MODE::EDIT) {
          // TODO: pop up address page for edits
        } else if(_pageMode == PAGE_MODE::NORMAL) {
          turnout->toggle();
          _turnoutButtons[slot].setPictureID(getDefaultTurnoutPictureID(turnout));
        }
      } else {
        log_w("Touched Turnout (slot:%d, addr:%d) was not found, refreshing page", slot, turnoutAddress);
        // since we couldn't find the touched turnout, reset the page mode and refresh the page
        _pageMode = PAGE_MODE::NORMAL;
        refreshPage();
      }
    }
  }
}

uint8_t NextionTurnoutPage::getDefaultTurnoutPictureID(Turnout *turnout) {
  switch(turnout->getOrientation()) {
    case TurnoutOrientation::LEFT:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::LEFT_HAND_THROWN;
      }
      return TURNOUT_IMAGE_IDS::LEFT_HAND_CLOSED;
    case TurnoutOrientation::RIGHT:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::RIGHT_HAND_THROWN;
      }
      return TURNOUT_IMAGE_IDS::RIGHT_HAND_CLOSED;
    case TurnoutOrientation::WYE:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::WYE_THROWN_RIGHT;
      }
      return TURNOUT_IMAGE_IDS::WYE_THROWN_LEFT;
    case TurnoutOrientation::MULTI:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::MULTI_STRAIGHT;
      }
      return TURNOUT_IMAGE_IDS::MULTI_CROSS_OVER;
  }
  // TODO: update this with a better image like a ?
  return TURNOUT_IMAGE_IDS::LEFT_HAND_CLOSED;
}
#endif
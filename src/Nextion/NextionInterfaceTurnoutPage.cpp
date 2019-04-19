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

constexpr uint8_t LH=106;
constexpr uint8_t RH=108;

constexpr uint8_t NEXT_BUTTON_DISABLED=122;
constexpr uint8_t NEXT_BUTTON_ENABLED=123;
constexpr uint8_t PREV_BUTTON_DISABLED=125;
constexpr uint8_t PREV_BUTTON_ENABLED=126;

constexpr uint8_t DELETE_INACTIVE=136;
constexpr uint8_t DELETE_ACTIVE=137;

constexpr uint8_t prevButtonID=4;
constexpr uint8_t addtoButtonID=5;
constexpr uint8_t nextButtonID=6;
constexpr uint8_t backButtonID=7;
constexpr uint8_t delButtonID=8;
constexpr uint8_t setupButtonID=9;

// 3.2" and 3.5" display
constexpr uint8_t ad0ButtonID=10;
constexpr uint8_t ad1ButtonID=11;
constexpr uint8_t ad2ButtonID=12;
constexpr uint8_t ad3ButtonID=13;
constexpr uint8_t ad4ButtonID=14;
constexpr uint8_t ad5ButtonID=15;
constexpr uint8_t ad6ButtonID=16;
constexpr uint8_t ad7ButtonID=17;
constexpr uint8_t ad8ButtonID=18;
constexpr uint8_t ad9ButtonID=19;
constexpr uint8_t ad10ButtonID=20;
constexpr uint8_t ad11ButtonID=21;
constexpr uint8_t ad12ButtonID=22;
constexpr uint8_t ad13ButtonID=23;
constexpr uint8_t ad14ButtonID=24;

constexpr uint8_t slot0ButtonID=25;
constexpr uint8_t slot1ButtonID=26;
constexpr uint8_t slot2ButtonID=27;
constexpr uint8_t slot3ButtonID=28;
constexpr uint8_t slot4ButtonID=29;
constexpr uint8_t slot5ButtonID=30;
constexpr uint8_t slot6ButtonID=31;
constexpr uint8_t slot7ButtonID=32;
constexpr uint8_t slot8ButtonID=33;
constexpr uint8_t slot9ButtonID=34;
constexpr uint8_t slot10ButtonID=35;
constexpr uint8_t slot11ButtonID=36;
constexpr uint8_t slot12ButtonID=37;
constexpr uint8_t slot13ButtonID=38;
constexpr uint8_t slot14ButtonID=39;

// 3.5" display only
constexpr uint8_t ad15ButtonID=40;
constexpr uint8_t ad16ButtonID=41;
constexpr uint8_t ad17ButtonID=42;
constexpr uint8_t ad18ButtonID=43;
constexpr uint8_t ad19ButtonID=44;
constexpr uint8_t ad20ButtonID=45;
constexpr uint8_t ad21ButtonID=46;
constexpr uint8_t ad22ButtonID=47;
constexpr uint8_t ad23ButtonID=48;

constexpr uint8_t slot15ButtonID=49;
constexpr uint8_t slot16ButtonID=50;
constexpr uint8_t slot17ButtonID=51;
constexpr uint8_t slot18ButtonID=52;
constexpr uint8_t slot19ButtonID=53;
constexpr uint8_t slot20ButtonID=54;
constexpr uint8_t slot21ButtonID=55;
constexpr uint8_t slot22ButtonID=56;
constexpr uint8_t slot23ButtonID=57;

// 5.0" display only
constexpr uint8_t ad24ButtonID=58;
constexpr uint8_t ad25ButtonID=59;
constexpr uint8_t ad26ButtonID=60;
constexpr uint8_t ad27ButtonID=61;
constexpr uint8_t ad28ButtonID=62;
constexpr uint8_t ad29ButtonID=63;
constexpr uint8_t ad30ButtonID=64;
constexpr uint8_t ad31ButtonID=65;
constexpr uint8_t ad32ButtonID=66;
constexpr uint8_t ad33ButtonID=67;
constexpr uint8_t ad34ButtonID=68;
constexpr uint8_t ad35ButtonID=69;
constexpr uint8_t ad36ButtonID=70;
constexpr uint8_t ad37ButtonID=71;
constexpr uint8_t ad38ButtonID=72;
constexpr uint8_t ad39ButtonID=73;
constexpr uint8_t ad40ButtonID=74;
constexpr uint8_t ad41ButtonID=75;
constexpr uint8_t ad42ButtonID=76;
constexpr uint8_t ad43ButtonID=77;
constexpr uint8_t ad44ButtonID=78;
constexpr uint8_t ad45ButtonID=79;
constexpr uint8_t ad46ButtonID=80;
constexpr uint8_t ad47ButtonID=81;
constexpr uint8_t ad48ButtonID=82;
constexpr uint8_t ad49ButtonID=83;
constexpr uint8_t ad50ButtonID=84;
constexpr uint8_t ad51ButtonID=85;
constexpr uint8_t ad52ButtonID=86;
constexpr uint8_t ad53ButtonID=87;
constexpr uint8_t ad54ButtonID=88;
constexpr uint8_t ad55ButtonID=89;
constexpr uint8_t ad56ButtonID=90;
constexpr uint8_t ad57ButtonID=91;
constexpr uint8_t ad58ButtonID=92;
constexpr uint8_t ad59ButtonID=93;

constexpr uint8_t slot24ButtonID=94;
constexpr uint8_t slot25ButtonID=95;
constexpr uint8_t slot26ButtonID=96;
constexpr uint8_t slot27ButtonID=97;
constexpr uint8_t slot28ButtonID=98;
constexpr uint8_t slot29ButtonID=99;
constexpr uint8_t slot30ButtonID=100;
constexpr uint8_t slot31ButtonID=101;
constexpr uint8_t slot32ButtonID=102;
constexpr uint8_t slot33ButtonID=103;
constexpr uint8_t slot34ButtonID=104;
constexpr uint8_t slot35ButtonID=105;
constexpr uint8_t slot36ButtonID=106;
constexpr uint8_t slot37ButtonID=107;
constexpr uint8_t slot38ButtonID=108;
constexpr uint8_t slot39ButtonID=109;
constexpr uint8_t slot40ButtonID=110;
constexpr uint8_t slot41ButtonID=111;
constexpr uint8_t slot42ButtonID=112;
constexpr uint8_t slot43ButtonID=113;
constexpr uint8_t slot44ButtonID=114;
constexpr uint8_t slot45ButtonID=115;
constexpr uint8_t slot46ButtonID=116;
constexpr uint8_t slot47ButtonID=117;
constexpr uint8_t slot48ButtonID=118;
constexpr uint8_t slot49ButtonID=119;
constexpr uint8_t slot50ButtonID=120;
constexpr uint8_t slot51ButtonID=121;
constexpr uint8_t slot52ButtonID=122;
constexpr uint8_t slot53ButtonID=123;
constexpr uint8_t slot54ButtonID=124;
constexpr uint8_t slot55ButtonID=125;
constexpr uint8_t slot56ButtonID=126;
constexpr uint8_t slot57ButtonID=127;
constexpr uint8_t slot58ButtonID=128;
constexpr uint8_t slot59ButtonID=129;

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
    NextionButton(nextion, TURNOUT_PAGE, slot14ButtonID, "To14"),
    NextionButton(nextion, TURNOUT_PAGE, slot15ButtonID, "To15"),
    NextionButton(nextion, TURNOUT_PAGE, slot16ButtonID, "To16"),
    NextionButton(nextion, TURNOUT_PAGE, slot17ButtonID, "To17"),
    NextionButton(nextion, TURNOUT_PAGE, slot18ButtonID, "To18"),
    NextionButton(nextion, TURNOUT_PAGE, slot19ButtonID, "To19"),
    NextionButton(nextion, TURNOUT_PAGE, slot20ButtonID, "To20"),
    NextionButton(nextion, TURNOUT_PAGE, slot21ButtonID, "To21"),
    NextionButton(nextion, TURNOUT_PAGE, slot22ButtonID, "To22"),
    NextionButton(nextion, TURNOUT_PAGE, slot23ButtonID, "To23"),
    NextionButton(nextion, TURNOUT_PAGE, slot24ButtonID, "To24"),
    NextionButton(nextion, TURNOUT_PAGE, slot25ButtonID, "To25"),
    NextionButton(nextion, TURNOUT_PAGE, slot26ButtonID, "To26"),
    NextionButton(nextion, TURNOUT_PAGE, slot27ButtonID, "To27"),
    NextionButton(nextion, TURNOUT_PAGE, slot28ButtonID, "To28"),
    NextionButton(nextion, TURNOUT_PAGE, slot29ButtonID, "To29"),
    NextionButton(nextion, TURNOUT_PAGE, slot30ButtonID, "To30"),
    NextionButton(nextion, TURNOUT_PAGE, slot31ButtonID, "To31"),
    NextionButton(nextion, TURNOUT_PAGE, slot32ButtonID, "To32"),
    NextionButton(nextion, TURNOUT_PAGE, slot33ButtonID, "To33"),
    NextionButton(nextion, TURNOUT_PAGE, slot34ButtonID, "To34"),
    NextionButton(nextion, TURNOUT_PAGE, slot35ButtonID, "To35"),
    NextionButton(nextion, TURNOUT_PAGE, slot36ButtonID, "To36"),
    NextionButton(nextion, TURNOUT_PAGE, slot37ButtonID, "To37"),
    NextionButton(nextion, TURNOUT_PAGE, slot38ButtonID, "To38"),
    NextionButton(nextion, TURNOUT_PAGE, slot39ButtonID, "To39"),
    NextionButton(nextion, TURNOUT_PAGE, slot40ButtonID, "To40"),
    NextionButton(nextion, TURNOUT_PAGE, slot41ButtonID, "To41"),
    NextionButton(nextion, TURNOUT_PAGE, slot42ButtonID, "To42"),
    NextionButton(nextion, TURNOUT_PAGE, slot43ButtonID, "To43"),
    NextionButton(nextion, TURNOUT_PAGE, slot44ButtonID, "To44"),
    NextionButton(nextion, TURNOUT_PAGE, slot45ButtonID, "To45"),
    NextionButton(nextion, TURNOUT_PAGE, slot46ButtonID, "To46"),
    NextionButton(nextion, TURNOUT_PAGE, slot47ButtonID, "To47"),
    NextionButton(nextion, TURNOUT_PAGE, slot48ButtonID, "To48"),
    NextionButton(nextion, TURNOUT_PAGE, slot49ButtonID, "To49"),
    NextionButton(nextion, TURNOUT_PAGE, slot50ButtonID, "To50"),
    NextionButton(nextion, TURNOUT_PAGE, slot51ButtonID, "To51"),
    NextionButton(nextion, TURNOUT_PAGE, slot52ButtonID, "To52"),
    NextionButton(nextion, TURNOUT_PAGE, slot53ButtonID, "To53"),
    NextionButton(nextion, TURNOUT_PAGE, slot54ButtonID, "To54"),
    NextionButton(nextion, TURNOUT_PAGE, slot55ButtonID, "To55"),
    NextionButton(nextion, TURNOUT_PAGE, slot56ButtonID, "To56"),
    NextionButton(nextion, TURNOUT_PAGE, slot57ButtonID, "To57"),
    NextionButton(nextion, TURNOUT_PAGE, slot58ButtonID, "To58"),
    NextionButton(nextion, TURNOUT_PAGE, slot59ButtonID, "To59")
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
    NextionButton(nextion, TURNOUT_PAGE, ad14ButtonID, "Ad14"),
    NextionButton(nextion, TURNOUT_PAGE, ad15ButtonID, "Ad15"),
    NextionButton(nextion, TURNOUT_PAGE, ad16ButtonID, "Ad16"),
    NextionButton(nextion, TURNOUT_PAGE, ad17ButtonID, "Ad17"),
    NextionButton(nextion, TURNOUT_PAGE, ad18ButtonID, "Ad18"),
    NextionButton(nextion, TURNOUT_PAGE, ad19ButtonID, "Ad19"),
    NextionButton(nextion, TURNOUT_PAGE, ad20ButtonID, "Ad20"),
    NextionButton(nextion, TURNOUT_PAGE, ad21ButtonID, "Ad21"),
    NextionButton(nextion, TURNOUT_PAGE, ad22ButtonID, "Ad22"),
    NextionButton(nextion, TURNOUT_PAGE, ad23ButtonID, "Ad23"),
    NextionButton(nextion, TURNOUT_PAGE, ad24ButtonID, "Ad24"),
    NextionButton(nextion, TURNOUT_PAGE, ad25ButtonID, "Ad25"),
    NextionButton(nextion, TURNOUT_PAGE, ad26ButtonID, "Ad26"),
    NextionButton(nextion, TURNOUT_PAGE, ad27ButtonID, "Ad27"),
    NextionButton(nextion, TURNOUT_PAGE, ad28ButtonID, "Ad28"),
    NextionButton(nextion, TURNOUT_PAGE, ad29ButtonID, "Ad29"),
    NextionButton(nextion, TURNOUT_PAGE, ad30ButtonID, "Ad30"),
    NextionButton(nextion, TURNOUT_PAGE, ad31ButtonID, "Ad31"),
    NextionButton(nextion, TURNOUT_PAGE, ad32ButtonID, "Ad32"),
    NextionButton(nextion, TURNOUT_PAGE, ad33ButtonID, "Ad33"),
    NextionButton(nextion, TURNOUT_PAGE, ad34ButtonID, "Ad34"),
    NextionButton(nextion, TURNOUT_PAGE, ad35ButtonID, "Ad35"),
    NextionButton(nextion, TURNOUT_PAGE, ad36ButtonID, "Ad36"),
    NextionButton(nextion, TURNOUT_PAGE, ad37ButtonID, "Ad37"),
    NextionButton(nextion, TURNOUT_PAGE, ad38ButtonID, "Ad38"),
    NextionButton(nextion, TURNOUT_PAGE, ad39ButtonID, "Ad39"),
    NextionButton(nextion, TURNOUT_PAGE, ad40ButtonID, "Ad40"),
    NextionButton(nextion, TURNOUT_PAGE, ad41ButtonID, "Ad41"),
    NextionButton(nextion, TURNOUT_PAGE, ad42ButtonID, "Ad42"),
    NextionButton(nextion, TURNOUT_PAGE, ad43ButtonID, "Ad43"),
    NextionButton(nextion, TURNOUT_PAGE, ad44ButtonID, "Ad44"),
    NextionButton(nextion, TURNOUT_PAGE, ad45ButtonID, "Ad45"),
    NextionButton(nextion, TURNOUT_PAGE, ad46ButtonID, "Ad46"),
    NextionButton(nextion, TURNOUT_PAGE, ad47ButtonID, "Ad47"),
    NextionButton(nextion, TURNOUT_PAGE, ad48ButtonID, "Ad48"),
    NextionButton(nextion, TURNOUT_PAGE, ad49ButtonID, "Ad49"),
    NextionButton(nextion, TURNOUT_PAGE, ad50ButtonID, "Ad50"),
    NextionButton(nextion, TURNOUT_PAGE, ad51ButtonID, "Ad51"),
    NextionButton(nextion, TURNOUT_PAGE, ad52ButtonID, "Ad52"),
    NextionButton(nextion, TURNOUT_PAGE, ad53ButtonID, "Ad53"),
    NextionButton(nextion, TURNOUT_PAGE, ad54ButtonID, "Ad54"),
    NextionButton(nextion, TURNOUT_PAGE, ad55ButtonID, "Ad55"),
    NextionButton(nextion, TURNOUT_PAGE, ad56ButtonID, "Ad56"),
    NextionButton(nextion, TURNOUT_PAGE, ad57ButtonID, "Ad57"),
    NextionButton(nextion, TURNOUT_PAGE, ad58ButtonID, "Ad58"),
    NextionButton(nextion, TURNOUT_PAGE, ad59ButtonID, "Ad59")
  },
  _backButton(nextion, TURNOUT_PAGE, backButtonID, "Back"),
  _prevButton(nextion, TURNOUT_PAGE, prevButtonID, "Prev"),
  _nextButton(nextion, TURNOUT_PAGE, nextButtonID, "Next"),
  _addButton(nextion, TURNOUT_PAGE, addtoButtonID, "Add"),
  _delButton(nextion, TURNOUT_PAGE, delButtonID, "Del"),
  _setupButton(nextion, TURNOUT_PAGE, setupButtonID, "Setup") {
  for(int slot = 0; slot < TURNOUTS_PER_PAGE_5_0_DISPLAY; slot++) {
    _turnoutButtons[slot].attachCallback([](NextionEventType type, INextionTouchable *widget) {
      if(type == NEX_EVENT_PUSH) {
        static_cast<NextionTurnoutPage*>(nextionPages[TURNOUT_PAGE])->toggleTurnout(static_cast<NextionButton *>(widget));
      }
    });
  };

  _setupButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      LOG(INFO, "Setup Button Pressed");
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
    for(uint8_t slot = 0; slot < getTurnoutsPerPageCount(); slot++) {
      if(_turnoutButtons[slot].getPictureID() == TURNOUT_IMAGE_IDS::TURNOUT_DELETED) {
        TurnoutManager::removeByAddress(_toAddress[slot].getTextAsNumber());
        LOG(INFO, "[Nextion] Turnout(%d) deleted from slot(%d)", _toAddress[slot].getTextAsNumber(), slot);
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
  int maxTurnoutsPerPage = getTurnoutsPerPageCount();
  // make sure that we only ever display a maximum of TURNOUTS_PER_PAGE turnouts per page
  uint16_t turnoutsToDisplay = min(TurnoutManager::getTurnoutCount() - _turnoutStartIndex, maxTurnoutsPerPage);

  while(turnoutsToDisplay == 0 && _turnoutStartIndex >= maxTurnoutsPerPage) {
    // we have overrun the list of turnouts (possibly from deletion)
    _turnoutStartIndex -= maxTurnoutsPerPage;
    if(_turnoutStartIndex < 0) {
      _turnoutStartIndex = 0;
    }
    // recalcuate the number of turnouts to display
    turnoutsToDisplay = min(TurnoutManager::getTurnoutCount() - _turnoutStartIndex, maxTurnoutsPerPage);
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
  if(turnoutsToDisplay < maxTurnoutsPerPage) {
    // we are displaying a partial page of turnouts, so hide the remaining
    // buttons and the next page button
    for (; turnoutsToDisplay < maxTurnoutsPerPage; turnoutsToDisplay++)
    {
      _turnoutButtons[turnoutsToDisplay].hide();
      _toAddress[turnoutsToDisplay].hide();
    }
    _nextButton.setPictureID(NEXT_BUTTON_DISABLED);
    _nextButton.disable();
  } else if(turnoutsToDisplay == maxTurnoutsPerPage &&
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
    addressPage->getNewAddress(), -1, addressPage->getTurnoutType());
  // reset page mode for normal operations
  _pageMode = PAGE_MODE::NORMAL;
  refresh();
}

void NextionTurnoutPage::toggleTurnout(const NextionButton *button) {
  for(uint8_t slot = 0; slot < getTurnoutsPerPageCount(); slot++) {
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
        LOG(WARNING, "[Nextion] Touched Turnout (slot:%d, addr:%d) was not found, refreshing page", slot, turnoutAddress);
        // since we couldn't find the touched turnout, reset the page mode and refresh the page
        _pageMode = PAGE_MODE::NORMAL;
        refreshPage();
      }
    }
  }
}

uint8_t NextionTurnoutPage::getDefaultTurnoutPictureID(Turnout *turnout) {
  switch(turnout->getType()) {
    case TurnoutType::LEFT:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::LEFT_HAND_THROWN;
      }
      return TURNOUT_IMAGE_IDS::LEFT_HAND_CLOSED;
    case TurnoutType::RIGHT:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::RIGHT_HAND_THROWN;
      }
      return TURNOUT_IMAGE_IDS::RIGHT_HAND_CLOSED;
    case TurnoutType::WYE:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::WYE_THROWN_RIGHT;
      }
      return TURNOUT_IMAGE_IDS::WYE_THROWN_LEFT;
    case TurnoutType::MULTI:
      if(turnout->isThrown()) {
        return TURNOUT_IMAGE_IDS::MULTI_STRAIGHT;
      }
      return TURNOUT_IMAGE_IDS::MULTI_CROSS_OVER;
    default:
      // TODO: update this with a better image like a ?
      return TURNOUT_IMAGE_IDS::LEFT_HAND_CLOSED;
  }
  // TODO: update this with a better image like a ?
  return TURNOUT_IMAGE_IDS::LEFT_HAND_CLOSED;
}

uint8_t NextionTurnoutPage::getTurnoutsPerPageCount() {
  uint8_t maxTurnoutsPerPage = TURNOUTS_PER_PAGE_3_2_DISPLAY;
  if(nextionDeviceType == NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY ||
    nextionDeviceType == NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY) {
    maxTurnoutsPerPage = TURNOUTS_PER_PAGE_3_5_DISPLAY;
  } else if(nextionDeviceType == NEXTION_DEVICE_TYPE::BASIC_5_0_DISPLAY ||
    nextionDeviceType == NEXTION_DEVICE_TYPE::ENHANCED_5_0_DISPLAY) {
    maxTurnoutsPerPage = TURNOUTS_PER_PAGE_5_0_DISPLAY;
  }
  return maxTurnoutsPerPage;
}
#endif

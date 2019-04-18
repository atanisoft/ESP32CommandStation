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

#pragma once

#include "Locomotive.h"
#include "Turnouts.h"
#include <HardwareSerial.h>
#include <Nextion.h>
#include <NextionPage.h>
#include <NextionButton.h>
#include <NextionText.h>
#include <NextionSlider.h>
#include <NextionNumber.h>

void nextionInterfaceInit();

enum NEXTION_PAGES {
  TITLE_PAGE = 0,
  ADDRESS_PAGE = 1,
  THROTTLE_PAGE = 2,
  TURNOUT_PAGE = 3,
  CONFIG_PAGE = 4,
  ROUTES_PAGE = 5,
  MAX_PAGES
};

enum NEXTION_DEVICE_TYPE {
  BASIC_3_2_DISPLAY,
  BASIC_3_5_DISPLAY,
  BASIC_5_0_DISPLAY,
  ENHANCED_3_2_DISPLAY,
  ENHANCED_3_5_DISPLAY,
  ENHANCED_5_0_DISPLAY,
  UNKOWN_DISPLAY
};

class DCCPPNextionPage : public NextionPage {
public:
  DCCPPNextionPage(Nextion &, uint8_t, const String &);
  void display();
  void refresh();
  virtual void refreshPage() = 0;
  void setTrackPower(bool=true);
  void setPreviousPage(uint8_t pageID) {
    _returnPageID = pageID;
  }
  void displayPreviousPage(bool=true);
  void returnToPreviousPage() {
    displayPreviousPage(false);
  }
  uint8_t getReturnPage() {
    return _returnPageID;
  }
  void sendEStop();
protected:
  // One time page initialization
  virtual void init() = 0;

  // Called anytime the page gets displayed, should be used for refresh of components etc.
  virtual void displayPage() = 0;

  virtual void previousPageCallback(DCCPPNextionPage *previousPage) {}
private:
  NextionButton _onButton;
  NextionButton _stopButton;
  NextionButton _offButton;
  bool _pageInitialized{false};
  uint8_t _returnPageID{0};
  void refreshPowerButtons();
};

extern DCCPPNextionPage *nextionPages[MAX_PAGES];
extern NEXTION_DEVICE_TYPE nextionDeviceType;

class NextionTitlePage : public DCCPPNextionPage {
public:
  NextionTitlePage(Nextion &nextion) : DCCPPNextionPage(nextion, TITLE_PAGE, "0"),
    _versionText(nextion, TITLE_PAGE, 3, "Version"),
    _statusText {
      NextionText(nextion, TITLE_PAGE, 4, "Status1"), 
      NextionText(nextion, TITLE_PAGE, 5, "Status2"),
      NextionText(nextion, TITLE_PAGE, 6, "Status3"),
      NextionText(nextion, TITLE_PAGE, 7, "Status4"),
      NextionText(nextion, TITLE_PAGE, 8, "Status5")
     } {}
  void refreshPage() override {}
  void setStatusText(int line, String text) {
    _statusText[line].setText(text);
  }
  void clearStatusText() {
    for(auto line : _statusText) {
      line.setText("");
    }
  }
protected:
  void init() override {
    _versionText.setText(VERSION);
  }
  void displayPage() override {}
private:
  NextionText _versionText;
  NextionText _statusText[5];
};

class NextionAddressPage : public DCCPPNextionPage {
public:
  NextionAddressPage(Nextion &);
  void setCurrentAddress(uint32_t address) {
    _address = address;
  }
  void addNumber(const NextionButton *);
  void removeNumber(const NextionButton *);
  void changeTurnoutType(const NextionButton *);
  uint32_t getNewAddress() {
    return _newAddressString.toInt();
  }
  TurnoutType getTurnoutType() {
    return (TurnoutType)_turnoutType;
  }
  void refreshPage() override {}
protected:
  void init() override {}
  void displayPage() override;
private:
  void refreshTurnoutTypeButton();
  NextionButton _buttons[10];
  NextionButton _addressPic;
  NextionText _boardAddress;
  NextionText _indexAddress;
  NextionButton _turnoutTypeButton;
  NextionButton _saveButton;
  NextionButton _quitButton;
  NextionButton _undoButton;
  NextionText _currentAddress;
  NextionText _newAddress;
  uint32_t _address{0};
  uint8_t _turnoutType{TurnoutType::LEFT};
  String _newAddressString{""};
};

class NextionThrottlePage : public DCCPPNextionPage {
public:
  NextionThrottlePage(Nextion &);
  void activateLoco(const NextionButton *);
  void activateFunctionGroup(const NextionButton *);
  void setLocoDirection(bool direction);
  void toggleFunction(const NextionButton *);
  void changeLocoAddress(uint32_t);
  uint32_t getCurrentLocoAddress();
  void decreaseLocoSpeed();
  void increaseLocoSpeed();
  void setLocoSpeed(uint8_t speed);
  void refreshPage() override {
    refreshLocomotiveDetails();
  }
  void invalidateLocomotive(uint32_t);
protected:
  void init() override;
  void displayPage() override;
  void previousPageCallback(DCCPPNextionPage *);
private:
  void refreshLocomotiveDetails();
  void refreshFunctionButtons();
  void refreshLocomotiveDirection();
  uint8_t _activeLoco;
  uint8_t _activeFunctionGroup;
  uint32_t _locoNumbers[3];
  NextionButton _locoButtons[3];
  NextionButton _fgroupButtons[3];
  NextionButton _functionButtons[10];
  NextionButton _fwdButton;
  NextionButton _revButton;
  NextionButton _locoAddress;
  NextionButton _setup;
  NextionButton _accessories;
  NextionButton _downButton;
  NextionButton _upButton;
  NextionSlider _speedSlider;
  NextionNumber _speedNumber;
};

class NextionTurnoutPage : public DCCPPNextionPage {
public:
  NextionTurnoutPage(Nextion &);
  void toggleTurnout(const NextionButton *);
  
  virtual void refreshPage();
  void incrementTurnoutPage() {
    _turnoutStartIndex += getTurnoutsPerPageCount();
    refresh();
  }
  void decrementTurnoutPage() {
    _turnoutStartIndex -= getTurnoutsPerPageCount();
    if(_turnoutStartIndex < 0) {
      _turnoutStartIndex = 0;
    }
    refresh();
  }
  void addNewTurnout() {
    _pageMode = PAGE_MODE::ADDITION;
    auto addressPage = static_cast<NextionAddressPage *>(nextionPages[ADDRESS_PAGE]);
    addressPage->setCurrentAddress(0);
    addressPage->setPreviousPage(TURNOUT_PAGE);
    addressPage->display();
  }
  void deleteButtonHandler();
protected:
  void init() override {}
  void displayPage() override {
    refreshPage();
  }
  void previousPageCallback(DCCPPNextionPage *);
private:
  static constexpr int TURNOUTS_PER_PAGE_3_2_DISPLAY = 15;
  static constexpr int TURNOUTS_PER_PAGE_3_5_DISPLAY = 24;
  static constexpr int TURNOUTS_PER_PAGE_5_0_DISPLAY = 60;
  uint8_t getDefaultTurnoutPictureID(Turnout *);
  uint8_t getTurnoutsPerPageCount();
  NextionButton _turnoutButtons[TURNOUTS_PER_PAGE_5_0_DISPLAY];
  NextionButton _toAddress[TURNOUTS_PER_PAGE_5_0_DISPLAY];
  NextionButton _backButton;
  NextionButton _prevButton;
  NextionButton _nextButton;
  NextionButton _addButton;
  NextionButton _delButton;
  NextionButton _setupButton;
  enum PAGE_MODE {
    NORMAL, ADDITION, DELETION, EDIT
  };
  int16_t _turnoutStartIndex{0};
  PAGE_MODE _pageMode{PAGE_MODE::NORMAL};
};

enum TURNOUT_IMAGE_IDS {
  LEFT_HAND_CLOSED = 106,
  LEFT_HAND_THROWN = 107,
  RIGHT_HAND_CLOSED = 108,
  RIGHT_HAND_THROWN = 109,
  WYE_THROWN_LEFT = 110,
  WYE_THROWN_RIGHT = 111,
  MULTI_STRAIGHT = 112,
  MULTI_CROSS_OVER = 113,
  MULTI_LEFT_STRAIGHT = 114,
  MULTI_RIGHT_STRAIGHT = 115,
  MULTI_LEFT_TO_RIGHT = 116,
  MULTI_RIGHT_TO_LEFT = 117,
  THREEWAY_LEFT = 118,
  THREEWAY_STRAIGHT = 119,
  THREEWAY_RIGHT = 120,
  TURNOUT_DELETED = 121
};
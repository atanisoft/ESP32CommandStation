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

#pragma once

#include "Locomotive.h"
#include <HardwareSerial.h>
#include <Nextion.h>
#include <NextionPage.h>
#include <NextionButton.h>
#include <NextionText.h>
#include <NextionSlider.h>
#include <NextionNumber.h>

class NextionInterface {
public:
  static void init();
  static void update();
private:
  static bool _initializing;
  static uint64_t _startupTransitionTimer;
};

enum NEXTION_PAGES
{
  TITLE_PAGE = 0,
  ADDRESS_PAGE = 1,
  THROTTLE_PAGE = 2,
  TURNOUT_PAGE = 3,
  CONFIG_PAGE = 4,
  ROUTES_PAGE = 5,
  MAX_PAGES
};

class DCCPPNextionPage : public NextionPage {
public:
  DCCPPNextionPage(Nextion &nextion, uint8_t pageID, const char *pageName) :
    NextionPage(nextion, pageID, 0, pageName), _pageInitialized(false) {
  }
  void display() {
    String cmd = String("page ") + String(m_name);
    m_nextion.sendCommand((char *)cmd.c_str());
    if(!m_nextion.checkCommandComplete()) {
      log_e("switching to page %s was not successful.", m_name);
    }
    if(!_pageInitialized) {
      init();
      _pageInitialized = true;
    }
    displayPage();
  }
  virtual void refresh() = 0;
protected:
  // One time page initialization
  virtual void init() = 0;

  // Called anytime the page gets displayed, should be used for refresh of components etc.
  virtual void displayPage() = 0;
private:
  bool _pageInitialized;
};

class NextionTitlePage : public DCCPPNextionPage {
public:
  NextionTitlePage(Nextion &nextion) : DCCPPNextionPage(nextion, TITLE_PAGE, "0"),
    _versionText(nextion, TITLE_PAGE, 3, "Version") {}
  virtual void refresh() {}
protected:
  virtual void init() {
    _versionText.setText(VERSION);
  }
  virtual void displayPage() {};
private:
  NextionText _versionText;
};

class NextionAddressPage : public DCCPPNextionPage {
public:
  NextionAddressPage(Nextion &nextion);
  void setCurrentAddress(uint32_t address);
  void setReturnPage(uint8_t pageID) {
    _returnPageID = pageID;
  }
  uint8_t getReturnPage() {
    return _returnPageID;
  }
  void addNumber(const NextionButton *button);
  uint32_t getNewAddress() {
    return _newAddressString.toInt();
  }
  virtual void refresh() {}
protected:
  virtual void init() {};
  virtual void displayPage();
private:
  NextionButton _buttons[10];
  NextionButton _doneButton;
  NextionButton _abortButton;
  NextionText _currentAddress;
  NextionNumber _newAddress;
  uint32_t _address;
  uint8_t _returnPageID;
  String _newAddressString;
};

class NextionThrottlePage : public DCCPPNextionPage {
public:
  NextionThrottlePage(Nextion &nextion);
  void activateLoco(const NextionButton *button);
  void activateFunctionGroup(const NextionButton *button);
  void setLocoDirection(bool direction);
  void setTrackPower(bool on);
  void toggleFunction(const NextionButton *button);
  void changeLocoAddress(uint32_t newAddress);
  uint32_t getCurrentLocoAddress();
  void decreaseLocoSpeed();
  void increaseLocoSpeed();
  void setLocoSpeed(uint8_t speed);
  virtual void refresh() {
    refreshLocomotiveDetails();
  }
  void invalidateLocomotive(uint32_t address);
protected:
  virtual void init();
  virtual void displayPage();
private:
  void refreshLocomotiveDetails();
  void refreshFunctionButtons();
  void refreshLocomotiveDirection();
  void refreshPowerButtons();
  uint8_t _activeLoco;
  uint8_t _activeFunctionGroup;
  uint32_t _locoNumbers[3];
  Locomotive *_locomotives[3];
  NextionButton _locoButtons[3];
  NextionButton _fgroupButtons[3];
  NextionButton _fwdButton;
  NextionButton _revButton;
  NextionButton _onButton;
  NextionButton _offButton;
  NextionButton _stopButton;
  NextionButton _functionButtons[10];
  NextionButton _locoAddress;
  NextionButton _routes;
  NextionButton _accessories;
  NextionButton _downButton;
  NextionButton _upButton;
  NextionSlider _speedSlider;
  NextionNumber _speedNumber;
};

extern DCCPPNextionPage *nextionPages[MAX_PAGES];

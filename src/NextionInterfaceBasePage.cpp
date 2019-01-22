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

constexpr uint8_t ON_PIC_OFF=54;
constexpr uint8_t ON_PIC_ON=55;
constexpr uint8_t OFF_PIC_OFF=56;
constexpr uint8_t OFF_PIC_ON=57;

DCCPPNextionPage::DCCPPNextionPage(Nextion &nextion, const uint8_t pageID, const String &pageName) :
  NextionPage(nextion, pageID, 0, pageName),
  _onButton(nextion, pageID, 1, "On"),
  _offButton(nextion, pageID, 2, "Off"),
  _stopButton(nextion, pageID, 3, "Stop"),
  _pageInitialized(false),
  _returnPageID(0) {
  _stopButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->sendEStop();
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
  if(!show()) {
    printf("display of page %s was not successful.\n", m_name.c_str());
  } else {
    printf("displayed page %s\n", m_name.c_str());
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

void DCCPPNextionPage::sendEStop() {
  LocomotiveManager::emergencyStop();
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
    _onButton.setPictureID(ON_PIC_ON);
    _offButton.setPictureID(OFF_PIC_OFF);
  } else {
    _onButton.setPictureID(ON_PIC_OFF);
    _offButton.setPictureID(OFF_PIC_ON);
  }
}
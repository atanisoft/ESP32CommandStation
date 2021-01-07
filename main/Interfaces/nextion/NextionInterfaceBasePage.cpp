/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2021 NormHal, Mike Dunston

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

#include "ESP32CommandStation.h"

#include <DCCSignalVFS.h>

#if CONFIG_NEXTION

constexpr uint8_t ON_PIC_OFF=54;
constexpr uint8_t ON_PIC_ON=55;
constexpr uint8_t OFF_PIC_OFF=56;
constexpr uint8_t OFF_PIC_ON=57;

BaseNextionPage::BaseNextionPage(Nextion &nextion, const uint8_t pageID, const std::string &pageName) :
  NextionPage(nextion, pageID, 0, pageName.c_str()),
  _onButton(nextion, pageID, 1, "On"),
  _stopButton(nextion, pageID, 2, "Stop"),
  _offButton(nextion, pageID, 3, "Off") {
  _stopButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->sendEStop();
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

void BaseNextionPage::display() {
  if(!show()) {
    LOG_ERROR("[Nextion] Display of page %s was not successful.", m_name.c_str());
  } else {
    LOG(VERBOSE, "[Nextion] Displayed page %s", m_name.c_str());
  }
  if(!_pageInitialized) {
    init();
    _pageInitialized = true;
  }
  refreshPowerButtons();
  displayPage();
}

void BaseNextionPage::refresh() {
  refreshPowerButtons();
  refreshPage();
}

void BaseNextionPage::setTrackPower(bool on) {
  if (on) {
    enable_ops_track_output();
  } else {
    disable_track_outputs();
  }
  refreshPowerButtons();
}

void BaseNextionPage::sendEStop() {
  initiate_estop();
  refresh();
}

void BaseNextionPage::displayPreviousPage(bool invokeCallback) {
  if(_returnPageID > 0) {
    nextionPages[_returnPageID]->display();
    if(invokeCallback) {
      nextionPages[_returnPageID]->previousPageCallback(this);
    }
  }
}

void BaseNextionPage::refreshPowerButtons() {
  if(is_ops_track_output_enabled()) {
    _onButton.setPictureID(ON_PIC_ON);
    _offButton.setPictureID(OFF_PIC_OFF);
  } else {
    _onButton.setPictureID(ON_PIC_OFF);
    _offButton.setPictureID(OFF_PIC_ON);
  }
}

#endif // CONFIG_NEXTION
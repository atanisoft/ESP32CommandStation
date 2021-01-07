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
#include <LCCWiFiManager.h>

#if CONFIG_NEXTION

//
/************************************************************************************************************/
// Setup Page
/************************************************************************************************************/
//
NextionSetupPage::NextionSetupPage(Nextion &nextion) : 
  BaseNextionPage(nextion, SETUP_PAGE, "4"),
  _saveButton(nextion, SETUP_PAGE, 4, "Save"),
  _quitButton(nextion, SETUP_PAGE, 6, "Quit"),
  _undoButton(nextion, SETUP_PAGE, 5, "Undo"),
  _routesButton(nextion, SETUP_PAGE, 12, "Routes"),
  _versionText(nextion, SETUP_PAGE, 7, "Version"),
  _ipAddrText(nextion, SETUP_PAGE, 10, "IPAddr"),
  _ssidText(nextion, SETUP_PAGE, 12, "SSID")
{
    
  _saveButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
  _quitButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
      nextionPages[widget->getPageID()]->returnToPreviousPage();
    }
  });
  _undoButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
        printf("Undo Button Pressed\n");
    }
  });
  _routesButton.attachCallback([](NextionEventType type, INextionTouchable *widget) {
    if(type == NEX_EVENT_PUSH) {
        printf("Routes Button Pressed\n");
    }
  });
}

void NextionSetupPage::displayPage()
{
  _versionText.setText(esp_ota_get_app_description()->version);
  _ssidText.setText(Singleton<LCCWiFiManager>::instance()->get_ssid());
  tcpip_adapter_ip_info_t ip;
  tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip);
  _ipAddrText.setText(ipv4_to_string(ntohl(ip.ip.addr)));
}

#endif // CONFIG_NEXTION
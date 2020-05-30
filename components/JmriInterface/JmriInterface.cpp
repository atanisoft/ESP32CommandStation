/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

#include "sdkconfig.h"

#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <Httpd.h>
#include <memory>
#include <utils/socket_listener.hxx>
#include "JmriClientFlow.h"

std::unique_ptr<SocketListener> listener;

void init_jmri_interface()
{
  Singleton<Esp32WiFiManager>::instance()->register_network_up_callback(
  [&](esp_interface_t interface, uint32_t ip)
  {
    if (!listener)
    {
      LOG(INFO, "[JMRI] Starting JMRI listener");
      listener.reset(
        new SocketListener(CONFIG_JMRI_LISTENER_PORT,
        [](int fd)
        {
        sockaddr_in source;
        socklen_t source_len = sizeof(sockaddr_in);
        bzero(&source, sizeof(sockaddr_in));
        getpeername(fd, (sockaddr *)&source, &source_len);
        // Create new JMRI client and attach it to the Httpd
        // instance rather than the default executor.
        new JmriClientFlow(fd, ntohl(source.sin_addr.s_addr)
                            , Singleton<http::Httpd>::instance());
        }, "jmri"));
      Singleton<Esp32WiFiManager>::instance()->mdns_publish(
        CONFIG_JMRI_MDNS_SERVICE_NAME, CONFIG_JMRI_LISTENER_PORT);
    }
  });
  Singleton<Esp32WiFiManager>::instance()->register_network_down_callback(
  [&](esp_interface_t interface)
  {
    LOG(INFO, "[WiFi] Shutting down JMRI listener");
    listener.reset(nullptr);
    Singleton<Esp32WiFiManager>::instance()->mdns_unpublish(
        CONFIG_JMRI_MDNS_SERVICE_NAME);
  });
}
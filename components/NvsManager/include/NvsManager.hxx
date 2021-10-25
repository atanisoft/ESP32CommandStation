/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

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

#ifndef NVS_MANAGER_H_
#define NVS_MANAGER_H_

#include "sdkconfig.h"
#include <esp_wifi_types.h>
#include <string>
#include <utils/Singleton.hxx>

namespace openlcb
{
  class BroadcastTimeServer;
  class Node;
  class SimpleStackBase;
}
namespace openmrn_arduino
{
  class Esp32WiFiManager;
}
namespace esp32cs
{

class NvsManager : public Singleton<NvsManager>
{
public:
  void init(uint8_t reset_reason);
  bool should_reset_config();
  bool should_reset_events();
  bool start_stack();
  void force_factory_reset();
  void force_reset_events();
  void set_led_brightness(uint8_t level);
  void restart_fast_clock();
  void save_fast_clock_time();

  bool memory_spaces_modified();
  void register_virtual_memory_spaces(openlcb::SimpleStackBase *stack);
  void register_clocks(openlcb::Node *node, openmrn_arduino::Esp32WiFiManager *wifi_mgr);
  
  uint64_t node_id();
  void node_id(uint64_t node_id);
  wifi_mode_t wifi_mode();
  void wifi_mode(wifi_mode_t mode);
  const char *station_ssid();
  const char *station_password();
  const char *softap_ssid();
  const char *softap_password();
  uint8_t softap_channel();
  wifi_auth_mode_t softap_auth();
  const char *hostname_prefix();
  bool sntp_enabled();
  const char *sntp_server();
  const char *timezone();
};

} // namespace esp32cs

#endif // NVS_MANAGER_H_
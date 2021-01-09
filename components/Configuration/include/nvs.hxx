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

#ifndef NVS_HXX_
#define NVS_HXX_

#include <esp_err.h>
#include <esp_wifi_types.h>
#include <stdint.h>

typedef struct
{
    bool force_reset;
    bool bootloader_req;
    uint64_t node_id;
    uint8_t status_led_brightness;
    bool reset_events;
    uint8_t reserved[20];
} node_config_t;

esp_err_t load_config(node_config_t *config);
esp_err_t save_config(node_config_t *config);
esp_err_t default_config(node_config_t *config);
void nvs_init();
void dump_config(node_config_t *config);
bool force_factory_reset();
bool set_node_id(uint64_t node_id);
bool update_status_led_brightness(uint8_t value);

#endif // NVS_CONFIG_HXX_
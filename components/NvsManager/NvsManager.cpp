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

#include "NvsManager.hxx"
#include "sdkconfig.h"
#include "hardware.hxx"
#include "StringUtils.hxx"
#include <esp32/rom/rtc.h>
#include <esp_err.h>
#include <esp_partition.h>
#include <freertos_drivers/esp32/Esp32BootloaderHal.hxx>
#include <nvs.h>
#include <nvs_flash.h>
#include <StatusLED.h>
using std::string;
#include <utils/format_utils.hxx>
#include <utils/logging.h>

#ifndef CONFIG_WIFI_STATION_SSID
#define CONFIG_WIFI_STATION_SSID ""
#endif

#ifndef CONFIG_WIFI_STATION_PASSWORD
#define CONFIG_WIFI_STATION_PASSWORD ""
#endif

#ifndef CONFIG_WIFI_SOFTAP_SSID
#define CONFIG_WIFI_SOFTAP_SSID "esp32csap"
#endif

#ifndef CONFIG_WIFI_SOFTAP_PASSWORD
#define CONFIG_WIFI_SOFTAP_PASSWORD "esp32csap"
#endif

#ifndef CONFIG_WIFI_SOFTAP_AUTH
#define CONFIG_WIFI_SOFTAP_AUTH WIFI_AUTH_OPEN
#endif

#ifndef CONFIG_WIFI_HOSTNAME_PREFIX
#define CONFIG_WIFI_HOSTNAME_PREFIX "esp32cs_"
#endif

#ifndef CONFIG_WIFI_SOFTAP_CHANNEL
#define CONFIG_WIFI_SOFTAP_CHANNEL 1
#endif

#ifndef CONFIG_SNTP_SERVER
#define CONFIG_SNTP_SERVER "pool.ntp.org"
#endif

#ifndef CONFIG_SNTP_TIMEZONE
#define CONFIG_SNTP_TIMEZONE "UTC0"
#endif

namespace esp32cs
{
  typedef struct
  {
    uint64_t node_id;
    wifi_mode_t wifi_mode;
    char hostname_prefix[16];
    char station_ssid[33];
    char station_pass[33];
    char softap_ssid[33];
    char softap_pass[33];
    wifi_auth_mode_t softap_auth;
    uint8_t softap_channel;
    char sntp_server[33];
    char timezone[33];
    bool sntp_enabled;
    uint8_t led_brightness;
    uint8_t reserved[28];
  } node_config_t;

  static node_config_t nvsConfig;

  /// NVS Persistence namespace.
  static constexpr char NVS_NAMESPACE[] = "node";

  /// NVS Persistence key.
  static constexpr char NVS_CFG_KEY[] = "cfg";

  ///
  static constexpr uint32_t RTC_BOOL_TRUE = 1;
  static constexpr uint32_t RTC_BOOL_FALSE = 0;

  /// Variable used to indicate that the persistent configuration should be
  /// discarded and recreated from compile time configuration data.
  static uint32_t RTC_NOINIT_ATTR reset_configuration;

  /// Variable used to indicate that persistent event IDs should be regenerated
  /// prior to startup of the OpenMRN stack.
  static uint32_t RTC_NOINIT_ATTR reset_events;

  /// Variable used to indicate that the startup should go into the bootloader
  /// rather than default startup mode.
  static uint32_t RTC_NOINIT_ATTR bootloader_request;

  /// Number of seconds to hold the Factory Reset button to force clear all
  /// stored configuration data.
  static constexpr int8_t FACTORY_RESET_HOLD_TIME = 10;

  /// Number of seconds to hold the Factory Reset button to force regeneration of
  /// all Event IDs. NOTE: This will *NOT* clear WiFi configuration data.
  static constexpr int8_t FACTORY_RESET_EVENTS_HOLD_TIME = 5;

  static constexpr const char *const WIFI_MODES[] =
      {
          "Disabled",
          "Station",
          "SoftAP",
          "Station and SoftAP"};
  static constexpr const char *const WIFI_AUTH_MODES[] =
      {
          "open",
          "WEP",
          "WPA_PSK",
          "WPA2_PSK",
          "WPA_WPA2_PSK",
          "Unsupported",
          "WPA3_PSK",
          "WPA2_WPA3_PSK",
          "Unsupported"};

  static inline void persist_configuration()
  {
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(
        nvs_set_blob(nvs, NVS_CFG_KEY, &nvsConfig, sizeof(node_config_t)));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);
  }

  static inline void reset_nvs_config_to_defaults()
  {
    memset(&nvsConfig, 0, sizeof(node_config_t));
    nvsConfig.node_id = CONFIG_OLCB_NODE_ID;
    nvsConfig.wifi_mode = (wifi_mode_t)CONFIG_WIFI_MODE;
    str_populate(nvsConfig.hostname_prefix, CONFIG_WIFI_HOSTNAME_PREFIX);
    str_populate(nvsConfig.station_ssid, CONFIG_WIFI_STATION_SSID);
    str_populate(nvsConfig.station_pass, CONFIG_WIFI_STATION_PASSWORD);
    str_populate(nvsConfig.softap_ssid, CONFIG_WIFI_SOFTAP_SSID);
    str_populate(nvsConfig.softap_pass, CONFIG_WIFI_SOFTAP_PASSWORD);
    nvsConfig.softap_auth = (wifi_auth_mode_t)CONFIG_WIFI_SOFTAP_AUTH;
#if defined(CONFIG_SNTP)
    nvsConfig.sntp_enabled = true;
    str_populate(nvsConfig.sntp_server, CONFIG_SNTP_SERVER);
    str_populate(nvsConfig.timezone, CONFIG_SNTP_TIMEZONE);
#endif
  }

  static inline void display_nvs_configuration()
  {
    // display current configuration settings.
    LOG(INFO, "[NVS] Node ID: %s",
        node_id_to_string(nvsConfig.node_id).c_str());
    LOG(INFO, "[NVS] WiFi configuration:");
    LOG(INFO, "Mode: %s (%d)", WIFI_MODES[nvsConfig.wifi_mode],
        nvsConfig.wifi_mode);
    LOG(INFO, "Hostname Prefix: %s", nvsConfig.hostname_prefix);
    if (nvsConfig.wifi_mode == WIFI_MODE_STA ||
        nvsConfig.wifi_mode == WIFI_MODE_APSTA)
    {
      LOG(INFO, "Station SSID: %s", nvsConfig.station_ssid);
    }
    if (nvsConfig.wifi_mode == WIFI_MODE_AP ||
        nvsConfig.wifi_mode == WIFI_MODE_APSTA)
    {
      LOG(INFO, "SoftAP SSID: %s", nvsConfig.softap_ssid);
      LOG(INFO, "SoftAP Auth: %s (%d)", WIFI_AUTH_MODES[nvsConfig.softap_auth],
          nvsConfig.softap_auth);
    }
    if (nvsConfig.sntp_enabled)
    {
      LOG(INFO, "SNTP: %s / %s", nvsConfig.sntp_server, nvsConfig.timezone);
    }
    else
    {
      LOG(INFO, "SNTP: Off");
    }
  }

  void NvsManager::init(uint8_t reset_reason)
  {
    LOG(INFO, "NVS.init(%d)", reset_reason);
    // If this is the first power up of the node we need to reset the flag
    // since it will not be initialized automatically.
    if (reset_reason == POWERON_RESET)
    {
      LOG(INFO, "[NVS] Initializing RTC vars");
      reset_configuration = RTC_BOOL_FALSE;
      reset_events = RTC_BOOL_FALSE;
      bootloader_request = RTC_BOOL_FALSE;
    }
    auto leds = Singleton<StatusLED>::instance();
    leds->hw_init();

    nvs_handle_t nvs;
    size_t config_size = sizeof(node_config_t);

    // Initialize NVS before we do any other initialization as it may be
    // internally used by various components even if we disable it's usage in
    // the WiFi connection stack.
    LOG(INFO, "[NVS] Initializing NVS");
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init()) == ESP_ERR_NVS_NO_FREE_PAGES)
    {
      const esp_partition_t *partition =
          esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                   ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
      if (partition != NULL)
      {
        LOG(INFO, "[NVS] Erasing partition %s...", partition->label);
        ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));
        ESP_ERROR_CHECK(nvs_flash_init());
      }
    }

    // Check for factory reset button being held to GND and the BOOTLOADER
    // button not being held to GND. If this is detected the factory reset
    // process will be started.
    if (FACTORY_RESET_BUTTON_Pin::instance()->is_clr() &&
        BOOTLOADER_BUTTON_Pin::instance()->is_set())
    {
      // Set the LEDs in an on/off/on pattern for the blink
      leds->set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::GREEN);
      leds->set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::OFF);
      leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::GREEN);
      leds->set(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::OFF);
      leds->set(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN);
      leds->refresh();

      // Count down from the overall factory reset time.
      uint8_t hold_time = FACTORY_RESET_HOLD_TIME;
      for (; hold_time > 0 && FACTORY_RESET_BUTTON_Pin::instance()->is_clr();
           hold_time--)
      {
        if (hold_time > FACTORY_RESET_EVENTS_HOLD_TIME)
        {
          LOG(WARNING,
              "Event ID reset in %d seconds, factory reset in %d seconds.",
              hold_time - FACTORY_RESET_EVENTS_HOLD_TIME, hold_time);
          leds->set(StatusLED::LED::WIFI_STA,
                    (hold_time % 2) ? StatusLED::COLOR::GREEN : StatusLED::COLOR::OFF);
          leds->set(StatusLED::LED::OPS_TRACK,
                    (hold_time % 2) ? StatusLED::COLOR::OFF : StatusLED::COLOR::GREEN);
          leds->set(StatusLED::LED::PROG_TRACK,
                    (hold_time % 2) ? StatusLED::COLOR::OFF : StatusLED::COLOR::GREEN);
        }
        else
        {
          leds->set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::OFF);
          leds->set(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::OFF);
          leds->set(StatusLED::LED::PROG_TRACK,  StatusLED::COLOR::OFF);
          LOG(WARNING, "Factory reset in %d seconds.", hold_time);
        }
        leds->set(StatusLED::LED::BOOTLOADER,
                  (hold_time % 2) ? StatusLED::COLOR::OFF : StatusLED::COLOR::GREEN);
        leds->refresh();
        usleep(SEC_TO_USEC(1));
      }
      if (FACTORY_RESET_BUTTON_Pin::instance()->is_clr() && hold_time == 0)
      {
        // if the button is still being held and the hold time expired
        // start a full factory reset.
        LOG(WARNING, "Factory reset triggered!");
        reset_configuration = RTC_BOOL_TRUE;
      }
      else if (hold_time <= FACTORY_RESET_EVENTS_HOLD_TIME)
      {
        // if the button is not being held and the hold time is less than
        // the event id reset count down trigger a reset of events.
        LOG(WARNING, "Reset of events triggered!");
        reset_events = RTC_BOOL_TRUE;
      }
      else
      {
        // The button was released prior to the event id reset limit, do
        // nothing.
        LOG(WARNING, "Factory reset aborted!");
      }
    }
    else if (BOOTLOADER_BUTTON_Pin::instance()->is_clr())
    {
      LOG(INFO, "[NVS] Bootloader requested");
      // If both the factory reset and user button are held to GND it is a
      // request to enter the bootloader mode.
      bootloader_request = RTC_BOOL_TRUE;

      // give a visual indicator that the bootloader request has been ACK'd
      // turn on both Bootloader and Activity LEDs, wait ~1sec, turn off
      // Bootloader LED, wait ~1sec, turn off Activity LED.
      leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::RED);
      leds->refresh();
      usleep(SEC_TO_USEC(1));
      leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::OFF);
      leds->refresh();
    }

    LOG(INFO, "[NVS] Loading configuration");
    // load non-CDI based config from NVS
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_get_blob(nvs, NVS_CFG_KEY, &nvsConfig, &config_size));
    if (config_size != sizeof(node_config_t) || res != ESP_OK)
    {
      LOG_ERROR("[NVS] Configuration missing or corrupt, using defaults");
      reset_nvs_config_to_defaults();
      ESP_ERROR_CHECK(
          nvs_set_blob(nvs, NVS_CFG_KEY, &nvsConfig, sizeof(node_config_t)));
      ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
    display_nvs_configuration();
  }

  bool NvsManager::should_reset_config()
  {
    return reset_configuration == RTC_BOOL_TRUE;
  }

  bool NvsManager::should_reset_events()
  {
    return reset_events == RTC_BOOL_TRUE;
  }

  void NvsManager::force_factory_reset()
  {
    reset_configuration = RTC_BOOL_TRUE;
    esp_restart();
  }

  void NvsManager::force_reset_events()
  {
    reset_events = RTC_BOOL_TRUE;
    esp_restart();
  }

  void NvsManager::set_led_brightness(uint8_t level)
  {
    nvsConfig.led_brightness = level;
    persist_configuration();
  }

  bool NvsManager::start_stack()
  {
    if (bootloader_request == esp32cs::RTC_BOOL_FALSE)
    {
      LOG(INFO, "[NVS] Bootloader flag is false, start the stack");
      return true;
    }
#if CONFIG_OLCB_TWAI_ENABLED
    LOG(INFO, "[NVS] Bootloader flag is true, start the bootloader");
    // automatic reset is diabled so we can reset the RTC persistent memory
    // flag used to indicate bootloader requested.
    esp32_bootloader_run(nvsConfig.node_id,
                         (gpio_num_t)CONFIG_OLCB_TWAI_RX_PIN,
                         (gpio_num_t)CONFIG_OLCB_TWAI_TX_PIN, false);
#else
    LOG_ERROR("[NVS] Bootloader requested but TWAI is not enabled!\n"
              "Disabling flag and restarting!");
#endif // CONFIG_OLCB_TWAI_ENABLED
    bootloader_request = esp32cs::RTC_BOOL_FALSE;
    esp_restart();
    return false;
  }

  uint64_t NvsManager::node_id()
  {
    return nvsConfig.node_id;
  }

  void NvsManager::node_id(uint64_t node_id)
  {
    nvsConfig.node_id = node_id;
    persist_configuration();
    force_factory_reset();
  }

  wifi_mode_t NvsManager::wifi_mode()
  {
    return nvsConfig.wifi_mode;
  }

  const char *NvsManager::station_ssid()
  {
    return nvsConfig.station_ssid;
  }

  const char *NvsManager::station_password()
  {
    return nvsConfig.station_pass;
  }

  void NvsManager::reconfigure_station(std::string ssid, std::string password)
  {
    str_populate(nvsConfig.station_ssid, ssid.c_str());
    str_populate(nvsConfig.station_pass, password.c_str());
    persist_configuration();
  }

  const char *NvsManager::softap_ssid()
  {
    return nvsConfig.softap_ssid;
  }

  const char *NvsManager::softap_password()
  {
    return nvsConfig.softap_pass;
  }

  uint8_t NvsManager::softap_channel()
  {
    return nvsConfig.softap_channel;
  }

  wifi_auth_mode_t NvsManager::softap_auth()
  {
    return nvsConfig.softap_auth;
  }

  void NvsManager::reconfigure_softap(std::string ssid, std::string password, uint8_t channel, wifi_auth_mode_t auth)
  {
    str_populate(nvsConfig.softap_ssid, ssid.c_str());
    str_populate(nvsConfig.softap_pass, password.c_str());
    nvsConfig.softap_channel = channel;
    nvsConfig.softap_auth = auth;
    persist_configuration();
  }

  const char *NvsManager::hostname_prefix()
  {
    return nvsConfig.hostname_prefix;
  }

  bool NvsManager::sntp_enabled()
  {
    return nvsConfig.sntp_enabled;
  }

  const char *NvsManager::sntp_server()
  {
    return nvsConfig.sntp_server;
  }

  const char *NvsManager::timezone()
  {
    return nvsConfig.timezone;
  }

  void NvsManager::reconfigure_sntp(bool enabled, std::string server, std::string timezone)
  {
    nvsConfig.sntp_enabled = enabled;
    str_populate(nvsConfig.sntp_server, server.c_str());
    str_populate(nvsConfig.timezone, timezone.c_str());
    persist_configuration();
  }

} // namespace esp32cs

extern "C"
{

/// Updates the state of a status LED.
///
/// @param led is the LED to update.
/// @param value is the new state of the LED.
///
/// NOTE: Currently the following mapping is being used for the LEDs:
/// LED_ACTIVE -> Bootloader LED
/// LED_REQUEST -> Used only as a hook for printing bootloader startup.
void bootloader_led(enum BootloaderLed led, bool value)
{
    LOG(VERBOSE, "[Bootloader] bootloader_led(%d, %d)", led, value);
    if (led == LED_ACTIVE)
    {
      auto leds = Singleton<esp32cs::StatusLED>::instance();
      leds->set(esp32cs::StatusLED::LED::BOOTLOADER,
               value ? esp32cs::StatusLED::COLOR::GREEN : esp32cs::StatusLED::COLOR::OFF);
      leds->refresh();
    }
    else if (led == LED_REQUEST)
    {
      LOG(INFO, "[Bootloader] Preparing to receive firmware");
      LOG(INFO, "[Bootloader] Current partition: %s", current->label);
      LOG(INFO, "[Bootloader] Target partition: %s", target->label);
    }
}
/// Initializes the node specific bootloader hardware (LEDs)
void bootloader_hw_set_to_safe(void)
{
  LOG(VERBOSE, "[Bootloader] bootloader_hw_set_to_safe");
}

/// Requests that the node enters the OpenLCB Bootloader mode.
void enter_bootloader()
{
  // set global flag that we need to enter the bootloader
  esp32cs::bootloader_request = esp32cs::RTC_BOOL_TRUE;
  LOG(INFO, "[Bootloader] Rebooting into bootloader");
  // reboot so we can enter the bootloader
  esp_restart();
}

/// Verifies that the bootloader has been requested.
///
/// @return true if bootloader_request is set to one, otherwise false.
bool request_bootloader(void)
{
  LOG(VERBOSE, "[Bootloader] request_bootloader");
  return esp32cs::bootloader_request == esp32cs::RTC_BOOL_TRUE;
}

}
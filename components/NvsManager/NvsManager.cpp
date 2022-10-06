/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "NvsManager.hxx"
#include "NodeIdMemoryConfigSpace.hxx"
#include "FastClockMemoryConfigSpace.hxx"
#include "RealTimeClockMemoryConfigSpace.hxx"
#include "WiFiMemoryConfigSpace.hxx"
#include "sdkconfig.h"
#include "hardware.hxx"
#if CONFIG_IDF_TARGET_ESP32
#include <esp32/rom/rtc.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/rom/rtc.h>
#endif
#include <esp_err.h>
#include <esp_partition.h>
#include <esp_wifi_types.h>
#include <freertos_drivers/esp32/Esp32BootloaderHal.hxx>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <nvs.h>
#include <nvs_flash.h>
#include <openlcb/BroadcastTimeServer.hxx>
#include <openlcb/SimpleStack.hxx>
#include <StatusLED.hxx>
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

#ifndef CONFIG_FASTCLOCK_RATE
#define CONFIG_FASTCLOCK_RATE 4
#endif

#ifndef CONFIG_FASTCLOCK_START_YEAR
#define CONFIG_FASTCLOCK_START_YEAR 1900
#endif

#ifndef CONFIG_FASTCLOCK_START_MONTH
#define CONFIG_FASTCLOCK_START_MONTH 1
#endif

#ifndef CONFIG_FASTCLOCK_START_DAY
#define CONFIG_FASTCLOCK_START_DAY 1
#endif

#ifndef CONFIG_FASTCLOCK_START_HOUR
#define CONFIG_FASTCLOCK_START_HOUR 1
#endif

#ifndef CONFIG_FASTCLOCK_START_MINUTE
#define CONFIG_FASTCLOCK_START_MINUTE 1
#endif

#ifndef CONFIG_FASTCLOCK_REALTIME_ID
#define CONFIG_FASTCLOCK_REALTIME_ID openlcb::BroadcastTimeDefs::DEFAULT_REALTIME_CLOCK_ID
#endif

#ifndef CONFIG_FASTCLOCK_DEFAULT_ID
#define CONFIG_FASTCLOCK_DEFAULT_ID openlcb::BroadcastTimeDefs::DEFAULT_FAST_CLOCK_ID
#endif

static_assert(CONFIG_FASTCLOCK_RATE >= -2048 &&
              CONFIG_FASTCLOCK_RATE <= 2048);
static_assert(CONFIG_FASTCLOCK_START_YEAR >= 0 &&
              CONFIG_FASTCLOCK_START_YEAR <= 4095);
static_assert(CONFIG_FASTCLOCK_START_MONTH >= 1 &&
              CONFIG_FASTCLOCK_START_MONTH <= 12);
static_assert(CONFIG_FASTCLOCK_START_DAY >= 1 &&
              CONFIG_FASTCLOCK_START_DAY <= 31);
static_assert(CONFIG_FASTCLOCK_START_HOUR >= 0 &&
              CONFIG_FASTCLOCK_START_HOUR <= 23);
static_assert(CONFIG_FASTCLOCK_START_MINUTE >= 0 &&
              CONFIG_FASTCLOCK_START_MINUTE <= 59);

namespace esp32cs
{

  static node_config_t nvsConfig;

  static uninitialized<openlcb::BroadcastTimeServer> fastclock;
  static uninitialized<openlcb::BroadcastTimeServer> realtime_clock;
  static uninitialized<WiFiMemoryConfigSpace> wifi_memory_space;
  static uninitialized<FastClockMemoryConfigSpace> fastclock_memory_space;
  static uninitialized<RealTimeClockMemoryConfigSpace> realtimeclock_memory_space;
  static uninitialized<NodeIdMemoryConfigSpace> node_id_memoryspace;

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
#if CONFIG_SNTP
    nvsConfig.sntp_enabled = true;
#endif // CONFIG_SNTP
    str_populate(nvsConfig.sntp_server, CONFIG_SNTP_SERVER);
    str_populate(nvsConfig.timezone, CONFIG_SNTP_TIMEZONE);
    nvsConfig.fastclock_id = CONFIG_FASTCLOCK_DEFAULT_ID;
    nvsConfig.fastclock_rate = CONFIG_FASTCLOCK_RATE;
    nvsConfig.fastclock_year = CONFIG_FASTCLOCK_START_YEAR - 1900;
    nvsConfig.fastclock_month = CONFIG_FASTCLOCK_START_MONTH;
    nvsConfig.fastclock_day = CONFIG_FASTCLOCK_START_DAY;
    nvsConfig.fastclock_hour = CONFIG_FASTCLOCK_START_HOUR - 1;
    nvsConfig.fastclock_minute = CONFIG_FASTCLOCK_START_MINUTE - 1;
#if CONFIG_FASTCLOCK
    nvsConfig.fastclock_enabled = true;
#endif
    nvsConfig.realtimeclock_id = CONFIG_FASTCLOCK_REALTIME_ID;
#if FASTCLOCK_REALTIME
    nvsConfig.realtimeclock_enabled = true;
#endif
  }

  static inline void display_nvs_configuration()
  {
    // display current configuration settings.
    LOG(INFO, "[NVS] Node ID: %s",
        utils::node_id_to_string(nvsConfig.node_id).c_str());
    LOG(INFO, "[NVS] WiFi Mode: %s (%d)", WIFI_MODES[nvsConfig.wifi_mode],
        nvsConfig.wifi_mode);
    LOG(INFO, "[NVS] Hostname Prefix: %s", nvsConfig.hostname_prefix);
    if (nvsConfig.wifi_mode == WIFI_MODE_STA ||
        nvsConfig.wifi_mode == WIFI_MODE_APSTA)
    {
      LOG(INFO, "[NVS] Station SSID: %s", nvsConfig.station_ssid);
    }
    if (nvsConfig.wifi_mode == WIFI_MODE_AP ||
        nvsConfig.wifi_mode == WIFI_MODE_APSTA)
    {
      LOG(INFO, "[NVS] SoftAP SSID: %s", nvsConfig.softap_ssid);
      LOG(INFO, "[NVS] SoftAP Auth: %s (%d)", WIFI_AUTH_MODES[nvsConfig.softap_auth],
          nvsConfig.softap_auth);
    }
    if (nvsConfig.sntp_enabled)
    {
      LOG(INFO, "[NVS] SNTP: %s / %s", nvsConfig.sntp_server, nvsConfig.timezone);
    }
    else
    {
      LOG(INFO, "[NVS] SNTP: Off");
    }
    if (nvsConfig.fastclock_enabled)
    {
      LOG(INFO, "[NVS] FastClock(%s): %04d-%02d-%02d %02d:%02d",
          utils::node_id_to_string(nvsConfig.fastclock_id).c_str(),
          nvsConfig.fastclock_year + 1900, nvsConfig.fastclock_month,
          nvsConfig.fastclock_day, nvsConfig.fastclock_hour,
          nvsConfig.fastclock_minute);
    }
    else
    {
      LOG(INFO, "[NVS] FastClock: Disabled");
    }
#if CONFIG_STATUS_LED_DATA_PIN != -1
    LOG(INFO, "[NVS] LED brightness: %d", nvsConfig.led_brightness);
#endif // CONFIG_STATUS_LED_DATA_PIN != -1
  }

  void NvsManager::init(uint8_t reset_reason)
  {
    LOG(VERBOSE, "NVS.init(%d)", reset_reason);
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
      leds->set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::GREEN_BLINK, true);
      leds->set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::GREEN_BLINK);
      leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::GREEN_BLINK,
                true);
      leds->set(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN_BLINK);
      leds->set(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN_BLINK,
                true);

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
        }
        else
        {
          leds->set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::BLUE_BLINK,
                    true);
          leds->set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::BLUE_BLINK);
          leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::BLUE_BLINK,
                    true);
          leds->set(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::BLUE_BLINK);
          leds->set(StatusLED::LED::PROG_TRACK,  StatusLED::COLOR::BLUE_BLINK,
                    true);
          LOG(WARNING, "Factory reset in %d seconds.", hold_time);
        }
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
      vTaskDelay(pdMS_TO_TICKS(1000));
      leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::OFF);
    }

    LOG(INFO, "[NVS] Loading configuration");
    // load non-CDI based config from NVS
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_get_blob(nvs, NVS_CFG_KEY, &nvsConfig, &config_size));
    bool need_persist = false;
    if (config_size != sizeof(node_config_t) || res != ESP_OK)
    {
      LOG_ERROR("[NVS] Configuration missing or corrupt, using defaults");
      reset_nvs_config_to_defaults();
      need_persist = true;
    }
    else
    {
      // validate fast clock settings are valid
      if (nvsConfig.fastclock_month < 1 || nvsConfig.fastclock_month > 12)
      {
        nvsConfig.fastclock_month = CONFIG_FASTCLOCK_START_MONTH;
        need_persist = true;
      }
      if (nvsConfig.fastclock_day < 1 || nvsConfig.fastclock_day > 31)
      {
        nvsConfig.fastclock_day = CONFIG_FASTCLOCK_START_DAY;
        need_persist = true;
      }
      if (nvsConfig.fastclock_hour > 23)
      {
        nvsConfig.fastclock_hour = CONFIG_FASTCLOCK_START_HOUR - 1;
        need_persist = true;
      }
      if (nvsConfig.fastclock_minute > 59)
      {
        nvsConfig.fastclock_minute = CONFIG_FASTCLOCK_START_MINUTE - 1;
        need_persist = true;
      }
      if (nvsConfig.led_brightness < 8)
      {
        nvsConfig.led_brightness = CONFIG_STATUS_LED_BRIGHTNESS;
        need_persist = true;
      }
    }
    if (need_persist)
    {
      ESP_ERROR_CHECK(
          nvs_set_blob(nvs, NVS_CFG_KEY, &nvsConfig, sizeof(node_config_t)));
      ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
    display_nvs_configuration();
    leds->setBrightness(nvsConfig.led_brightness);
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

  void NvsManager::restart_fast_clock()
  {
    if (fastclock.operator->() != nullptr)
    {
      if (fastclock->is_running())
      {
        fastclock->stop();
      }
      fastclock->set_rate_quarters(nvsConfig.fastclock_rate);
      fastclock->set_year(nvsConfig.fastclock_year + 1900);
      fastclock->set_date(nvsConfig.fastclock_month, nvsConfig.fastclock_day);
      fastclock->set_time(nvsConfig.fastclock_hour, nvsConfig.fastclock_minute);
      fastclock->start();
    }
  }

  void NvsManager::save_fast_clock_time()
  {
    if (fastclock.operator->() != nullptr)
    {
      struct tm current_time;
      fastclock->gmtime_r(&current_time);
      nvsConfig.fastclock_rate = fastclock->get_rate_quarters();
      nvsConfig.fastclock_year = current_time.tm_year - 1900;
      nvsConfig.fastclock_month = current_time.tm_mon;
      nvsConfig.fastclock_day = current_time.tm_mday;
      nvsConfig.fastclock_hour = current_time.tm_hour;
      nvsConfig.fastclock_minute = current_time.tm_min;
      persist_configuration();
    }
  }
  bool NvsManager::memory_spaces_modified()
  {
    return false;
  }

  void NvsManager::register_virtual_memory_spaces(openlcb::SimpleStackBase *stack)
  {
    node_id_memoryspace.emplace(stack, this);
    wifi_memory_space.emplace(stack, &nvsConfig);
    fastclock_memory_space.emplace(stack, &nvsConfig);
    realtimeclock_memory_space.emplace(stack, &nvsConfig);
  }

  void NvsManager::register_clocks(openlcb::Node *node, Esp32WiFiManager *wifi_mgr)
  {
    if (nvsConfig.realtimeclock_enabled)
    {
      realtime_clock.emplace(node, nvsConfig.realtimeclock_id);
      realtime_clock->set_rate_quarters(4);
      wifi_mgr->register_network_time_callback(
      [&](time_t sync_time)
      {
        LOG(INFO, "[FastClock] Time sync: %s", ctime(&sync_time));
        struct tm timeinfo;
        localtime_r(&sync_time, &timeinfo);
        realtime_clock->set_time(timeinfo.tm_hour, timeinfo.tm_min);
        realtime_clock->set_date(timeinfo.tm_mon + 1, timeinfo.tm_mday);
        realtime_clock->set_year(timeinfo.tm_year + 1900);
        if (!realtime_clock->is_running())
        {
          realtime_clock->start();
          LOG(INFO, "[FastClock] Starting real-time clock");
        }
        else
        {
          LOG(INFO, "[FastClock] real-time clock synced");
        }
      });
    }
    if (nvsConfig.fastclock_enabled)
    {
      fastclock.emplace(node, nvsConfig.fastclock_id);
      fastclock->set_rate_quarters(nvsConfig.fastclock_rate);
      fastclock->set_year(nvsConfig.fastclock_year + 1900);
      fastclock->set_date(nvsConfig.fastclock_month, nvsConfig.fastclock_day);
      fastclock->set_time(nvsConfig.fastclock_hour, nvsConfig.fastclock_minute);
    }
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

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

#include "ConfigurationManager.h"
#include "JsonConstants.h"
#include "LCCStackManager.h"

#include <dirent.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_types.h>
#include <driver/sdspi_host.h>
#include <esp_spiffs.h>
#include <esp_vfs_fat.h>
#include <Httpd.h>
#include <json.hpp>
#include <openlcb/MemoryConfig.hxx>
#include <sdmmc_cmd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <utils/AutoSyncFileFlow.hxx>
#include <utils/FileUtils.hxx>

using nlohmann::json;

static constexpr const char ESP32_CS_CONFIG_JSON[] = "esp32cs-config.json";
static constexpr const char FACTORY_RESET_MARKER_FILE[] = "resetcfg.txt";

#if defined(CONFIG_ESP32CS_FORCE_FACTORY_RESET_PIN) && CONFIG_ESP32CS_FORCE_FACTORY_RESET_PIN >= 0
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
GPIO_PIN(FACTORY_RESET, GpioInputPU, CONFIG_ESP32CS_FORCE_FACTORY_RESET_PIN);
#else
#include <freertos_drivers/arduino/DummyGPIO.hxx>
typedef DummyPinWithReadHigh FACTORY_RESET_Pin;
#endif // CONFIG_ESP32CS_FORCE_FACTORY_RESET_PIN

// holder of the parsed configuration.
json csConfig;

// Helper which converts a string to a uint64 value.
uint64_t string_to_uint64(string value)
{
  // remove period characters if present
  value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
  // convert the string to a uint64_t value
  return std::stoull(value, nullptr, 16);
}

void recursiveWalkTree(const string &path, bool remove=false)
{
  DIR *dir = opendir(path.c_str());
  if (dir)
  {
    dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL)
    {
      string fullPath = path + "/" + ent->d_name;
      if (ent->d_type == DT_REG)
      {
        struct stat statbuf;
        stat(fullPath.c_str(), &statbuf);
        if (remove)
        {
          LOG(VERBOSE, "[Config] Deleting %s (%lu bytes)", fullPath.c_str()
            , statbuf.st_size);
          ERRNOCHECK(fullPath.c_str(), unlink(fullPath.c_str()));
        }
        else
        {
          LOG(INFO, "[Config] %s (%lu bytes) mtime: %s", fullPath.c_str()
            , statbuf.st_size, ctime(&statbuf.st_mtime));
        }
      }
      else if (ent->d_type == DT_DIR)
      {
        recursiveWalkTree(fullPath, remove);
      }
    }
    closedir(dir);
    if (remove)
    {
      rmdir(path.c_str());
    }
  }
  else
  {
    LOG_ERROR("[Config] Failed to open directory: %s", path.c_str());
  }
}

ConfigurationManager::ConfigurationManager(const esp32cs::Esp32ConfigDef &cfg)
  : cfg_(cfg)
{
#if defined(CONFIG_ESP32CS_FORCE_FACTORY_RESET)
  bool factory_reset_config{true};
#else
  bool factory_reset_config{false};
#endif

  if (FACTORY_RESET_Pin::instance()->is_clr())
  {
    factory_reset_config = true;
  }

  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t sd_slot = SDSPI_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_mount_config_t sd_cfg =
  {
    .format_if_mount_failed = true,
    .max_files = 10,
    .allocation_unit_size = 0
  };
  esp_err_t err = esp_vfs_fat_sdmmc_mount(CFG_MOUNT, &sd_host, &sd_slot
                                        , &sd_cfg, &sd_);
  if (err == ESP_OK)
  {
    LOG(INFO, "[Config] SD card (%s %.2f MB) mounted successfully."
      , sd_->cid.name
      , (float)(((uint64_t)sd_->csd.capacity) * sd_->csd.sector_size) / 1048576);
    FATFS *fsinfo;
    DWORD clusters;
    if (f_getfree("0:", &clusters, &fsinfo) == FR_OK)
    {
      LOG(INFO, "[Config] SD usage: %.2f/%.2f MB",
          (float)(((uint64_t)fsinfo->csize *
                  (fsinfo->n_fatent - 2 - fsinfo->free_clst)) *
                  fsinfo->ssize) / 1048576L,
          (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2)) *
                  fsinfo->ssize) / 1048576L);
    }
    LOG(INFO, "[Config] SD will be used for persistent storage.");
  }
  else
  {
    // unmount the SD VFS since it failed to successfully mount. We will
    // remount SPIFFS in it's place instead.
    esp_vfs_fat_sdmmc_unmount();
    LOG(INFO, "[Config] SD Card not present or mounting failed, using SPIFFS");
    esp_vfs_spiffs_conf_t conf =
    {
      .base_path = CFG_MOUNT,
      .partition_label = NULL,
      .max_files = 10,
      .format_if_mount_failed = true
    };
    // Attempt to mount the partition
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    // check that the partition mounted
    size_t total = 0, used = 0;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK)
    {
      LOG(INFO, "[Config] SPIFFS usage: %.2f/%.2f KiB", (float)(used / 1024.0f)
        , (float)(total / 1024.0f));
    }
    else
    {
      LOG_ERROR("[Config] Unable to retrieve SPIFFS utilization statistics.");
    }
    LOG(INFO, "[Config] SPIFFS will be used for persistent storage.");
  }

  if (exists(FACTORY_RESET_MARKER_FILE))
  {
    factory_reset_config = true;
  }

  if (factory_reset_config)
  {
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING,
        "[Config] The factory reset flag has been set to true, all persistent "
        "data will be cleared.");
    uint8_t countdown = 10;
    while (--countdown)
    {
      LOG(WARNING, "[Config] Factory reset will be initiated in %d seconds..."
        , countdown);
      usleep(SEC_TO_USEC(1));
    }
    LOG(WARNING, "[Config] Factory reset starting!");
  }

  LOG(VERBOSE, "[Config] Persistent storage contents:");
  recursiveWalkTree(CFG_MOUNT, factory_reset_config);
  // Pre-create ESP32 CS configuration directory.
  mkdir(CS_CONFIG_DIR, ACCESSPERMS);
  // Pre-create LCC configuration directory.
  mkdir(LCC_CFG_DIR, ACCESSPERMS);
}

void ConfigurationManager::shutdown()
{
  // Unmount the SPIFFS partition
  if (esp_spiffs_mounted(NULL))
  {
    LOG(INFO, "[Config] Unmounting SPIFFS...");
    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
  }

  // Unmount the SD card if it was mounted
  if (sd_)
  {
    LOG(INFO, "[Config] Unmounting SD...");
    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_unmount());
  }
}

bool ConfigurationManager::exists(const string &name)
{
  struct stat statbuf;
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Checking for %s", configFilePath.c_str());
  // this code is not using access(path, F_OK) as that is not available for
  // SPIFFS VFS. stat(path, buf) does work though.
  return !stat(configFilePath.c_str(), &statbuf);
}

void ConfigurationManager::remove(const string &name)
{
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Removing %s", configFilePath.c_str());
  unlink(configFilePath.c_str());
}

string ConfigurationManager::load(const string &name)
{
  string configFilePath = getFilePath(name);
  if (!exists(name))
  {
    LOG_ERROR("[Config] %s does not exist, returning blank json object"
            , configFilePath.c_str());
    return "{}";
  }
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  return read_file_to_string(configFilePath);
}

void ConfigurationManager::store(const char *name, const string &content)
{
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Storing %s, %d bytes", configFilePath.c_str()
    , content.length());
  write_string_to_file(configFilePath, content);
}

string ConfigurationManager::getFilePath(const string &name)
{
  return StringPrintf("%s/%s", CS_CONFIG_DIR, name.c_str());
}

string ConfigurationManager::getCSConfig()
{
  nlohmann::json clone = csConfig;
  openlcb::TcpClientConfig<openlcb::TcpClientDefaultParams> uplink =
    cfg_.seg().wifi().uplink();
  openmrn_arduino::HubConfiguration hub = cfg_.seg().wifi().hub();
  esp32cs::TrackOutputConfig ops = cfg_.seg().hbridge().entry(0);
  esp32cs::TrackOutputConfig prog = cfg_.seg().hbridge().entry(1);

  // insert non-persistent CDI elements that we can modify from web
  clone[JSON_CDI_NODE][JSON_CDI_UPLINK_NODE] =
  {
    {JSON_CDI_UPLINK_RECONNECT_NODE,
      CDI_READ_TRIMMED(uplink.reconnect, configFd_)},
    {JSON_CDI_UPLINK_MODE_NODE,
      CDI_READ_TRIMMED(uplink.search_mode, configFd_)},
    {JSON_CDI_UPLINK_AUTO_HOST_NODE,
      uplink.auto_address().host_name().read(configFd_)},
    {JSON_CDI_UPLINK_AUTO_SERVICE_NODE,
      uplink.auto_address().service_name().read(configFd_)},
    {JSON_CDI_UPLINK_MANUAL_HOST_NODE,
      uplink.manual_address().ip_address().read(configFd_)},
    {JSON_CDI_UPLINK_MANUAL_PORT_NODE,
      CDI_READ_TRIMMED(uplink.manual_address().port, configFd_)},
  };
  clone[JSON_CDI_NODE][JSON_CDI_HUB_NODE] =
  {
    {JSON_CDI_HUB_ENABLE_NODE, CDI_READ_TRIMMED(hub.enable, configFd_)},
    {JSON_CDI_HUB_PORT_NODE, CDI_READ_TRIMMED(hub.port, configFd_)},
    {JSON_CDI_HUB_SERVICE_NODE, hub.service_name().read(configFd_)},
  };
  clone[JSON_CDI_NODE][JSON_HBRIDGES_NODE] =
  {
    {CONFIG_OPS_TRACK_NAME,
      {
        {JSON_DESCRIPTION_NODE, ops.description().read(configFd_)},
        {JSON_CDI_HBRIDGE_SHORT_EVENT_NODE,
          uint64_to_string_hex(ops.event_short().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHORT_CLEAR_EVENT_NODE,
          uint64_to_string_hex(ops.event_short_cleared().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_EVENT_NODE,
          uint64_to_string_hex(ops.event_shutdown().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_CLEAR_EVENT_NODE,
          uint64_to_string_hex(ops.event_shutdown_cleared().read(configFd_))}
      }
    },
    {CONFIG_PROG_TRACK_NAME,
      {
        {JSON_DESCRIPTION_NODE, prog.description().read(configFd_)},
        {JSON_CDI_HBRIDGE_SHORT_EVENT_NODE,
          uint64_to_string_hex(prog.event_short().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHORT_CLEAR_EVENT_NODE,
          uint64_to_string_hex(prog.event_short_cleared().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_EVENT_NODE,
          uint64_to_string_hex(prog.event_shutdown().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_CLEAR_EVENT_NODE,
          uint64_to_string_hex(prog.event_shutdown_cleared().read(configFd_))},
      }
    },
  };
  return clone.dump();
}

void ConfigurationManager::force_factory_reset()
{
  LOG(INFO, "[Config] Enabling forced factory_reset.");
  string marker = "force factory reset";
  store(FACTORY_RESET_MARKER_FILE, marker);
  Singleton<esp32cs::LCCStackManager>::instance()->reboot_node();
}

#if defined(CONFIG_GPIO_OUTPUTS) || defined(CONFIG_GPIO_SENSORS)
bool is_restricted_pin(int8_t pin)
{
  vector<uint8_t> restrictedPins
  {
#if !defined(CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
    0,                        // Bootstrap / Firmware Flash Download
    1,                        // UART0 TX
    2,                        // Bootstrap / Firmware Flash Download
    3,                        // UART0 RX
    5,                        // Bootstrap
    6, 7, 8, 9, 10, 11,       // on-chip flash pins
    12, 15,                   // Bootstrap / SD pins
#endif // ! CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
    CONFIG_OPS_ENABLE_PIN
  , CONFIG_OPS_SIGNAL_PIN
  , CONFIG_PROG_ENABLE_PIN
  , CONFIG_PROG_SIGNAL_PIN

#if defined(CONFIG_OPS_RAILCOM)
#if defined(CONFIG_OPS_HBRIDGE_LMD18200)
  , CONFIG_OPS_RAILCOM_BRAKE_PIN
#endif
  , CONFIG_OPS_RAILCOM_ENABLE_PIN
  , CONFIG_OPS_RAILCOM_UART_RX_PIN
#endif // CONFIG_OPS_RAILCOM

#if defined(CONFIG_LCC_CAN_ENABLED)
  , CONFIG_LCC_CAN_RX_PIN
  , CONFIG_LCC_CAN_TX_PIN
#endif

#if defined(CONFIG_HC12)
  , CONFIG_HC12_RX_PIN
  , CONFIG_HC12_TX_PIN
#endif

#if defined(CONFIG_NEXTION)
  , CONFIG_NEXTION_RX_PIN
  , CONFIG_NEXTION_TX_PIN
#endif

#if defined(CONFIG_DISPLAY_TYPE_OLED) || defined(CONFIG_DISPLAY_TYPE_LCD)
  , CONFIG_DISPLAY_SCL
  , CONFIG_DISPLAY_SDA
#if defined(CONFIG_DISPLAY_OLED_RESET_PIN) && CONFIG_DISPLAY_OLED_RESET_PIN != -1
  , CONFIG_DISPLAY_OLED_RESET_PIN
#endif
#endif

#if defined(CONFIG_LOCONET)
  , CONFIG_LOCONET_RX_PIN
  , CONFIG_LOCONET_TX_PIN
#endif

#if defined(CONFIG_GPIO_S88)
  , CONFIG_GPIO_S88_CLOCK_PIN
  , CONFIG_GPIO_S88_LOAD_PIN
#if defined(CONFIG_GPIO_S88_RESET_PIN) && CONFIG_GPIO_S88_RESET_PIN != -1
  , CONFIG_GPIO_S88_RESET_PIN
#endif
#endif

#if defined(CONFIG_STATUS_LED)
  , CONFIG_STATUS_LED_DATA_PIN
#endif
  };

  return std::find(restrictedPins.begin()
                 , restrictedPins.end()
                 , pin) != restrictedPins.end();
}
#endif // CONFIG_GPIO_OUTPUTS || CONFIG_GPIO_SENSORS
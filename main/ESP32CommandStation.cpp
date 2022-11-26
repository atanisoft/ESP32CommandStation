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

#include "sdkconfig.h"
#include <AccessoryDecoderDatabase.hxx>
#include <cdi.hxx>
#include <dcc/Packet.hxx>
#include <dcc/PacketSource.hxx>
#include <dcc/UpdateLoop.hxx>
#include <DCCSignalVFS.hxx>
#include <DelayRebootHelper.hxx>
#include <esp_adc_cal.h>
#include <esp_core_dump.h>
#include <esp_ipc.h>
#include <esp_log.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp32/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s3/rom/rtc.h>
#endif
#include <esp_ota_ops.h>
#include <FileSystem.hxx>
#include <EventBroadcastHelper.hxx>
#include <FactoryResetHelper.hxx>
#include <freertos_drivers/esp32/Esp32CoreDumpUtil.hxx>
#include <freertos_drivers/esp32/Esp32SocInfo.hxx>
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <hardware.hxx>
#include <HealthMonitor.hxx>
#include <Httpd.h>
#if CONFIG_ROSTER_EXPOSE_VMS
#include <locodb/LocoDatabaseVirtualMemorySpace.hxx>
#endif
#include <mutex>
#include <NodeRebootHelper.hxx>
#include <NvsManager.hxx>
#include <OpenLCBConfigurationGroup.hxx>
#include <openlcb/BroadcastTime.hxx>
#include <openlcb/BroadcastTimeServer.hxx>
#include <openlcb/MemoryConfigClient.hxx>
#include <openlcb/SimpleStack.hxx>
#include <StatusDisplay.hxx>
#include <StatusLED.hxx>
#include <ThermalMonitorFlow.hxx>
#include <TrainDatabase.hxx>
#include <TrainManager.hxx>
#include <UlpAdc.hxx>
#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>

/// Enable usage of select() for GridConnect connections.
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

/// Increase the number of socket backlog connections to improve performance of
/// the http server.
OVERRIDE_CONST(socket_listener_backlog, 3);

/// Increase the number of memory spaces available at runtime to account for
/// the Traction protocol CDI/FDI needs.
OVERRIDE_CONST(num_memory_spaces, CONFIG_OLCB_MEMORY_SPACES);

/// Increase the GridConnect buffer size to improve performance by bundling
/// more than one GridConnect packet into the same send() call to the socket.
OVERRIDE_CONST(gridconnect_buffer_size, CONFIG_LWIP_TCP_MSS);

/// Increase the time for the buffer to fill up before sending it out over the
/// socket connection.
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 500);

/// This limites the number of outbound GridConnect packets which limits the
/// memory used by the BufferPort.
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets,
               CONFIG_OLCB_GC_OUTBOUND_PACKET_LIMIT);

/// This limites the number of incoming GridConnect packets which limits the
/// memory used by the BufferPort.
OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets,
               CONFIG_OLCB_GC_INBOUND_PACKET_LIMIT);

/// This increases number of state flows to invoke before checking for any FDs
/// that have pending data.
OVERRIDE_CONST(executor_select_prescaler,
               CONFIG_OLCB_EXECUTOR_SELECT_PRESCALER);

/// Remove restriction on number of local nodes that the OpenLCB stack allows
/// since the CS will host a number of virtual train nodes.
OVERRIDE_CONST(local_nodes_count, 0);

/// Increase the number of local aliases available for the OpenLCB stack so we
/// have extras available for virtual train nodes.
OVERRIDE_CONST(local_alias_cache_size, CONFIG_OLCB_LOCAL_ALIAS_COUNT);

/// Allow adjustment of the CAN TX/RX buffer sizes via config.
OVERRIDE_CONST(can_tx_buffer_size, CONFIG_OLCB_TWAI_TX_BUFFER_SIZE);
OVERRIDE_CONST(can_rx_buffer_size, CONFIG_OLCB_TWAI_RX_BUFFER_SIZE);

#if CONFIG_OLCB_GC_NEWLINES
/// Generate GridConnect frames with a newline appended to each frame.
OVERRIDE_CONST_TRUE(gc_generate_newlines);
#endif // CONFIG_OLCB_GC_NEWLINES

/// ESP32 CS does not support the Marklin protocol.
OVERRIDE_CONST_FALSE(trainmgr_support_marklin);

#ifndef CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS
OVERRIDE_CONST_FALSE(trainmgr_automatically_create_train_impl);
#endif // CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS

/// CDI configuration for the ESP32 Commmand Station.
/// Offset is set to zero and is likely to be removed in the future.
static constexpr esp32cs::ConfigDef cfg(0);

/// When the CDI data is persisted on an SD card we need to periodically call
/// fsync to ensure the changes are not left in cache since the file is kept
/// open while the ESP32 Command Station remains running.
std::unique_ptr<AutoSyncFileFlow> sd_auto_sync;

/// Reboot helper for the ESP32 Command Station which will ensure that all
/// dependent systems are shutdown prior to actual restart.
std::unique_ptr<esp32cs::NodeRebootHelper> reboot_helper;

#if CONFIG_OLCB_TWAI_ENABLED
/// ESP32 Hardware TWAI driver instance.
Esp32HardwareTwai twai(CONFIG_OLCB_TWAI_RX_PIN, CONFIG_OLCB_TWAI_TX_PIN);
#endif // CONFIG_OLCB_TWAI_ENABLED

namespace openlcb
{
  /// Define the SNIP data for the ESP32 Command Station.
  const SimpleNodeStaticValues SNIP_STATIC_DATA =
      {
          4,                                      /* version           */
          SNIP_PROJECT_PAGE,                      /* manufacturer_name */
          SNIP_PROJECT_NAME,                      /* model_name        */
          SNIP_HW_VERSION " " CONFIG_IDF_TARGET,  /* hardware_version  */
          SNIP_SW_VERSION                         /* software_version  */
      };

  /// Path to where OpenMRN should persist general configuration data.
  const char *const CONFIG_FILENAME = "/fs/olcb_config";

  /// The size of the memory space to export over the above device.
  const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

  /// Default to store the dynamic SNIP data is stored in the same persistant
  /// data file as general configuration data.
  const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
} // namespace openlcb

void init_webserver(Service *service, esp32cs::NvsManager *nvs_mgr,
                    openlcb::Node *node, openlcb::MemoryConfigHandler *mem_cfg,
                    esp32cs::Esp32TrainDatabase *train_db);

void check_for_coredump();

using esp32cs::NvsManager;
using esp32cs::StatusLED;

class OpenLCBFactoryResetHelper : public DefaultConfigUpdateListener
{
public:
    OpenLCBFactoryResetHelper(
      const esp32cs::OpenLCBConfiguration &cfg, uint8_t reset_reason)
      : cfg_(cfg), resetReason_(reset_reason)
    {
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        LOG(VERBOSE, "[CFG] apply_configuration(%d, %d)", fd, initial_load);

        uint8_t dcc_en = CDI_READ_TRIM_DEFAULT(cfg_.advanced().dcc_enable, fd);
        auto lcc_dcc = get_dcc_output(DccOutput::LCC);
        LOG(INFO, "[OpenLCB] DCC signal output: %s",
            dcc_en ? "Enabled" : "Disabled");
        if (dcc_en)
        {
          lcc_dcc->clear_disable_output_for_reason(
            DccOutput::DisableReason::CONFIG_SETTING);
        }
        else
        {
          lcc_dcc->disable_output_for_reason(
            DccOutput::DisableReason::CONFIG_SETTING);
        }

        uint8_t brownout_en =
          CDI_READ_TRIM_DEFAULT(cfg_.advanced().brownout_enable, fd);
        LOG(INFO, "[OpenLCB] Brownout event publishing: %s",
            dcc_en ? "Enabled" : "Disabled");
        if (resetReason_ == RTCWDT_BROWN_OUT_RESET && brownout_en)
        {
          LOG(INFO, "[OpenLCB] Brownout event detected, trying to send event");
          Singleton<esp32cs::EventBroadcastHelper>::instance()->send_event(
            openlcb::Defs::POWER_STANDARD_BROWNOUT_EVENT);
        }

        uint8_t acc_en =
          CDI_READ_TRIM_DEFAULT(cfg_.advanced().enable_accessory_bus, fd);
        LOG(INFO, "[OpenLCB] DCC Accessory Decoder Listener: %s",
            dcc_en ? "Enabled" : "Disabled");
        Singleton<esp32cs::AccessoryDecoderDB>::instance()->configure(acc_en);

        uint8_t throttles_en =
          CDI_READ_TRIM_DEFAULT(cfg_.advanced().enable_throttles, fd);
        LOG(INFO, "[OpenLCB] Train Search Listener: %s",
            dcc_en ? "Enabled" : "Disabled");
        Singleton<locomgr::LocoManager>::instance()->set_enabled(throttles_en);

        return ConfigUpdateListener::UpdateAction::UPDATED;
    }

    void factory_reset(int fd) override
    {
        LOG(VERBOSE, "[CDI] OpenLCBConfiguration factory reset invoked.");
        CDI_FACTORY_RESET(cfg_.advanced().brownout_enable);
        CDI_FACTORY_RESET(cfg_.advanced().dcc_enable);
        CDI_FACTORY_RESET(cfg_.advanced().enable_accessory_bus);
        CDI_FACTORY_RESET(cfg_.advanced().enable_throttles);
        CDI_FACTORY_RESET(cfg_.advanced().throttle_heartbeat);
    }
private:
    const esp32cs::OpenLCBConfiguration cfg_;
    const uint8_t resetReason_;
};

/// Utility method that verifies if a core dump exists in the flash partition.
///
/// When one is found the on-board LEDs will be set to:
/// WIFI_STA:   YELLOW
/// WIFI_AP:    RED
/// BOOTLOADER: YELLOW
/// OPS_TRACK:  RED
/// PROG_TRACK: YELLOW
/// and will blink in alterating fashion.
///
/// When ESP-IDF v4.4+ is used the coredump will be converted to a text file
/// which will be available on the SD card as "coredump.txt". For earlier
/// ESP-IDF versions no conversion is possible.
///
/// After the conversion of the core dump is complete the CS will attempt to
/// verify if the user has requested the core dump to be erased from flash
/// by pressing and holding the FACTORY_RESET button for at least one second
/// during the first fifteen seconds of the blinking LED pattern. If the
/// FACTORY_RESET button is not pressed during this period the CS will halt
/// execution with the blink pattern continued.
void check_for_coredump()
{
#if CONFIG_CRASH_COLLECT_CORE_DUMP
  if (Esp32CoreDumpUtil::is_present())
  {
    auto leds = Singleton<StatusLED>::instance();
    // Give visual indication that there is a core dump using yellow and red
    // blinking LEDs.
    leds->set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::YELLOW_BLINK, true);
    leds->set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::RED_BLINK);
    leds->set(StatusLED::LED::BOOTLOADER, StatusLED::COLOR::YELLOW_BLINK, true);
    leds->set(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::RED_BLINK);
    leds->set(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::YELLOW_BLINK, true);
    esp32cs::mount_fs(false);
    Esp32CoreDumpUtil::display("/fs/coredump.txt");
    esp32cs::unmount_fs();
    Esp32CoreDumpUtil::cleanup();

    // Clear LEDs
    leds->clear();
  }
#endif // CONFIG_CRASH_COLLECT_CORE_DUMP
}

extern "C"
{

/// Application main entry point.
void app_main()
{
  const esp_app_desc_t *app_data = esp_ota_get_app_description();
  LOG(INFO, "\n\nESP32 Command Station starting up...");
  LOG(INFO, "Compiled on %s %s using IDF %s", app_data->date, app_data->time,
      app_data->idf_ver);
  LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);
  LOG(INFO, "ESP32 Command Station uses the OpenMRN library\n"
            "Copyright (c) 2019-2022, OpenMRN\n"
            "All rights reserved.");
  LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s",
      openlcb::SNIP_STATIC_DATA.version,
      openlcb::SNIP_STATIC_DATA.manufacturer_name,
      openlcb::SNIP_STATIC_DATA.model_name,
      openlcb::SNIP_STATIC_DATA.hardware_version,
      openlcb::SNIP_STATIC_DATA.software_version);
  LOG(INFO, "[CDI] Size: %zu, Version:%04x", openlcb::CDI_SIZE, CDI_VERSION);
  uint8_t reset_reason = Esp32SocInfo::print_soc_info();
  GpioInit::hw_init();
  DccHwDefs::InternalBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
  NvsManager nvs;
  StatusLED leds;
  nvs.init(reset_reason);
  check_for_coredump();  

  if (nvs.start_stack())
  {
    bool using_sd = esp32cs::mount_fs(nvs.should_reset_config());

    esp32cs::initialize_ulp_adc();

    // initialize the OpenMRN stack and dependent components
    openlcb::SimpleCanStack stack(nvs.node_id());
    Esp32WiFiManager wifi_manager(nvs.station_ssid(), nvs.station_password(),
                                  &stack, cfg.seg().olcb().wifi(),
                                  nvs.wifi_mode(),
                                  CONFIG_OLCB_UPLINK_MODE_DEFAULT,
                                  nvs.hostname_prefix(),
                                  nvs.sntp_server(), nvs.timezone(),
                                  nvs.sntp_enabled(), nvs.softap_channel(),
                                  nvs.softap_auth(), nvs.softap_ssid(),
                                  nvs.softap_password());
    wifi_manager.register_network_up_callback(
      [&](esp_network_interface_t interface, uint32_t ip)
      {
        if (interface == esp_network_interface_t::SOFTAP_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::BLUE);
        }
        else if (interface == esp_network_interface_t::STATION_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::GREEN);
        }
    });
    wifi_manager.register_network_down_callback(
      [&](esp_network_interface_t interface)
      {
        if (interface == esp_network_interface_t::SOFTAP_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::OFF);
        }
        else if (interface == esp_network_interface_t::STATION_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::OFF);
        }
    });
    wifi_manager.register_network_init_callback(
      [&](esp_network_interface_t interface)
      {
        if (interface == esp_network_interface_t::SOFTAP_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_AP, StatusLED::COLOR::BLUE_BLINK);
        }
        else if (interface == esp_network_interface_t::STATION_INTERFACE)
        {
          leds.set(StatusLED::LED::WIFI_STA, StatusLED::COLOR::GREEN_BLINK);
        }
    });
    wifi_manager.display_configuration();

    // declare EventBroadcastHelper first since it is used in the
    // OpenLCBFactoryResetHelper code for brownout event publishing.
    esp32cs::EventBroadcastHelper event_helper(stack.service(), stack.node());
    esp32cs::FactoryResetHelper factory_reset_helper(cfg.userinfo());
    OpenLCBFactoryResetHelper olcb_factory_reset(cfg.seg().olcb(), reset_reason);
    esp32cs::DelayRebootHelper delayed_reboot(&wifi_manager);
    esp32cs::HealthMonitor health_monitor(&wifi_manager);
    esp32cs::StatusDisplay status_display(&wifi_manager,
                                          &wifi_manager, &nvs);
    openlcb::TrainService trainService(stack.iface());
    esp32cs::Esp32TrainDatabase train_db(&stack, &wifi_manager);
#if CONFIG_ROSTER_EXPOSE_VMS
    locodb::LocoDatabaseVirtualMemorySpace train_db_vms(&stack);
#endif // CONFIG_ROSTER_EXPOSE_VMS

    MDNS mdns;
    http::Httpd httpd(&wifi_manager, &mdns);
    esp32cs::ThermalMonitorFlow thermal_monitor(&wifi_manager,
                                                stack.node(),
                                                cfg.seg().thermal());
    esp32cs::init_dcc(stack.node(), stack.service(), cfg.seg().track());
    nvs.register_virtual_memory_spaces(&stack);
    nvs.register_clocks(stack.node(), &wifi_manager);

    // Initialize after DCC since there is a dependency on UpdataLoop
    trainmanager::TrainManager train_mgr(&trainService, stack.info_flow(),
                                         stack.memory_config_handler());

    // Create config file and initiate factory reset if it doesn't exist or is
    // otherwise corrupted.
    int config_fd =
        stack.create_config_file_if_needed(cfg.seg().internal_config(),
                                           CDI_VERSION,
                                           openlcb::CONFIG_FILE_SIZE);
    if (using_sd)
    {
      LOG(INFO, "[FS] Configuring fsync of data to SD card ever %d seconds.",
          CONFIG_OLCB_SD_FSYNC_SEC);
      sd_auto_sync =
        std::make_unique<AutoSyncFileFlow>(&wifi_manager, config_fd,
                                           SEC_TO_USEC(CONFIG_OLCB_SD_FSYNC_SEC));
    }
    reboot_helper =
      std::make_unique<esp32cs::NodeRebootHelper>(
        &stack, &nvs, config_fd, sd_auto_sync.get());

#if CONFIG_OLCB_TWAI_ENABLED
    twai.hw_init();
    LOG(INFO, "[TWAI] Enabling select() API");
    stack.add_can_port_select("/dev/twai/twai0");
#endif // CONFIG_OLCB_TWAI_ENABLED

#if CONFIG_OLCB_PRINT_ALL_PACKETS
    stack.print_all_packets();
#endif

    init_webserver(&wifi_manager, &nvs, stack.node(),
                   stack.memory_config_handler(), &train_db);

    // hand-off to the OpenMRN stack executor
    stack.loop_executor();
  }
}

/// Node reboot task entry point.
void *node_reboot(void *arg)
{
  // Kick off the node reboot process
  reboot_helper->reboot();
  return nullptr;
}

/// OpenMRN reboot request method from the Memory Configuration Protocol.
void reboot()
{
  os_thread_create(nullptr, nullptr, 0, 0, node_reboot, nullptr);
}

/// OpenMRN free heap hook, implemented to return the free heap that can be
/// accessed as uint8_t.
ssize_t os_get_free_heap(void)
{
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

std::mutex log_mux;
// OpenMRN log output method, overridden to add mutex guard around fwrite/fputc
// due to what appears to be a bug in esp-idf where it thinks a recursive mutex
// is being held and that it is in an ISR context.
void log_output(char* buf, int size)
{
  if (size > 0)
  {
    const std::lock_guard<std::mutex> lock(log_mux);
    buf[size] = '\0';
    puts(buf);
  }
}

} // extern "C"
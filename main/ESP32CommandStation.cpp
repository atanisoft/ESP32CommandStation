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
#include "cdi.hxx"
#include "DelayRebootHelper.hxx"
#include "EventBroadcastHelper.hxx"
#include "FactoryResetHelper.hxx"
#include "FileSystem.hxx"
#include "hardware.hxx"
#include "HealthMonitor.hxx"
#include "NodeIdMemoryConfigSpace.hxx"
#include "NodeRebootHelper.hxx"
#include "NvsManager.hxx"
#include "StatusDisplay.hxx"
#include "ThermalMonitorFlow.hxx"
#include <AllTrainNodes.hxx>
#include <CDIXMLGenerator.hxx>
#include <dcc/Packet.hxx>
#include <dcc/PacketSource.hxx>
#include <dcc/UpdateLoop.hxx>
#include <DCCSignalVFS.hxx>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <freertos_drivers/esp32/Esp32SocInfo.hxx>
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <Httpd.h>
#include <mutex>
#include <openlcb/MemoryConfigClient.hxx>
#include <openlcb/SimpleStack.hxx>
#include <StatusLED.h>
#include <TrainDatabase.h>
#include <Turnouts.h>
#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>

///////////////////////////////////////////////////////////////////////////////
// Enable usage of select() for GridConnect connections.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

///////////////////////////////////////////////////////////////////////////////
// Increase the number of socket backlog connections to improve performance of
// the http server.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(socket_listener_backlog, 3);

///////////////////////////////////////////////////////////////////////////////
// Increase the number of memory spaces available at runtime to account for the
// Traction protocol CDI/FDI needs.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(num_memory_spaces, CONFIG_OLCB_MEMORY_SPACES);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_buffer_size, CONFIG_LWIP_TCP_MSS);

///////////////////////////////////////////////////////////////////////////////
// Increase the time for the buffer to fill up before sending it out over the
// socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 500);

///////////////////////////////////////////////////////////////////////////////
// This limites the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets,
               CONFIG_OLCB_GC_OUTBOUND_PACKET_LIMIT);

///////////////////////////////////////////////////////////////////////////////
// This limites the number of incoming GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets,
               CONFIG_OLCB_GC_INBOUND_PACKET_LIMIT);

///////////////////////////////////////////////////////////////////////////////
// This increases number of state flows to invoke before checking for any FDs
// that have pending data.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(executor_select_prescaler, CONFIG_OLCB_EXECUTOR_SELECT_PRESCALER);

///////////////////////////////////////////////////////////////////////////////
// This increases the number of local nodes and aliases available for the
// OpenLCB stack. This is needed to allow for virtual train nodes.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(local_nodes_count, CONFIG_OLCB_LOCAL_NODE_COUNT);
OVERRIDE_CONST(local_alias_cache_size, CONFIG_OLCB_LOCAL_NODE_COUNT);

// Esp32ConfigDef comes from CSConfigDescriptor.h and is specific to this
// particular device and target. It defines the layout of the configuration
// memory space and is also used to generate the cdi.xml file. Here we
// instantiate the configuration layout. The argument of offset zero is ignored
// and will be removed later.
static constexpr esp32cs::ConfigDef cfg(0);
static Esp32HardwareTwai twai(CONFIG_OLCB_TWAI_RX_PIN, CONFIG_OLCB_TWAI_TX_PIN);
static esp32cs::NvsManager nvs;
static esp32cs::StatusLED leds;
static uninitialized<openlcb::SimpleCanStack> stack;
static uninitialized<openlcb::MemoryConfigClient> memory_client;
static uninitialized<openlcb::TrainService> trainService;
static uninitialized<AutoSyncFileFlow> sd_auto_sync;
static uninitialized<Esp32WiFiManager> wifi_manager;
static uninitialized<commandstation::AllTrainNodes> trains;
static uninitialized<esp32cs::NodeRebootHelper> node_reboot_helper;
static uninitialized<esp32cs::NodeIdMemoryConfigSpace> node_id_memoryspace;
static uninitialized<esp32cs::FactoryResetHelper> factory_reset_helper;
static uninitialized<esp32cs::EventBroadcastHelper> event_helper;
static uninitialized<esp32cs::DelayRebootHelper> delayed_reboot;
static uninitialized<esp32cs::HealthMonitor> health_monitor;
static uninitialized<esp32cs::StatusDisplay> status_display;
static uninitialized<esp32cs::Esp32TrainDatabase> train_db;
#if TEMPSENSOR_ADC_CHANNEL != -1
static uninitialized<esp32cs::ThermalMonitorFlow> thermal_monitor;
#endif
static uninitialized<esp32cs::TurnoutManager> turnout_manager;
static uninitialized<http::Httpd> httpd;
static MDNS mdns;

namespace openlcb
{
  // define the SNIP data for the Command Station.
  const SimpleNodeStaticValues SNIP_STATIC_DATA =
      {
          4,                  /* version           */
          SNIP_PROJECT_PAGE,  /* manufacturer_name */
          SNIP_PROJECT_NAME,  /* model_name        */
          SNIP_HW_VERSION,    /* hardware_version  */
          SNIP_SW_VERSION     /* software_version  */
      };

  // This will stop openlcb from exporting the CDI memory space upon start.
  const char CDI_DATA[] = "";

  // Path to where OpenMRN should persist general configuration data.
  const char *const CONFIG_FILENAME = "/fs/olcb_config";

  // The size of the memory space to export over the above device.
  const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

  // Default to store the dynamic SNIP data is stored in the same persistant
  // data file as general configuration data.
  const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;

  /// Name of CDI.xml to generate dynamically.
  const char CDI_FILENAME[] = "/fs/cdi.xml";
} // namespace openlcb

void init_webserver(openlcb::SimpleCanStack *can_stack,
                    Esp32WiFiManager *wifi_mgr,
                    esp32cs::NvsManager *nvs_mgr,
                    openlcb::MemoryConfigClient *mem_client);

extern "C" void app_main()
{
  const esp_app_desc_t *app_data = esp_ota_get_app_description();
  LOG(INFO, "\n\nESP32 Command Station starting up...");
  LOG(INFO, "Compiled on %s %s using IDF %s", app_data->date, app_data->time, app_data->idf_ver);
  LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);
  LOG(INFO, "ESP32 Command Station uses the OpenMRN library\n"
            "Copyright (c) 2019-2021, OpenMRN\n"
            "All rights reserved.");
  LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s",
      openlcb::SNIP_STATIC_DATA.version,
      openlcb::SNIP_STATIC_DATA.manufacturer_name,
      openlcb::SNIP_STATIC_DATA.model_name,
      openlcb::SNIP_STATIC_DATA.hardware_version,
      openlcb::SNIP_STATIC_DATA.software_version);
  uint8_t reset_reason = Esp32SocInfo::print_soc_info();
  GpioInit::hw_init();
#if CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY
  DccHwDefs::Output1::set_disable_reason(
        DccOutput::DisableReason::CONFIG_SETTING);
#elif CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
  DccHwDefs::Output2::set_disable_reason(
        DccOutput::DisableReason::CONFIG_SETTING);
#endif
  DccHwDefs::Output1::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
  DccHwDefs::Output2::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
  nvs.init(reset_reason);

  if (nvs.start_stack())
  {
    twai.hw_init();

    // Configure ADC1 up front to use 12 bit (0-4095) as we use it for all
    // monitored h-bridges.
    LOG(INFO, "[ADC] Configure 12-bit ADC resolution");
    adc1_config_width(ADC_WIDTH_BIT_12);
    bool using_sd = esp32cs::mount_fs(nvs.should_reset_config());

    // initialize the OpenMRN stack and dependent components
    stack.emplace(nvs.node_id());
    wifi_manager.emplace(nvs.station_ssid(), nvs.station_password(),
                         stack.operator->(), cfg.seg().wifi(),
                         nvs.wifi_mode(), nvs.hostname_prefix(),
                         nvs.sntp_server(), nvs.timezone(),
                         nvs.sntp_enabled(), nvs.softap_channel(),
                         nvs.softap_auth(), nvs.softap_ssid(),
                         nvs.softap_password());
    leds.attach_callbacks(wifi_manager.operator->());
    node_id_memoryspace.emplace(stack.operator->(), &nvs);
    factory_reset_helper.emplace(cfg.userinfo());
    event_helper.emplace(stack.operator->());
    delayed_reboot.emplace(stack->service());
    health_monitor.emplace(stack->service());
    status_display.emplace(stack->service(), wifi_manager.operator->(), &nvs);
    trainService.emplace(stack->iface());
    train_db.emplace(stack.operator->());
    trains.emplace(train_db.operator->(), trainService.operator->(),
                   stack->info_flow(), stack->memory_config_handler(),
                   train_db->get_train_cdi(), train_db->get_temp_train_cdi());
    httpd.emplace(wifi_manager.operator->(), &mdns);
    memory_client.emplace(stack->node(), stack->memory_config_handler());
#if TEMPSENSOR_ADC_CHANNEL != -1
    thermal_monitor.emplace(stack->service(), stack->node(),
                            cfg.seg().thermal(),
                            (adc1_channel_t)TEMPSENSOR_ADC_CHANNEL);
#endif
    esp32cs::init_dcc(stack->node(), stack->service(), cfg.seg().track());
    turnout_manager.emplace(stack->node(), stack->service());

    // Create / update CDI, if the CDI is out of date a factory reset will be
    // forced.
    if (CDIXMLGenerator::create_config_descriptor_xml(
        cfg, openlcb::CDI_FILENAME, stack.operator->()))
    {
      LOG(WARNING, "[CDI] Forcing factory reset due to CDI update");
      unlink(openlcb::CONFIG_FILENAME);
    }

    // Create config file and initiate factory reset if it doesn't exist or is
    // otherwise corrupted.
    int config_fd =
        stack->create_config_file_if_needed(cfg.seg().internal_config(),
                                            CDI_VERSION,
                                            openlcb::CONFIG_FILE_SIZE);
    node_reboot_helper.emplace(stack.operator->(), config_fd);

    if (using_sd)
    {
      LOG(INFO, "[FS] Configuring fsync of data to SD card ever %d seconds.",
          CONFIG_OLCB_SD_FSYNC_SEC);
      sd_auto_sync.emplace(stack->service(), config_fd,
                           SEC_TO_USEC(CONFIG_OLCB_SD_FSYNC_SEC));
    }

#if CONFIG_OLCB_TWAI_SELECT
    LOG(INFO, "[TWAI] Enabling select() API");
    stack->add_can_port_select("/dev/twai/twai0");
#else
    LOG(INFO, "[TWAI] Enabling async API");
    stack->add_can_port_async("/dev/twai/twai0");
#endif

    init_webserver(stack.operator->(), wifi_manager.operator->(), &nvs,
                   memory_client.operator->());

    // Start the stack in the background using it's own task.
    stack->start_executor_thread("OpenMRN", 0, 4096);
  }
}

extern "C" void *node_reboot(void *arg)
{
  node_reboot_helper->reboot();
  return nullptr;
}

extern "C" void reboot()
{
  os_thread_create(nullptr, nullptr, 0, 0, node_reboot, nullptr);
}

extern "C" ssize_t os_get_free_heap()
{
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

std::mutex log_mux;
// OpenMRN log output method, overridden to add mutex guard around fwrite/fputc
// due to what appears to be a bug in esp-idf where it thinks a recursive mutex
// is being held and that it is in an ISR context.
extern "C" void log_output(char* buf, int size)
{
    const std::lock_guard<std::mutex> lock(log_mux);
    // drop null/short messages
    if (size <= 0) return;

    // no error checking is done here, any error check logs would get lost if
    // there was a failure at this point anyway.
    fwrite(buf, 1, size, stdout);
    fputc('\n', stdout);
}
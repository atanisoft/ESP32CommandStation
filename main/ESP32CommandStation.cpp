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
#include "CSConfigDescriptor.h"
#include "ESP32TrainDatabase.h"
#include "nvs.hxx"
#include "OTAMonitor.h"

#include <AllTrainNodes.hxx>
#include <FileSystemManager.h>
#include <DCCSignalVFS.h>
#include <driver/uart.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_task.h>
#include <esp32/rom/rtc.h>
#include <FreeRTOSTaskMonitor.h>
#if !defined(CONFIG_WIFI_MODE_DISABLED)
#include <Httpd.h>
#include <HttpStringUtils.h>
#endif
#include <LCCStackManager.h>
#include <LCCWiFiManager.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <os/MDNS.hxx>
#include <Turnouts.h>

#if CONFIG_HC12
#include <HC12Radio.h>
#endif

#if !CONFIG_DISPLAY_TYPE_NONE
#include <StatusDisplay.h>
#endif

#if CONFIG_STATUS_LED
#include <StatusLED.h>
#endif

#if CONFIG_THERMALMONITOR
#include <ThermalMonitorFlow.hxx>
#endif // CONFIG_THERMALMONITOR

#if CONFIG_GPIO_SENSORS
#include <Sensors.h>
#include <RemoteSensors.h>
#if CONFIG_GPIO_S88
#include <S88Sensors.h>
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS
#if CONFIG_GPIO_OUTPUTS
#include <Outputs.h>
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_JMRI
#include <JmriInterface.h>
#endif

///////////////////////////////////////////////////////////////////////////////
// Set the priority of the httpd executor to the effective value used for the
// primary OpenMRN executor.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(httpd_server_priority, ESP_TASK_MAIN_PRIO);

///////////////////////////////////////////////////////////////////////////////
// Increase the number of CAN frame queue size to reduce the number of dropped
// frames.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(can_rx_buffer_size, 64);

#if CONFIG_LCC_GC_NEWLINES
///////////////////////////////////////////////////////////////////////////////
// This will generate newlines after GridConnect each packet being sent.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gc_generate_newlines);
#endif

///////////////////////////////////////////////////////////////////////////////
// Increase the number of socket backlog connections to improve performance of
// the http server.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(socket_listener_backlog, 3);

///////////////////////////////////////////////////////////////////////////////
// Increase the number of memory spaces available at runtime to account for the
// Traction protocol CDI/FDI needs.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(num_memory_spaces, CONFIG_LCC_MEMORY_SPACES);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_buffer_size, CONFIG_LWIP_TCP_MSS);

///////////////////////////////////////////////////////////////////////////////
// Increase the time for the buffer to fill up before sending it out over the
// socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_buffer_delay_usec, 1500);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,2,0)
///////////////////////////////////////////////////////////////////////////////
// Enable usage of select() for GridConnect connections.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);
#endif // CONFIG_LCC_USE_SELECT

///////////////////////////////////////////////////////////////////////////////
// This limites the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_bridge_max_outgoing_packets
                      , CONFIG_LCC_GC_OUTBOUND_PACKET_LIMIT);

///////////////////////////////////////////////////////////////////////////////
// This increases number of state flows to invoke before checking for any FDs
// that have pending data.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(executor_select_prescaler
                      , CONFIG_LCC_EXECUTOR_SELECT_PRESCALER);

///////////////////////////////////////////////////////////////////////////////
// This increases the number of local nodes and aliases available for the LCC
// stack. This is needed to allow for virtual train nodes.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(local_nodes_count, CONFIG_LCC_LOCAL_NODE_COUNT);
OVERRIDE_CONST_DEFERRED(local_alias_cache_size, CONFIG_LCC_LOCAL_NODE_COUNT);

///////////////////////////////////////////////////////////////////////////////
// Increase the WebSocket URI count since the CS offers both DCC++ (/ws) and
// JSON (/wsjson) based WebSockets.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(httpd_websocket_max_uris, 2);

// Esp32ConfigDef comes from CSConfigDescriptor.h and is specific to this
// particular device and target. It defines the layout of the configuration
// memory space and is also used to generate the cdi.xml file. Here we
// instantiate the configuration layout. The argument of offset zero is ignored
// and will be removed later.
static constexpr esp32cs::Esp32ConfigDef cfg(0);

// define the SNIP data for the Command Station.
namespace openlcb
{
  const SimpleNodeStaticValues SNIP_STATIC_DATA =
  {
    4,
    "github.com/atanisoft (Mike Dunston)",
    "ESP32 Command Station",
    CDI_HW_VERSION,
    SNIP_SW_VERSION
  };
}

// override LCC defaults with the ESP32 CS values.
namespace openlcb
{
  // Path to where OpenMRN should persist general configuration data.
  const char *const CONFIG_FILENAME = LCC_CONFIG_FILE;

  // The size of the memory space to export over the above device.
  const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

  // Default to store the dynamic SNIP data is stored in the same persistant
  // data file as general configuration data.
  const char *const SNIP_DYNAMIC_FILENAME = LCC_CONFIG_FILE;

  extern const char CDI_DATA[];
  const char CDI_DATA[] = R"xmlpayload(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>github.com/atanisoft (Mike Dunston)</manufacturer>
<model>ESP32 Command Station</model>
<hardwareVersion>)xmlpayload" CDI_HW_VERSION R"xmlpayload(</hardwareVersion>
<softwareVersion>)xmlpayload" CDI_SW_VERSION R"xmlpayload(</softwareVersion>
</identification>
<acdi/>
<segment space='251' origin='1'>
<string size='63'>
<name>User Name</name>
<description>This name will appear in network browsers for the current node.</description>
</string>
<string size='64'>
<name>User Description</name>
<description>This description will appear in network browsers for the current node.</description>
</string>
</segment>
<segment space='253' origin='128'>
<group>
<name>Internal data</name>
<description>Do not change these settings.</description>
<int size='2'>
<name>Version</name>
</int>
<int size='2'>
<name>Next event ID</name>
</int>
</group>
<group>
<name>WiFi Configuration</name>
<int size='1'>
<name>WiFi mode</name>
<description>Configures the WiFi operating mode.</description>
<min>0</min>
<max>3</max>
<default>2</default>
<map><relation><property>0</property><value>Off</value></relation><relation><property>1</property><value>Station Only</value></relation><relation><property>2</property><value>SoftAP Only</value></relation><relation><property>3</property><value>SoftAP and Station</value></relation></map>
</int>
<string size='21'>
<name>Hostname prefix</name>
<description>Configures the hostname prefix used by the node.
Note: the node ID will be appended to this value.</description>
</string>
<string size='32'>
<name>SSID</name>
<description>Configures the SSID that the ESP32 will connect to.</description>
</string>
<string size='128'>
<name>Password</name>
<description>Configures the SSID that the ESP32 will connect to.</description>
</string>
<string size='32'>
<name>SSID</name>
<description>Configures the SSID that the ESP32 will use for the SoftAP.</description>
</string>
<string size='128'>
<name>Password</name>
<description>Configures the SSID that the ESP32 will use for the SoftAP.</description>
</string>
<int size='1'>
<name>Authentication Mode</name>
<description>Configures the authentication mode of the SoftAP.</description>
<min>0</min>
<max>7</max>
<default>3</default>
<map><relation><property>0</property><value>Open</value></relation><relation><property>1</property><value>WEP</value></relation><relation><property>2</property><value>WPA</value></relation><relation><property>3</property><value>WPA2</value></relation><relation><property>4</property><value>WPA/WPA2</value></relation><relation><property>6</property><value>WPA3</value></relation><relation><property>7</property><value>WPA2/WPA3</value></relation></map>
</int>
<int size='1'>
<name>WiFi Channel</name>
<description>Configures the WiFi channel to use for the SoftAP.
Note: Some channels overlap eachother and may not provide optimal performance.Recommended channels are: 1, 6, 11 since these do not overlap.</description>
<min>1</min>
<max>14</max>
<default>1</default>
</int>
<int size='1'>
<name>Enable SNTP</name>
<description>Enabling this option will allow the ESP32 to poll an SNTP server at regular intervals to obtain the current time. The refresh interval roughly once per hour.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='64'>
<name>SNTP Server</name>
<description>Enter the SNTP Server address. Example: pool.ntp.org
Most of the time this does not need to be changed.</description>
</string>
<string size='64'>
<name>TimeZone</name>
<description>This is the timezone that the ESP32 should use, note it must be in POSIX notation. Note: The timezone is only configured when SNTP is also enabled.
A few common values:
PST8PDT,M3.2.0,M11.1.0 -- UTC-8 with automatic DST adjustment
MST7MDT,M3.2.0,M11.1.0 -- UTC-7 with automatic DST adjustment
CST6CDT,M3.2.0,M11.1.0 -- UTC-6 with automatic DST adjustment
EST5EDT,M3.2.0,M11.1.0 -- UTC-5 with automatic DST adjustment
A complete list can be seen here in the second column:
https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv</description>
</string>
<group>
<name>Hub Configuration</name>
<description>Configuration settings for an OpenLCB Hub</description>
<int size='1'>
<name>Enable</name>
<description>Configures this node as an OpenLCB hub which can accept connections from other nodes.
NOTE: This may cause some instability as the number of connected nodes increases.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='2'>
<name>Hub Listener Port</name>
<description>Defines the TCP/IP listener port this node will use when operating as a hub. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
<string size='64'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<group offset='6'/>
</group>
<group>
<name>Uplink Configuration</name>
<description>Configures how this node will connect to other nodes.</description>
<int size='1'>
<name>Enable</name>
<description>Enables connecting to an OpenLCB Hub. In some cases it may be desirable to disable the uplink, such as a CAN only configuration.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='64'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<string size='64'>
<name>IP Address</name>
<description>Enter the server IP address. Example: 192.168.0.55
Note: This will be used as a fallback when mDNS lookup is not successful.</description>
</string>
<int size='2'>
<name>Port Number</name>
<description>TCP port number of the server. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
</group>
<int size='1'>
<name>WiFi Power Savings Mode</name>
<description>When enabled this allows the ESP32 WiFi radio to use power savings mode which puts the radio to sleep except to receive beacon updates from the connected SSID. This should generally not need to be enabled unless you are powering the ESP32 from a battery.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>WiFi Transmit Power</name>
<description>WiFi Radio transmit power in dBm. This can be used to limit the WiFi range. This option generally does not need to be changed.
NOTE: Setting this option to a very low value can cause communication failures.</description>
<min>8</min>
<max>78</max>
<default>78</default>
<map><relation><property>8</property><value>2 dBm</value></relation><relation><property>20</property><value>5 dBm</value></relation><relation><property>28</property><value>7 dBm</value></relation><relation><property>34</property><value>8 dBm</value></relation><relation><property>44</property><value>11 dBm</value></relation><relation><property>52</property><value>13 dBm</value></relation><relation><property>56</property><value>14 dBm</value></relation><relation><property>60</property><value>15 dBm</value></relation><relation><property>66</property><value>16 dBm</value></relation><relation><property>72</property><value>18 dBm</value></relation><relation><property>78</property><value>20 dBm</value></relation></map>
</int>
<int size='1'>
<name>Wait for successful SSID connection</name>
<description>Enabling this option will cause the node to restart when there is a failure (or timeout) during the SSID connection process.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
</group>
<group replication='2'>
<name>H-Bridge Configuration</name>
<repname>H-Bridge</repname>
<string size='15'>
<name>Description</name>
<description>Track output description.</description>
</string>
<eventid>
<name>Short Detected</name>
<description>This event will be produced when a short has been detected on the track output.</description>
</eventid>
<eventid>
<name>Short Cleared</name>
<description>This event will be produced when a short has been cleared on the track output.</description>
</eventid>
<eventid>
<name>H-Bridge Shutdown</name>
<description>This event will be produced when the track output power has exceeded the safety threshold of the H-Bridge.</description>
</eventid>
<eventid>
<name>H-Bridge Shutdown Cleared</name>
<description>This event will be produced when the track output power has returned to safe levels.</description>
</eventid>
</group>
<group>
<name>Thermal Configuration</name>
<int size='1'>
<name>Warning Temperature</name>
<description>Temperature (in celsius) to use for thermal warning.</description>
<min>0</min>
<max>125</max>
<default>50</default>
</int>
<int size='1'>
<name>Shutdown Temperature</name>
<description>Temperature (in celsius) at which to shutdown active monitoring and switch to low-power mode until temperatures return to below the shutdown level.</description>
<min>0</min>
<max>125</max>
<default>80</default>
</int>
<eventid>
<name>Warning Temperature Exceeded</name>
<description>This event will be produced when the temperature has exceeded the warning temperature but is below the shutdown temperature.</description>
</eventid>
<eventid>
<name>Warning Temperature (cleared)</name>
<description>This event will be produced when the temperature has dropped below the warning temperature.
Note: This event will not be generated if the warning temperature has not been exceeded previously.</description>
</eventid>
<eventid>
<name>Shutdown Temperature Exceeded</name>
<description>This event will be produced when the temperature has exceeded the shutdown temperature or there is a failure in reading the temperature.</description>
</eventid>
<eventid>
<name>Shutdown (Cleared)</name>
<description>This event will be produced when the temperature has dropped below the shutdown temperature.
Note: This event will not be generated if the shutdown temperature has not been exceeded previously.</description>
</eventid>
</group>
</segment>
<segment space='253'>
<name>Version information</name>
<int size='1'>
<name>ACDI User Data version</name>
<description>Set to 2 and do not change.</description>
</int>
</segment>
</cdi>
)xmlpayload";
  extern const size_t CDI_SIZE;
  const size_t CDI_SIZE = sizeof(CDI_DATA);
  extern const uint16_t CDI_EVENT_OFFSETS[] =
  {
    827, 835, 843, 851, 874, 882, 890, 898, 908, 916, 924, 932, 0
  };
}

// when the command station starts up the first time the config is blank
// and needs to be reset to factory settings. This class being declared here
// takes care of that.
class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
  UpdateAction apply_configuration(int fd, bool initial_load,
                                    BarrierNotifiable *done) override
  {
    AutoNotify n(done);
    return UPDATED;
  }

  void factory_reset(int fd) override
  {
    LOG(VERBOSE, "ESP32 Command Station factory_reset(%d) triggered.", fd);
    cfg.userinfo().name().write(fd, "ESP32 Command Station");
    cfg.userinfo().description().write(fd, "");
  }
};

void init_webserver();

uninitialized<FileSystemManager> fs;
uninitialized<esp32cs::LCCStackManager> stackManager;
uninitialized<esp32cs::LCCWiFiManager> wifiManager;
#if CONFIG_THERMALMONITOR
uninitialized<esp32cs::ThermalMonitorFlow> thermal_monitor;
#endif
#if !CONFIG_DISPLAY_TYPE_NONE
uninitialized<StatusDisplay> statusDisplay;
#endif
uninitialized<MDNS> mDNS;
uninitialized<http::Httpd> httpd;
uninitialized<TurnoutManager> turnoutManager;
#if CONFIG_HC12
uninitialized<esp32cs::HC12Radio> hc12;
#endif
#if CONFIG_STATUS_LED
uninitialized<StatusLED> statusLED;
#endif
uninitialized<OTAMonitorFlow> ota;
uninitialized<FactoryResetHelper> resetHelper;
uninitialized<openlcb::TrainService> trainService;
uninitialized<esp32cs::Esp32TrainDatabase> trainDb;
uninitialized<commandstation::AllTrainNodes> trainNodes;
uninitialized<FreeRTOSTaskMonitor> taskMon;

extern "C" void enter_bootloader()
{
    node_config_t config;
    if (load_config(&config) != ESP_OK)
    {
        default_config(&config);
    }
    config.bootloader_req = true;
    save_config(&config);
    LOG(INFO, "[Bootloader] Rebooting into bootloader");
    reboot();
}

static const char * const reset_reasons[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // SW_RESET                 3
    "watchdog reset (legacy)",  // OWDT_RESET               4
    "deep sleep reset",         // DEEPSLEEP_RESET          5
    "reset (SLC)",              // SDIO_RESET               6
    "watchdog reset (group0)",  // TG0WDT_SYS_RESET         7
    "watchdog reset (group1)",  // TG1WDT_SYS_RESET         8
    "RTC system reset",         // RTCWDT_SYS_RESET         9
    "Intrusion test reset",     // INTRUSION_RESET          10
    "WDT Timer group reset",    // TGWDT_CPU_RESET          11
    "software reset (CPU)",     // SW_CPU_RESET             12
    "RTC WDT reset",            // RTCWDT_CPU_RESET         13
    "software reset (CPU)",     // EXT_CPU_RESET            14
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   15
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         16
};

void start_bootloader_stack(uint64_t id);

extern "C" void app_main()
{
  esp_log_level_set("*", ESP_LOG_ERROR);
  // capture the reason for the CPU reset
  uint8_t reset_reason = rtc_get_reset_reason(PRO_CPU_NUM);
  uint8_t orig_reset_reason = reset_reason;
  // Ensure the reset reason it within bounds.
  if (reset_reason > ARRAYSIZE(reset_reasons))
  {
      reset_reason = 0;
  }

  const esp_app_desc_t *app_data = esp_ota_get_app_description();
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  LOG(INFO, "\n\nESP32 Command Station %s (%s) starting up (%d:%s)..."
    , SNIP_SW_VERSION, app_data->version, reset_reason
    , reset_reasons[reset_reason]);
  LOG(INFO
    , "[SoC] model:%s, rev:%d, cores:%d, flash:%s, WiFi:%s, BLE:%s, BT:%s"
    , chip_info.model == CHIP_ESP32 ? "ESP32" :
      chip_info.model == CHIP_ESP32S2 ? "ESP32-S2" : "unknown"
    , chip_info.revision, chip_info.cores
    , chip_info.features & CHIP_FEATURE_EMB_FLASH ? "Yes" : "No"
    , chip_info.features & CHIP_FEATURE_WIFI_BGN ? "Yes" : "No"
    , chip_info.features & CHIP_FEATURE_BLE ? "Yes" : "No"
    , chip_info.features & CHIP_FEATURE_BT ? "Yes" : "No");
  LOG(INFO, "[SoC] Heap: %.2fkB / %.2fKb"
    , heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f
    , heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024.0f);
  LOG(INFO, "Compiled on %s %s using IDF %s", app_data->date, app_data->time
    , app_data->idf_ver);
  LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);
  LOG(INFO, "ESP32 Command Station uses the OpenMRN library\n"
            "Copyright (c) 2019-2021, OpenMRN\n"
            "All rights reserved.");
  if (reset_reason != orig_reset_reason)
  {
    LOG(WARNING, "Reset reason mismatch: %d vs %d", reset_reason
      , orig_reset_reason);
  }
  nvs_init();
  // load non-CDI based config from NVS.
  bool factory_reset = false;
  node_config_t config;
  if (load_config(&config) != ESP_OK)
  {
    default_config(&config);
    factory_reset = true;
  }

  bool run_bootloader = false;
  bool config_updated = false;
  // Check for and reset factory reset flag.
  if (config.force_reset)
  {
    factory_reset = true;
    config.force_reset = false;
    config_updated = true;
  }

  if (config.bootloader_req)
  {
    run_bootloader = true;
    // reset the flag so we start in normal operating mode next time.
    config.bootloader_req = false;
    config_updated = true;
  }

  if (config_updated)
  {
    save_config(&config);
  }

  dump_config(&config);

#if CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
  if (run_bootloader)
  {
    start_bootloader_stack(config.node_id);
  }
  else
#else
  // silence unused variable warning
  (void)run_bootloader;
#endif
  {
    // Initialize the FileSystemManager, this manages the underlying persistent
    // filesystem. This may also trigger a factory reset if the reset pin is
    // shorted to GND or the marker file is present.
    fs.emplace();
    // Configure ADC1 up front to use 12 bit (0-4095) as we use it for all
    // monitored h-bridges.
    LOG(INFO, "[ADC] Configure 12-bit ADC resolution");
    adc1_config_width(ADC_WIDTH_BIT_12);

    stackManager.emplace(cfg, config.node_id, factory_reset);
    wifiManager.emplace(stackManager->stack(), cfg);
    auto stack = stackManager->stack();

#if CONFIG_THERMALMONITOR
    thermal_monitor.emplace(stackManager->service(), stackManager->node()
                          , cfg.seg().thermal()
                          , (adc1_channel_t)CONFIG_THERMALMONITOR_ADC);
#endif // CONFIG_THERMALMONITOR

#if !CONFIG_DISPLAY_TYPE_NONE
    // Initialize the status display module (dependency of WiFi)
    statusDisplay.emplace(stack, stackManager->service());
#endif // !CONFIG_DISPLAY_TYPE_NONE

    // Initialize the DCC VFS adapter, this will also initialize the DCC signal
    // generation code.
    esp32cs::init_dcc(stackManager->node(), stackManager->service()
                    , cfg.seg().hbridge().entry(esp32cs::OPS_CDI_TRACK_OUTPUT_IDX)
                    , cfg.seg().hbridge().entry(esp32cs::PROG_CDI_TRACK_OUTPUT_IDX));

    // Initialize the Http server and mDNS instance
    mDNS.emplace();
    httpd.emplace(mDNS.get_mutable());
    init_webserver();

#if CONFIG_NEXTION
    // Initialize the Nextion module (dependency of WiFi)
    LOG(INFO, "[Config] Enabling Nextion module");
    nextionInterfaceInit(stackManager.service());
#endif // CONFIG_NEXTION

#if CONFIG_JMRI
    init_jmri_interface();
#endif // CONFIG_JMRI

    // Initialize the turnout manager and register it with the LCC stack to
    // process accessories packets.
    turnoutManager.emplace(stackManager->node(), stackManager->service());

#if CONFIG_GPIO_OUTPUTS
    LOG(INFO, "[Config] Enabling GPIO Outputs");
    OutputManager::init();
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_GPIO_SENSORS
    LOG(INFO, "[Config] Enabling GPIO Inputs");
    SensorManager::init();
    RemoteSensorManager::init();
#if CONFIG_GPIO_S88
    S88BusManager s88(stackManager.node());
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS

#if CONFIG_HC12
    hc12.emplace(stackManager->service(), (uart_port_t)CONFIG_HC12_UART
              , (gpio_num_t)CONFIG_HC12_RX_PIN
              , (gpio_num_t)CONFIG_HC12_TX_PIN));
#endif // CONFIG_HC12

#if CONFIG_STATUS_LED
    statusLED.emplace(stackManager->service());
#endif // CONFIG_STATUS_LED

    // Initialize the OTA monitor
    ota.emplace(stackManager->service());

    // Initialize the factory reset helper for the CS.
    resetHelper.emplace();

    // Starts the OpenMRN stack, this needs to be done *AFTER* all other LCC
    // dependent components as it will initiate configuration load and factory
    // reset calls.
    stackManager->start(fs->is_sd());

    // Initialize the DCC++ protocol adapter
    DCCPPProtocolHandler::init();

    // Initialize the Traction Protocol support
    trainService.emplace(stack->iface());

    // Initialize the train database
    trainDb.emplace(stack);

    // Initialize the Train Search and Train Manager.
    trainNodes.emplace(trainDb.get_mutable(), trainService.get_mutable()
                    , stackManager->info_flow()
                    , stackManager->memory_config_handler()
                    , trainDb->get_train_cdi(), trainDb->get_temp_train_cdi());

    // Task Monitor, periodically dumps runtime state to STDOUT.
    LOG(VERBOSE, "Starting FreeRTOS Task Monitor");
    taskMon.emplace(stackManager->service());

    LOG(INFO, "\n\nESP32 Command Station Startup complete!\n");
#if !CONFIG_DISPLAY_TYPE_NONE
    Singleton<StatusDisplay>::instance()->status("ESP32-CS Started");
#endif // !CONFIG_DISPLAY_TYPE_NONE

    // Start the OpenMRN stack executor
    stack->start_executor_thread("OpenMRN"
                              , config_arduino_openmrn_task_priority()
                              , config_arduino_openmrn_stack_size());
  }

  // At this point the OpenMRN stack is running in it's own task and we can
  // safely exit from this one. We do not need to cleanup as that will be
  // handled automatically by ESP-IDF.
}

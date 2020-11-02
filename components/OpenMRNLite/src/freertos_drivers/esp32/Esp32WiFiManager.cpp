/** \copyright
 * Copyright (c) 2019, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file Esp32WiFiManager.cxx
 *
 * ESP32 WiFi Manager
 *
 * @author Mike Dunston
 * @date 4 February 2019
 */

// Ensure we only compile this code for the ESP32
#ifdef ESP32

#include "Esp32WiFiManager.hxx"
#include "os/MDNS.hxx"
#include "utils/FdUtils.hxx"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include <lwip/dns.h>
#include <mdns.h>

// ESP-IDF v4+ has a slightly different directory structure to previous
// versions.
#ifdef ESP_IDF_VERSION
// ESP-IDF v4+
#include <esp32/rom/crc.h>
#include <esp_private/wifi.h>
#include <esp_event.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
#include <dhcpserver/dhcpserver.h>
#endif // IDF v4.1+
#else
// ESP-IDF v3.x
#include <esp_event_loop.h>
#include <esp_wifi_internal.h>
#include <rom/crc.h>
#endif // ESP_IDF_VERSION

using openlcb::NodeID;
using openlcb::SimpleCanStackBase;
using openlcb::SimpleStackBase;
using openlcb::TcpAutoAddress;
using openlcb::TcpClientConfig;
using openlcb::TcpClientDefaultParams;
using openlcb::TcpDefs;
using openlcb::TcpManualAddress;
using std::string;
using std::unique_ptr;

#ifndef ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL
/// Allows setting the log level for mDNS related log messages from 
/// @ref DefaultSocketClientParams.
#define ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL INFO
#endif

#ifndef ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL
/// Allows setting the log level for mDNS results in the @ref mdns_lookup
/// method.
#define ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL INFO
#endif

// Start of global namespace block.

// These must be declared *OUTSIDE* the openmrn_arduino namespace in order to
// be visible in the MDNS.cxx code.

/// Advertises an mDNS service name. This is a hook point for the MDNS class
/// and is used as part of the Esp32 WiFi hub support.
void mdns_publish(const char *name, const char *service, uint16_t port);

/// Removes advertisement of an mDNS service name. This is not currently
/// exposed in the MDNS class but is supported on the ESP32.
void mdns_unpublish(const char *service);

/// Splits a service name since the ESP32 mDNS library requires the service
/// name and service protocol to be passed in individually.
///
/// @param service_name is the service name to be split.
/// @param protocol_name is the protocol portion of the service name.
///
/// Note: service_name *WILL* be modified by this call.
void split_mdns_service_name(string *service_name, string *protocol_name);

// End of global namespace block.

namespace openmrn_arduino
{

/// Stack size for the wifi_manager_task.
static constexpr uint32_t WIFI_TASK_STACK_SIZE = 2560L;

/// Interval at which to check the WiFi connection status.
static constexpr TickType_t WIFI_CONNECT_CHECK_INTERVAL = pdMS_TO_TICKS(5000);

/// Interval at which to check if the GcTcpHub has started or not.
static constexpr uint32_t HUB_STARTUP_DELAY_USEC = MSEC_TO_USEC(50);

/// Interval at which to check if the WiFi task has shutdown or not.
static constexpr uint32_t TASK_SHUTDOWN_DELAY_USEC = MSEC_TO_USEC(1);

/// Bit designator for wifi_status_event_group which indicates we are connected
/// to the SSID.
static constexpr int WIFI_CONNECTED_BIT = BIT0;

/// Bit designator for wifi_status_event_group which indicates we have an IPv4
/// address assigned.
static constexpr int WIFI_GOTIP_BIT = BIT1;

/// Allow up to 36 checks to see if we have connected to the SSID and
/// received an IPv4 address. This allows up to ~3 minutes for the entire
/// process to complete, in most cases this should be complete in under 30
/// seconds.
static constexpr uint8_t MAX_CONNECTION_CHECK_ATTEMPTS = 36;

/// This is the number of consecutive IP addresses which will be available in
/// the SoftAP DHCP server IP pool. These will be allocated immediately
/// following the SoftAP IP address (default is 192.168.4.1). Default number to
/// reserve is 48 IP addresses. Only four stations can be connected to the
/// ESP32 SoftAP at any single time.
static constexpr uint8_t SOFTAP_IP_RESERVATION_BLOCK_SIZE = 48;

/// The esp_wifi API only allows up to 78 0.25 dBm increments, but the CDI will
/// allow up to 79 to allow full range usage.
static constexpr int8_t MAX_WIFI_TX_POWER_API_LIMIT = 78;

// With this constructor being used the Esp32WiFiManager will manage the
// WiFi connection, mDNS system and the hostname of the ESP32.
Esp32WiFiManager::Esp32WiFiManager(const char *ssid
                                 , const char *password
                                 , SimpleStackBase *stack
                                 , const WiFiConfiguration &cfg
                                 , const char *hostname_prefix
                                 , wifi_mode_t wifi_mode
                                 , ESP32_ADAPTER_IP_INFO_TYPE *station_static_ip
                                 , ip_addr_t primary_dns_server
                                 , uint8_t soft_ap_channel
                                 , wifi_auth_mode_t soft_ap_auth
                                 , const char *soft_ap_password
                                 , ESP32_ADAPTER_IP_INFO_TYPE *softap_static_ip)
    : DefaultConfigUpdateListener()
    , hostname_(hostname_prefix)
    , ssid_(ssid)
    , password_(password)
    , cfg_(cfg)
    , stack_(stack)
    , wifiMode_(wifi_mode)
    , stationStaticIP_(station_static_ip)
    , primaryDNSAddress_(primary_dns_server)
    , softAPChannel_(soft_ap_channel)
    , softAPAuthMode_(soft_ap_auth)
    , softAPPassword_(soft_ap_password ? soft_ap_password : password)
    , softAPStaticIP_(softap_static_ip)
{
    // Extend the capacity of the hostname to make space for the node-id and
    // underscore.
    hostname_.reserve(ESP32_MAX_HOSTNAME_LENGTH);

    // Generate the hostname for the ESP32 based on the provided node id.
    // node_id : 0x050101011425
    // hostname_ : esp32_050101011425
    NodeID node_id = stack_->node()->node_id();
    hostname_.append(uint64_to_string_hex(node_id, 0));

    // The maximum length hostname for the ESP32 is 32 characters so truncate
    // when necessary. Reference to length limitation:
    // https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
    if (hostname_.length() > ESP32_MAX_HOSTNAME_LENGTH)
    {
        LOG(WARNING, "ESP32 hostname is too long, original hostname: %s",
            hostname_.c_str());
        hostname_.resize(ESP32_MAX_HOSTNAME_LENGTH);
        LOG(WARNING, "truncated hostname: %s", hostname_.c_str());
    }

    // Release any extra capacity allocated for the hostname.
    hostname_.shrink_to_fit();
}


// destructor to ensure cleanup of owned resources
Esp32WiFiManager::~Esp32WiFiManager()
{
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    // Remove our event listeners from the event loop, note that we do not stop
    // the event loop.
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID
                               , &Esp32WiFiManager::process_idf_event);
    esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID
                               , &Esp32WiFiManager::process_idf_event);
#else
    // Disconnect from the event loop to prevent a possible null deref.
    esp_event_loop_set_cb(nullptr, nullptr);
#endif // IDF v4.1+

    // set flag to shutdown WiFi background task and wake it up
    shutdownRequested_ = true;
    xTaskNotifyGive(wifiTaskHandle_);

    // wait for WiFi background task shutdown to complete
    while (shutdownRequested_)
    {
        usleep(TASK_SHUTDOWN_DELAY_USEC);
    }

    // cleanup event group
    vEventGroupDelete(wifiStatusEventGroup_);

    // cleanup internal vectors/maps
    ssidScanResults_.clear();
    mdnsDeferredPublish_.clear();
}

ConfigUpdateListener::UpdateAction Esp32WiFiManager::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    AutoNotify n(done);
    LOG(VERBOSE, "Esp32WiFiManager::apply_configuration(%d, %d)", fd,
        initial_load);

    // Cache the fd for later use by the wifi background task.
    configFd_ = fd;
    initialConfigLoad_ = initial_load;

    // Load the CDI entry into memory to do an CRC-32 check against our last
    // loaded configuration so we can avoid reloading configuration when there
    // are no interesting changes.
    unique_ptr<uint8_t[]> crcbuf(new uint8_t[cfg_.size()]);

    // If we are unable to seek to the right position in the persistent storage
    // give up and request a reboot.
    if (lseek(fd, cfg_.offset(), SEEK_SET) != cfg_.offset())
    {
        LOG_ERROR("lseek failed to reset fd offset, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // Read the full configuration to the buffer for crc check.
    FdUtils::repeated_read(fd, crcbuf.get(), cfg_.size());

    // Calculate CRC-32 from the loaded buffer.
    uint32_t configCrc32 = crc32_le(0, crcbuf.get(), cfg_.size());
    LOG(VERBOSE, "existing config CRC-32: \"%s\", new CRC-32: \"%s\"",
        integer_to_string(configCrc32_, 0).c_str(),
        integer_to_string(configCrc32, 0).c_str());

    // update local cache of config settings before waking the background task.
    uplinkEnabled_ = CDI_READ_TRIM_DEFAULT(cfg_.uplink().enable, fd);
    uplinkManualHost_ = cfg_.uplink().manual().ip_address().read(fd);
    uplinkManualPort_ =
        CDI_READ_TRIM_DEFAULT(cfg_.uplink().manual().port, fd);
    uplinkAutoService_ = cfg_.uplink().automatic().service_name().read(fd);
    enableRadioSleep_ = CDI_READ_TRIM_DEFAULT(cfg_.sleep, fd);
#if defined(CONFIG_IDF_TARGET_ESP32)
    enableHub_ = CDI_READ_TRIM_DEFAULT(cfg_.hub().enable, fd);
    hubServiceName_ = cfg_.hub().service_name().read(fd);
    hubPort_ = CDI_READ_TRIM_DEFAULT(cfg_.hub().port, fd);
#endif // CONFIG_IDF_TARGET_ESP32
    // Read the desired TX power level from the CDI with a bounds check to
    // ensure it is does not exceed 78 (max supported by underlying API).
    wifiTXPower_ =
        std::min(MAX_WIFI_TX_POWER_API_LIMIT,
                 (int8_t)CDI_READ_TRIM_DEFAULT(cfg_.tx_power, fd));

    // if this is not the initial loading of the CDI entry check the CRC-32
    // value and trigger a configuration reload if necessary.
    if (!initial_load)
    {
        if (configCrc32 != configCrc32_)
        {
            configReloadRequested_ = true;
            // If a configuration change has been detected, wake up the
            // wifi_manager_task so it can consume the change prior to the next
            // wake up interval.
            xTaskNotifyGive(wifiTaskHandle_);
        }
    }
    else
    {
        // This is the initial loading of the CDI entry, start the background
        // task that will manage the node's WiFi connection(s).
        start_wifi_task();
    }

    // Store the calculated CRC-32 for future use when the apply_configuration
    // method is called to detect any configuration changes.
    configCrc32_ = configCrc32;

    // Inform the caller that the configuration has been updated as the wifi
    // task will reload the configuration as part of it's next wake up cycle.
    return ConfigUpdateListener::UpdateAction::UPDATED;
}

// Factory reset handler for the WiFiConfiguration CDI entry.
void Esp32WiFiManager::factory_reset(int fd)
{
    LOG(VERBOSE, "Esp32WiFiManager::factory_reset(%d)", fd);

    // General WiFi configuration settings.
    CDI_FACTORY_RESET(cfg_.sleep);
    CDI_FACTORY_RESET(cfg_.tx_power);

#if defined(CONFIG_IDF_TARGET_ESP32)
    // Hub specific configuration settings.
    CDI_FACTORY_RESET(cfg_.hub().enable);
    CDI_FACTORY_RESET(cfg_.hub().port);
    cfg_.hub().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
#endif // CONFIG_IDF_TARGET_ESP32

    // Node link configuration settings.
    CDI_FACTORY_RESET(cfg_.uplink().enable);

    // Node link manual configuration settings.
    cfg_.uplink().manual().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.uplink().manual().port);

    // Node link automatic configuration settings.
    cfg_.uplink().automatic().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
}

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
void Esp32WiFiManager::process_idf_event(void *arg, esp_event_base_t event_base
                                       , int32_t event_id, void *event_data)
{
    Esp32WiFiManager *wifi = Singleton<Esp32WiFiManager>::instance();
    LOG(VERBOSE, "Esp32WiFiManager::process_idf_event(%s, %d, %p)", event_base
      , event_id, event_data);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START &&
        (wifi->wifiMode_ == WIFI_MODE_APSTA ||
         wifi->wifiMode_ == WIFI_MODE_STA))
    {
        wifi->on_station_started();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        wifi->on_station_connected();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi->on_station_disconnected(
            static_cast<wifi_event_sta_disconnected_t *>(event_data)->reason);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START)
    {
        wifi->on_softap_start();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP)
    {
        wifi->on_softap_stop();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi->on_softap_station_connected(
            *(static_cast<wifi_event_ap_staconnected_t *>(event_data)));
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi->on_softap_station_disconnected(
            *(static_cast<wifi_event_ap_stadisconnected_t *>(event_data)));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        wifi->on_wifi_scan_completed(
            *(static_cast<wifi_event_sta_scan_done_t *>(event_data)));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *data = static_cast<ip_event_got_ip_t *>(event_data);
        wifi->on_station_ip_assigned(htonl(data->ip_info.ip.addr));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
    {
        wifi->on_station_ip_lost();
    }
}

#else
// Processes a WiFi system event
esp_err_t Esp32WiFiManager::process_wifi_event(void *ctx, system_event_t *event)
{
    Esp32WiFiManager *wifi = Singleton<Esp32WiFiManager>::instance();
    LOG(VERBOSE, "Esp32WiFiManager::process_wifi_event(%d)", event->event_id);

    // We only are interested in this event if we are managing the
    // WiFi and MDNS systems and our mode includes STATION.
    if (event->event_id == SYSTEM_EVENT_STA_START &&
        (wifi->wifiMode_ == WIFI_MODE_APSTA ||
         wifi->wifiMode_ == WIFI_MODE_STA))
    {
        wifi->on_station_started();
    }
    else if (event->event_id == SYSTEM_EVENT_STA_CONNECTED)
    {
        wifi->on_station_connected();
    }
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED)
    {
        wifi->on_station_disconnected(event->event_info.disconnected.reason);
    }
    else if (event->event_id == SYSTEM_EVENT_STA_GOT_IP)
    {
        wifi->on_station_ip_assigned(
            htonl(event->event_info.got_ip.ip_info.ip.addr));
    }
    else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP)
    {
        wifi->on_station_ip_lost();
    }
    else if (event->event_id == SYSTEM_EVENT_AP_START)
    {
        wifi->on_softap_start();
    }
    else if (event->event_id == SYSTEM_EVENT_AP_STOP)
    {
        wifi->on_softap_stop();
    }
    else if (event->event_id == SYSTEM_EVENT_AP_STACONNECTED)
    {
        wifi->on_softap_station_connected(event->event_info.sta_connected);
    }
    else if (event->event_id == SYSTEM_EVENT_AP_STADISCONNECTED)
    {
        wifi->on_softap_station_disconnected(
            event->event_info.sta_disconnected);
    }
    else if (event->event_id == SYSTEM_EVENT_SCAN_DONE)
    {
        wifi->on_wifi_scan_completed(event->event_info.scan_done);
    }

    return ESP_OK;
}
#endif

// Set configuration flag that enables the verbose logging.
// NOTE: this should be called as early as possible to ensure proper logging
// from all esp-wifi code paths.
void Esp32WiFiManager::enable_verbose_logging()
{
    verboseLogging_ = true;
    enable_esp_wifi_logging();
}

// Set configuration flag controlling SSID connection checking behavior.
void Esp32WiFiManager::wait_for_ssid_connect(bool enable)
{
    waitForStationConnect_ = enable;
}

// Adds a callback which will be called when the network is up.
void Esp32WiFiManager::register_network_up_callback(
    esp32_network_up_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkUpCallbacks_.push_back(callback);
}

// Adds a callback which will be called when the network is down.
void Esp32WiFiManager::register_network_down_callback(
    esp32_network_down_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkDownCallbacks_.push_back(callback);
}

// Adds a callback which will be called when the network is initializing.
void Esp32WiFiManager::register_network_init_callback(
    esp32_network_init_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkInitCallbacks_.push_back(callback);
}

// If the Esp32WiFiManager is setup to manage the WiFi system, the following
// steps are executed:
// 1) Start the TCP/IP adapter.
// 2) Hook into the ESP event loop so we receive WiFi events.
// 3) Initialize the WiFi system.
// 4) Set the WiFi mode to STATION (WIFI_STA)
// 5) Configure the WiFi system to store parameters only in memory to avoid
// potential corruption of entries in NVS.
// 6) Configure the WiFi system for SSID/PW.
// 7) Set the hostname based on the generated hostname.
// 8) Connect to WiFi and wait for IP assignment.
// 9) Verify that we connected and received a IP address, if not log a FATAL
// message and give up.
void Esp32WiFiManager::start_wifi_system()
{
    // Create the event group used for tracking connected/disconnected status.
    // This is used internally regardless of if we manage the rest of the WiFi
    // or mDNS systems.
    wifiStatusEventGroup_ = xEventGroupCreate();

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    // create default interfaces for station and SoftAP, ethernet is not used
    // today.
    ESP_ERROR_CHECK(esp_netif_init());

    // create the event loop.
    esp_err_t err = esp_event_loop_create_default();

    // The esp_event_loop_create_default() method will return either ESP_OK if
    // the event loop was created or ESP_ERR_INVALID_STATE if one already
    // exists.
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        LOG(FATAL, "[WiFi] Failed to initialize the default event loop: %s"
          , esp_err_to_name(err));
    }

    esp_netifs[ESP_IF_WIFI_STA] = esp_netif_create_default_wifi_sta();
    esp_netifs[ESP_IF_WIFI_AP] = esp_netif_create_default_wifi_ap();

    // Connect our event listeners.
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID
                             , &Esp32WiFiManager::process_idf_event, nullptr);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID
                             , &Esp32WiFiManager::process_idf_event, nullptr);
#else
    // Initialize the TCP/IP adapter stack.
    LOG(INFO, "[WiFi] Starting TCP/IP stack");
    tcpip_adapter_init();

    // Install event loop handler.
    ESP_ERROR_CHECK(esp_event_loop_init(&Esp32WiFiManager::process_wifi_event, nullptr));

#endif // ESP-IDF v4.1+

    // Start the WiFi adapter.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    LOG(INFO, "[WiFi] Initializing WiFi stack");

    // Disable NVS storage for the WiFi driver
    cfg.nvs_enable = false;

    // override the defaults coming from arduino-esp32, the ones below improve
    // throughput and stability of TCP/IP, for more info on these values, see:
    // https://github.com/espressif/arduino-esp32/issues/2899 and
    // https://github.com/espressif/arduino-esp32/pull/2912
    //
    // Note: these numbers are slightly higher to allow compatibility with the
    // WROVER chip and WROOM-32 chip. The increase results in ~2kb less heap
    // at runtime.
    //
    // These do not require recompilation of arduino-esp32 code as these are
    // used in the WIFI_INIT_CONFIG_DEFAULT macro, they simply need to be redefined.
    cfg.static_rx_buf_num = 16;
    cfg.dynamic_rx_buf_num = 32;
    cfg.rx_ba_win = 16;

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (verboseLogging_)
    {
        enable_esp_wifi_logging();
    }

    wifi_mode_t requested_wifi_mode = wifiMode_;
    if (wifiMode_ == WIFI_MODE_AP)
    {
      // override the wifi mode from AP only to AP+STA so we can perform wifi
      // scans on demand.
      requested_wifi_mode = WIFI_MODE_APSTA;
    }
    // Set the requested WiFi mode.
    ESP_ERROR_CHECK(esp_wifi_set_mode(requested_wifi_mode));

    // This disables storage of SSID details in NVS which has been shown to be
    // problematic at times for the ESP32, it is safer to always pass fresh
    // config and have the ESP32 resolve the details at runtime rather than
    // use a cached set from NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    // If we want to host a SoftAP configure it now.
    if (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_AP)
    {
        wifi_config_t conf;
        bzero(&conf, sizeof(wifi_config_t));
        conf.ap.authmode = softAPAuthMode_;
        conf.ap.beacon_interval = 100;
        conf.ap.channel = softAPChannel_;
        conf.ap.max_connection = 4;
        if (wifiMode_ == WIFI_MODE_AP)
        {
            // Configure the SSID for the Soft AP based on the SSID passed to
            // the Esp32WiFiManager constructor.
            strcpy(reinterpret_cast<char *>(conf.ap.ssid), ssid_);
        }
        else
        {
            // Configure the SSID for the Soft AP based on the generated
            // hostname when operating in WIFI_MODE_APSTA mode.
            strcpy(reinterpret_cast<char *>(conf.ap.ssid), hostname_.c_str());
        }
        
        if (password_ && softAPAuthMode_ != WIFI_AUTH_OPEN)
        {
            strcpy(reinterpret_cast<char *>(conf.ap.password), password_);
        }

        LOG(INFO, "[WiFi] Configuring SoftAP (SSID: %s)", conf.ap.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &conf));
    }

    // If we need to connect to an SSID, configure it now.
    if (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_STA)
    {
        // Configure the SSID details for the station based on the SSID and
        // password provided to the Esp32WiFiManager constructor.
        wifi_config_t conf;
        bzero(&conf, sizeof(wifi_config_t));
        strcpy(reinterpret_cast<char *>(conf.sta.ssid), ssid_);
        if (password_)
        {
            strcpy(reinterpret_cast<char *>(conf.sta.password), password_);
        }

        LOG(INFO, "[WiFi] Configuring Station (SSID: %s)", conf.sta.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &conf));
    }

    // Start the WiFi stack. This will start the SoftAP and/or connect to the
    // SSID based on the configuration set above.
    LOG(INFO, "[WiFi] Starting WiFi stack");
    ESP_ERROR_CHECK(esp_wifi_start());

    // If we need the STATION interface *AND* configured to wait until 
    // successfully connected to the SSID this code block will wait for up to
    // approximately three minutes for an IP address to be assigned. In most
    // cases this completes in under thirty seconds. If there is a connection
    // failure the esp32 will be restarted via a FATAL error being logged.
    if (waitForStationConnect_ &&
       (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_STA))
    {
        uint8_t attempt = 0;
        EventBits_t bits = 0;
        uint32_t bit_mask = WIFI_CONNECTED_BIT;
        while (++attempt <= MAX_CONNECTION_CHECK_ATTEMPTS)
        {
            // If we have connected to the SSID we then are waiting for IP
            // address.
            if (bits & WIFI_CONNECTED_BIT)
            {
                LOG(INFO, "[IPv4] [%d/%d] Waiting for IP address assignment.",
                    attempt, MAX_CONNECTION_CHECK_ATTEMPTS);
            }
            else
            {
                // Waiting for SSID connection
                LOG(INFO, "[WiFi] [%d/%d] Waiting for SSID connection.",
                    attempt, MAX_CONNECTION_CHECK_ATTEMPTS);
            }
            bits = xEventGroupWaitBits(wifiStatusEventGroup_,
                bit_mask, // bits we are interested in
                pdFALSE,  // clear on exit
                pdTRUE,   // wait for all bits
                WIFI_CONNECT_CHECK_INTERVAL);
            // Check if have connected to the SSID
            if (bits & WIFI_CONNECTED_BIT)
            {
                // Since we have connected to the SSID we now need to track
                // that we get an IP.
                bit_mask |= WIFI_GOTIP_BIT;
            }
            // Check if we have received an IP.
            if (bits & WIFI_GOTIP_BIT)
            {
                break;
            }
        }

        // Check if we successfully connected or not. If not, force a reboot.
        if ((bits & WIFI_CONNECTED_BIT) != WIFI_CONNECTED_BIT)
        {
            LOG(FATAL, "[WiFi] Failed to connect to SSID: %s.", ssid_);
        }

        // Check if we successfully connected or not. If not, force a reboot.
        if ((bits & WIFI_GOTIP_BIT) != WIFI_GOTIP_BIT)
        {
            LOG(FATAL, "[IPv4] Timeout waiting for an IP.");
        }
    }
}

// Starts a background task for the Esp32WiFiManager.
void Esp32WiFiManager::start_wifi_task()
{
    LOG(INFO, "[WiFi] Starting WiFi Manager task");
    os_thread_create(&wifiTaskHandle_, "Esp32WiFiMgr",
        config_arduino_openmrn_task_priority(),
        WIFI_TASK_STACK_SIZE, wifi_manager_task, this);
}

// Background task for the Esp32WiFiManager. This handles all outbound
// connection attempts, configuration loading and making this node as a hub.
void *Esp32WiFiManager::wifi_manager_task(void *param)
{
    Esp32WiFiManager *wifi = static_cast<Esp32WiFiManager *>(param);

    // Start the WiFi system before proceeding with remaining tasks.
    wifi->start_wifi_system();

    while (!wifi->shutdownRequested_)
    {
        EventBits_t bits = xEventGroupGetBits(wifi->wifiStatusEventGroup_);
        if (bits & WIFI_GOTIP_BIT)
        {
            // If we do not have not an uplink connection force a config reload
            // to start the connection process.
            if (!wifi->uplink_)
            {
                wifi->configReloadRequested_ = true;
            }
        }
        else
        {
            // Since we do not have an IP address we need to shutdown any
            // active connections since they will be invalid until a new IP
            // has been provisioned.
            wifi->stop_hub();
            wifi->stop_uplink();

            // Make sure we don't try and reload configuration since we can't
            // create outbound connections at this time.
            wifi->configReloadRequested_ = false;
        }

        if (wifi->shutdownRequested_)
        {
            LOG(INFO, "[WiFi] Shutdown requested, stopping background thread.");
            break;
        }

        // Check if there are configuration changes to pick up.
        if (wifi->configReloadRequested_ || wifi->initialConfigLoad_)
        {
            // Since we are loading configuration data, shutdown the hub and
            // uplink if created previously.
            wifi->stop_hub();
            wifi->stop_uplink();

            if (wifi->enableRadioSleep_)
            {
                // When sleep is enabled this will trigger the WiFi system to
                // only wake up every DTIM period to receive beacon updates.
                // no data loss is expected for this setting but it does delay
                // receiption until the DTIM period.
                ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
            }
            else
            {
                // When sleep is disabled the WiFi radio will always be active.
                // This will increase power consumption of the ESP32 but it
                // will result in a more reliable behavior when the ESP32 is
                // connected to an always-on power supply (ie: not a battery).
                ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
            }

            // If this is not the initial loading of the configuration, set the
            // maximum transmit power. Otherwise it will be handled as part of
            // the initial configuration of the Station or SoftAP interface.
            if (!wifi->initialConfigLoad_)
            {
                wifi->configure_wifi_max_tx_power();
            }
#if defined(CONFIG_IDF_TARGET_ESP32)
            if (wifi->enableHub_)
            {
                // Since hub mode is enabled start the hub creation process.
                wifi->start_hub();
            }
#endif // CONFIG_IDF_TARGET_ESP32
            if (wifi->wifiMode_ != WIFI_MODE_AP)
            {
                // Start the uplink connection process in the background.
                wifi->start_uplink();
            }
            wifi->configReloadRequested_ = false;
            wifi->initialConfigLoad_ = false;
        }

        // Sleep until we are woken up again for configuration update or WiFi
        // event.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    // Stop the hub and uplink (if they are active)
    wifi->stop_hub();
    wifi->stop_uplink();

    // reset flag to indicate we have shutdown
    wifi->shutdownRequested_ = false;

    return nullptr;
}

// Shuts down the hub listener (if enabled and running) for this node.
void Esp32WiFiManager::stop_hub()
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    if (hub_)
    {
        mdns_unpublish(hubServiceName_);
        LOG(INFO, "[Hub] Shutting down TCP/IP listener");
        hub_.reset(nullptr);
    }
#endif // CONFIG_IDF_TARGET_ESP32
}

// Creates a hub listener for this node after loading configuration details.
void Esp32WiFiManager::start_hub()
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    LOG(INFO, "[Hub] Starting TCP/IP listener on port %d", hubPort_);
    // TODO: find a better solution for this that does not require a cast and
    // will work with the TCP stack.
    openlcb::SimpleCanStackBase *canStack =
        static_cast<openlcb::SimpleCanStackBase *>(stack_);
    hub_.reset(new GcTcpHub(canStack->can_hub(), hubPort_));

    // wait for the hub to complete it's startup tasks
    while (!hub_->is_started())
    {
        usleep(HUB_STARTUP_DELAY_USEC);
    }
    mdns_publish(hubServiceName_, hubPort_);
#endif // CONFIG_IDF_TARGET_ESP32
}

// Disconnects and shuts down the uplink connector socket if running.
void Esp32WiFiManager::stop_uplink()
{
    if (uplink_)
    {
        LOG(INFO, "[Uplink] Disconnecting from uplink.");
        uplink_->shutdown();
        uplink_.reset(nullptr);
    }
}

// Creates an uplink connector socket that will automatically add the uplink to
// the node's hub.
void Esp32WiFiManager::start_uplink()
{
    if (!uplinkEnabled_)
    {
        return;
    }
    uplink_.reset(
        new SocketClient(
            stack_->service(), stack_->executor(), stack_->executor(),
            SocketClientParams::from_static_and_mdns(uplinkManualHost_,
                                                     uplinkManualPort_,
                                                     uplinkAutoService_),
            std::bind(&Esp32WiFiManager::on_uplink_created, this,
                      std::placeholders::_1, std::placeholders::_2)));
}

// Converts the passed fd into a GridConnect port and adds it to the stack.
void Esp32WiFiManager::on_uplink_created(int fd, Notifiable *on_exit)
{
    LOG(INFO, "[Uplink] Connected to hub, configuring GridConnect port.");

    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);

    // TODO: find a better solution for this that does not require a cast and
    // will work with the TCP stack.
    openlcb::SimpleCanStackBase *canStack =
        static_cast<openlcb::SimpleCanStackBase *>(stack_);

    // create the GridConnect port from the provided socket fd.
    create_gc_port_for_can_hub(canStack->can_hub(), fd, on_exit, use_select);

    // restart the stack to kick off alias allocation and send node init
    // packets.
    stack_->restart_stack();
}

// Enables the ESP-IDF wifi module logging at verbose level, will also set the
// sub-modules to verbose if they are available.
void Esp32WiFiManager::enable_esp_wifi_logging()
{
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);

// arduino-esp32 1.0.2 uses ESP-IDF 3.2 which does not have these two methods
// in the headers, they are only available in ESP-IDF 3.3.
#if defined(WIFI_LOG_SUBMODULE_ALL)
    esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);
    esp_wifi_internal_set_log_mod(
        WIFI_LOG_MODULE_ALL, WIFI_LOG_SUBMODULE_ALL, true);
#endif // WIFI_LOG_SUBMODULE_ALL
}

// Starts a background scan of SSIDs that can be seen by the ESP32.
void Esp32WiFiManager::start_ssid_scan(Notifiable *n)
{
    clear_ssid_scan_results();
    std::swap(ssidCompleteNotifiable_, n);
    // If there was a previous notifiable notify it now, there will be no
    // results but that should be fine since a new scan will be started.
    if (n)
    {
        n->notify();
    }
    // Start an active scan all channels, 120ms per channel (defaults)
    wifi_scan_config_t cfg;
    bzero(&cfg, sizeof(wifi_scan_config_t));
    // The boolean flag when set to false triggers an async scan.
    ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, false));
}

// Returns the number of SSIDs found in the last scan.
size_t Esp32WiFiManager::get_ssid_scan_result_count()
{
    OSMutexLock l(&ssidScanResultsLock_);
    return ssidScanResults_.size();
}

// Returns one SSID record from the last scan.
wifi_ap_record_t Esp32WiFiManager::get_ssid_scan_result(size_t index)
{
    OSMutexLock l(&ssidScanResultsLock_);
    wifi_ap_record_t record = wifi_ap_record_t();
    if (index < ssidScanResults_.size())
    {
        record = ssidScanResults_[index];
    }
    return record;
}

// Clears all cached SSID scan results.
void Esp32WiFiManager::clear_ssid_scan_results()
{
    OSMutexLock l(&ssidScanResultsLock_);
    ssidScanResults_.clear();
}

// Advertises a service via mDNS.
//
// If mDNS has not yet been initialized the data will be cached and replayed
// after mDNS has been initialized.
void Esp32WiFiManager::mdns_publish(string service, const uint16_t port)
{
    {
        OSMutexLock l(&mdnsInitLock_);
        if (!mdnsInitialized_)
        {
            // since mDNS has not been initialized, store this publish until
            // it has been initialized.
            mdnsDeferredPublish_[service] = port;
            return;
        }
    }

    // Schedule the publish to be done through the Executor since we may need
    // to retry it.
    stack_->executor()->add(new CallbackExecutable([service, port]()
    {
        string service_name = service;
        string protocol_name;
        split_mdns_service_name(&service_name, &protocol_name);
        esp_err_t res = mdns_service_add(
            NULL, service_name.c_str(), protocol_name.c_str(), port, NULL, 0);
        LOG(VERBOSE, "[mDNS] mdns_service_add(%s.%s:%d): %s."
          , service_name.c_str(), protocol_name.c_str(), port
          , esp_err_to_name(res));
        // ESP_FAIL will be triggered if there is a timeout during publish of
        // the new mDNS entry. The mDNS task runs at a very low priority on the
        // PRO_CPU which is also where the OpenMRN Executor runs from which can
        // cause a race condition.
        if (res == ESP_FAIL)
        {
            // Send it back onto the scheduler to be retried
            Singleton<Esp32WiFiManager>::instance()->mdns_publish(service
                                                                , port);
        }
        else if (res != ESP_OK)
        {
            LOG_ERROR("[mDNS] Failed to advertise %s.%s:%d due to: %s (%d)"
                    , service_name.c_str(), protocol_name.c_str(), port
                    , esp_err_to_name(res), res);
        }
        else
        {
            LOG(INFO, "[mDNS] Advertising %s.%s:%d.", service_name.c_str()
              , protocol_name.c_str(), port);
        }
    }));
}

// Removes advertisement of a service from mDNS.
void Esp32WiFiManager::mdns_unpublish(string service)
{
    {
        OSMutexLock l(&mdnsInitLock_);
        if (!mdnsInitialized_)
        {
            // Since mDNS is not in an initialized state we can discard the
            // unpublish event.
            return;
        }
    }
    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);
    LOG(INFO, "[mDNS] Removing advertisement of %s.%s."
      , service_name.c_str(), protocol_name.c_str());
    esp_err_t res =
        mdns_service_remove(service_name.c_str(), protocol_name.c_str());
    LOG(VERBOSE, "[mDNS] mdns_service_remove: %s.", esp_err_to_name(res));
}

// Initializes the mDNS system on the ESP32.
//
// After initialization, if any services are pending publish they will be
// published at this time.
void Esp32WiFiManager::start_mdns_system()
{
    // Initialize the mDNS system if it has not already been started.
    {
        OSMutexLock l(&mdnsInitLock_);
        // If we have already initialized mDNS we can exit early.
        if (mdnsInitialized_)
        {
            return;
        }

        // Initialize the mDNS system.
        LOG(INFO, "[mDNS] Initializing mDNS system");
        ESP_ERROR_CHECK(mdns_init());

        // Set the mDNS hostname based on our generated hostname so it can be
        // found by other nodes.
        LOG(INFO, "[mDNS] Setting mDNS hostname to \"%s\"", hostname_.c_str());
        ESP_ERROR_CHECK(mdns_hostname_set(hostname_.c_str()));

        // Set the default mDNS instance name to the generated hostname.
        ESP_ERROR_CHECK(mdns_instance_name_set(hostname_.c_str()));

        // Set flag to indicate we have initialized mDNS.
        mdnsInitialized_ = true;
    }

    // Publish any deferred mDNS entries
    for (auto & entry : mdnsDeferredPublish_)
    {
        mdns_publish(entry.first, entry.second);
    }
    mdnsDeferredPublish_.clear();
}

void Esp32WiFiManager::configure_wifi_max_tx_power()
{
    int8_t current_power = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_get_max_tx_power(&current_power));
    if (wifiTXPower_ != current_power)
    {
        LOG(INFO, "[WiFi] Adjusting maximum WiFi TX power %d -> %d.",
            current_power, wifiTXPower_);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_max_tx_power(wifiTXPower_));
    }
}

void Esp32WiFiManager::on_station_started()
{
    // Set the generated hostname prior to connecting to the SSID
    // so that it shows up with the generated hostname instead of
    // the default "Espressif".
    LOG(INFO, "[WiFi] Setting ESP32 hostname to \"%s\".",
        hostname_.c_str());
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    esp_netif_set_hostname(esp_netifs[ESP_IF_WIFI_STA], hostname_.c_str());
#else
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(
        TCPIP_ADAPTER_IF_STA, hostname_.c_str()));
#endif
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    LOG(INFO, "[WiFi] MAC Address: %s", mac_to_string(mac).c_str());

    if (stationStaticIP_)
    {
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        // Stop the DHCP service before connecting, this allows us to
        // specify a static IP address for the WiFi connection
        LOG(INFO, "[DHCP] Stopping DHCP Client (if running).");
        esp_netif_dhcpc_stop(esp_netifs[ESP_IF_WIFI_STA]);

        LOG(INFO,
            "[WiFi] Configuring Static IP address:\n"
            "IP     : " IPSTR "\n"
            "Gateway: " IPSTR "\n"
            "Netmask: " IPSTR,
            IP2STR(&stationStaticIP_->ip),
            IP2STR(&stationStaticIP_->gw),
            IP2STR(&stationStaticIP_->netmask));
        ESP_ERROR_CHECK(
            esp_netif_set_ip_info(esp_netifs[ESP_IF_WIFI_STA]
                                , stationStaticIP_));

        // if we do not have a primary DNS address configure the default
        if (ip_addr_isany(&primaryDNSAddress_))
        {
            IP4_ADDR(&primaryDNSAddress_.u_addr.ip4, 8, 8, 8, 8);
        }
        LOG(INFO, "[WiFi] Configuring primary DNS address to: " IPSTR,
            IP2STR(&primaryDNSAddress_.u_addr.ip4));
        esp_netif_dns_info_t dns_info;
        dns_info.ip.u_addr.ip4.addr = primaryDNSAddress_.u_addr.ip4.addr;
        dns_info.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(esp_netifs[ESP_IF_WIFI_STA], ESP_NETIF_DNS_MAIN
                             , &dns_info);
#else
        // Stop the DHCP service before connecting, this allows us to
        // specify a static IP address for the WiFi connection
        LOG(INFO, "[DHCP] Stopping DHCP Client (if running).");
        ESP_ERROR_CHECK(
            tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));

        LOG(INFO,
            "[WiFi] Configuring Static IP address:\n"
            "IP     : " IPSTR "\n"
            "Gateway: " IPSTR "\n"
            "Netmask: " IPSTR,
            IP2STR(&stationStaticIP_->ip),
            IP2STR(&stationStaticIP_->gw),
            IP2STR(&stationStaticIP_->netmask));
        ESP_ERROR_CHECK(
            tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA,
                                        stationStaticIP_));

        // if we do not have a primary DNS address configure the default
        if (ip_addr_isany(&primaryDNSAddress_))
        {
            IP4_ADDR(&primaryDNSAddress_.u_addr.ip4, 8, 8, 8, 8);
        }
        LOG(INFO, "[WiFi] Configuring primary DNS address to: " IPSTR,
            IP2STR(&primaryDNSAddress_.u_addr.ip4));
        // set the primary server (0)
        dns_setserver(0, &primaryDNSAddress_);
#endif
    }
    else
    {
        // Start the DHCP service before connecting so it hooks into
        // the flow early and provisions the IP automatically.
        LOG(INFO, "[DHCP] Starting DHCP Client.");
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        ESP_ERROR_CHECK(esp_netif_dhcpc_start(esp_netifs[ESP_IF_WIFI_STA]));
#else
        ESP_ERROR_CHECK(tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA));
#endif // IDF v4.1+
    }

    // Set the maximum transmit power before we connect to the SSID.
    configure_wifi_max_tx_power();

    LOG(INFO,
        "[WiFi] Station started, attempting to connect to SSID: %s.", ssid_);
    // Start the SSID connection process.
    esp_wifi_connect();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_init_callback_t cb : networkInitCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb]
            {
                cb(ESP_IF_WIFI_STA);
            }));
        }
    }
}

void Esp32WiFiManager::on_station_connected()
{
    LOG(INFO, "[WiFi] Connected to SSID: %s", ssid_);
    // Set the flag that indictes we are connected to the SSID.
    xEventGroupSetBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
}

void Esp32WiFiManager::on_station_disconnected(uint8_t reason)
{
    // flag to indicate that we should print the reconnecting log message.
    bool was_previously_connected = false;

    // Check if we have already connected, this event can be raised
    // even before we have successfully connected during the SSID
    // connect process.
    if (xEventGroupGetBits(wifiStatusEventGroup_) & WIFI_CONNECTED_BIT)
    {
        // track that we were connected previously.
        was_previously_connected = true;

        LOG(INFO, "[WiFi] Lost connection to SSID: %s (reason:%d)", ssid_
            , reason);
        // Clear the flag that indicates we are connected to the SSID.
        xEventGroupClearBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
        // Clear the flag that indicates we have an IPv4 address.
        xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

        // Wake up the wifi_manager_task so it can clean up
        // connections.
        xTaskNotifyGive(wifiTaskHandle_);
    }

    // If we are managing the WiFi and MDNS systems we need to
    // trigger the reconnection process at this point.
    if (was_previously_connected)
    {
        LOG(INFO, "[WiFi] Attempting to reconnect to SSID: %s.", ssid_);
    }
    else
    {
        LOG(INFO,
            "[WiFi] Connection failed, reconnecting to SSID: %s (reason:%d).",
            ssid_, reason);
    }
    esp_wifi_connect();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_init_callback_t cb : networkInitCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb]
            {
                cb(ESP_IF_WIFI_STA);
            }));
        }
    }
}

void Esp32WiFiManager::on_station_ip_assigned(uint32_t ip_address)
{
    LOG(INFO, "[WiFi] IP address is %s, starting hub (if enabled) and uplink"
      , ipv4_to_string(ip_address).c_str());

    // Start the mDNS system since we have an IP address, the mDNS system
    // on the ESP32 requires that the IP address be assigned otherwise it
    // will not start the UDP listener.
    start_mdns_system();

    // Set the flag that indictes we have an IPv4 address.
    xEventGroupSetBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

    // Wake up the wifi_manager_task so it can start connections
    // creating connections, this will be a no-op for initial startup.
    xTaskNotifyGive(wifiTaskHandle_);

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_up_callback_t cb : networkUpCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb, ip_address]
            {
                cb(ESP_IF_WIFI_STA, ip_address);
            }));
        }
    }
}

void Esp32WiFiManager::on_station_ip_lost()
{
    // Clear the flag that indicates we are connected and have an
    // IPv4 address.
    xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

    // Wake up the wifi_manager_task so it can clean up connections.
    xTaskNotifyGive(wifiTaskHandle_);

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_down_callback_t cb : networkDownCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb]
            {
                cb(ESP_IF_WIFI_STA);
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_start()
{
    uint32_t ip_address = 0;
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    LOG(INFO, "[SoftAP] MAC Address: %s", mac_to_string(mac).c_str());

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    // Set the generated hostname prior to connecting to the SSID
    // so that it shows up with the generated hostname instead of
    // the default "Espressif".
    LOG(INFO, "[SoftAP] Setting ESP32 hostname to \"%s\".",
        hostname_.c_str());
    ESP_ERROR_CHECK(
        esp_netif_set_hostname(esp_netifs[ESP_IF_WIFI_AP], hostname_.c_str()));

    // If the SoftAP is not configured to use a static IP it will default
    // to 192.168.4.1.
    if (softAPStaticIP_ && wifiMode_ != WIFI_MODE_STA)
    {
        // Stop the DHCP server so we can reconfigure it.
        esp_netif_dhcp_status_t dhcp_status;
        ESP_ERROR_CHECK(
            esp_netif_dhcps_get_status(esp_netifs[ESP_IF_WIFI_AP]
                                     , &dhcp_status));
        if (dhcp_status == ESP_NETIF_DHCP_STARTED)
        {
            LOG(INFO, "[SoftAP] Stopping DHCP Server.");
            ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_netifs[ESP_IF_WIFI_AP]));
        }

        LOG(INFO,
            "[SoftAP] Configuring Static IP address:\n"
            "IP     : " IPSTR "\n"
            "Gateway: " IPSTR "\n"
            "Netmask: " IPSTR,
            IP2STR(&softAPStaticIP_->ip),
            IP2STR(&softAPStaticIP_->gw),
            IP2STR(&softAPStaticIP_->netmask));
        ESP_ERROR_CHECK(
            esp_netif_set_ip_info(esp_netifs[ESP_IF_WIFI_AP]
                                , softAPStaticIP_));

        // Convert the Soft AP Static IP to a uint32 for manipulation
        ip_address = ntohl(ip4_addr_get_u32(&softAPStaticIP_->ip));

        // Default configuration is for DHCP addresses to follow
        // immediately after the static ip address of the Soft AP.
        ip4_addr_t first_ip, last_ip;
        ip4_addr_set_u32(&first_ip, htonl(ip_address + 1));
        ip4_addr_set_u32(&last_ip
                       , htonl(ip_address + SOFTAP_IP_RESERVATION_BLOCK_SIZE));
        dhcps_lease_t dhcp_lease {
            true,                   // enable dhcp lease functionality
            first_ip,               // first ip to assign
            last_ip,                // last ip to assign
        };

        LOG(INFO,
            "[SoftAP] Configuring DHCP Server for IPs: " IPSTR " - " IPSTR,
            IP2STR(&dhcp_lease.start_ip), IP2STR(&dhcp_lease.end_ip));
        ESP_ERROR_CHECK(
            esp_netif_dhcpc_option(esp_netifs[ESP_IF_WIFI_AP]
                                 , ESP_NETIF_OP_SET
                                 , ESP_NETIF_REQUESTED_IP_ADDRESS
                                 , (void *)&dhcp_lease
                                 , sizeof(dhcps_lease_t)));

        // Start the DHCP server so it can provide IP addresses to stations
        // when they connect.
        LOG(INFO, "[SoftAP] Starting DHCP Server.");
        ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netifs[ESP_IF_WIFI_AP]));
    }
    else
    {
        // fetch the IP address from the adapter since it defaults to
        // 192.168.4.1 but can be altered via sdkconfig.
        ESP32_ADAPTER_IP_INFO_TYPE ip_info;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netifs[ESP_IF_WIFI_AP]
                                            , &ip_info));
        ip_address = ntohl(ip4_addr_get_u32(&ip_info.ip));
    }
#else
    // Set the generated hostname prior to connecting to the SSID
    // so that it shows up with the generated hostname instead of
    // the default "Espressif".
    LOG(INFO, "[SoftAP] Setting ESP32 hostname to \"%s\".",
        hostname_.c_str());
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(
        TCPIP_ADAPTER_IF_AP, hostname_.c_str()));

    // If the SoftAP is not configured to use a static IP it will default
    // to 192.168.4.1.
    if (softAPStaticIP_ && wifiMode_ != WIFI_MODE_STA)
    {
        // Stop the DHCP server so we can reconfigure it.
        LOG(INFO, "[SoftAP] Stopping DHCP Server (if running).");
        ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

        LOG(INFO,
            "[SoftAP] Configuring Static IP address:\n"
            "IP     : " IPSTR "\n"
            "Gateway: " IPSTR "\n"
            "Netmask: " IPSTR,
            IP2STR(&softAPStaticIP_->ip),
            IP2STR(&softAPStaticIP_->gw),
            IP2STR(&softAPStaticIP_->netmask));
        ESP_ERROR_CHECK(
            tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP,
                                        softAPStaticIP_));

        // Convert the Soft AP Static IP to a uint32 for manipulation
        ip_address = ntohl(ip4_addr_get_u32(&softAPStaticIP_->ip));

        // Default configuration is for DHCP addresses to follow
        // immediately after the static ip address of the Soft AP.
        ip4_addr_t first_ip, last_ip;
        ip4_addr_set_u32(&first_ip, htonl(ip_address + 1));
        ip4_addr_set_u32(&last_ip
                       , htonl(ip_address + SOFTAP_IP_RESERVATION_BLOCK_SIZE));

        dhcps_lease_t dhcp_lease {
            true,                   // enable dhcp lease functionality
            first_ip,               // first ip to assign
            last_ip,                // last ip to assign
        };

        LOG(INFO,
            "[SoftAP] Configuring DHCP Server for IPs: " IPSTR " - " IPSTR,
            IP2STR(&dhcp_lease.start_ip), IP2STR(&dhcp_lease.end_ip));
        ESP_ERROR_CHECK(
            tcpip_adapter_dhcps_option(TCPIP_ADAPTER_OP_SET,
                                        TCPIP_ADAPTER_REQUESTED_IP_ADDRESS,
                                        (void *)&dhcp_lease,
                                        sizeof(dhcps_lease_t)));

        // Start the DHCP server so it can provide IP addresses to stations
        // when they connect.
        LOG(INFO, "[SoftAP] Starting DHCP Server.");
        ESP_ERROR_CHECK(
            tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
    }
    else
    {
        // fetch the IP address from the adapter since it defaults to
        // 192.168.4.1 but can be altered via sdkconfig.
        ESP32_ADAPTER_IP_INFO_TYPE ip_info;
        ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP
                                                , &ip_info));
        ip_address = ntohl(ip4_addr_get_u32(&ip_info.ip));
    }
#endif // IDF v4.1+

    if (wifiMode_ == WIFI_MODE_AP)
    {
        // Set the maximum transmit power. In the case of Station+SoftAP mode
        // this will be set as part of the station startup.
        configure_wifi_max_tx_power();

        // If we are operating in SoftAP mode only we can start mDNS and uplink
        // connection, otherwise defer it until the station has received it's
        // IP address to avoid reinitializing mDNS and uplink mDNS search or
        // connection failures.
        start_mdns_system();
        start_uplink();
    }

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_up_callback_t cb : networkUpCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb, ip_address]
            {
                cb(ESP_IF_WIFI_AP, htonl(ip_address));
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_stop()
{
    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp32_network_down_callback_t cb : networkDownCallbacks_)
        {
            stack_->executor()->add(new CallbackExecutable([cb]
            {
                cb(ESP_IF_WIFI_AP);
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_station_connected(wifi_event_ap_staconnected_t sta_info)
{
    LOG(INFO, "[SoftAP aid:%d] %s connected.", sta_info.aid
      , mac_to_string(sta_info.mac).c_str());

}

void Esp32WiFiManager::on_softap_station_disconnected(wifi_event_ap_stadisconnected_t sta_info)
{
    LOG(INFO, "[SoftAP aid:%d] %s disconnected.", sta_info.aid
      , mac_to_string(sta_info.mac).c_str());
}

void Esp32WiFiManager::on_wifi_scan_completed(wifi_event_sta_scan_done_t scan_info)
{
    OSMutexLock l(&ssidScanResultsLock_);
    if (scan_info.status)
    {
        LOG_ERROR("[WiFi] SSID scan failed!");
    }
    else
    {
        uint16_t num_found = scan_info.number;
        esp_wifi_scan_get_ap_num(&num_found);
        LOG(VERBOSE, "[WiFi] %d SSIDs found via scan", num_found);
        ssidScanResults_.resize(num_found);
        esp_wifi_scan_get_ap_records(&num_found, ssidScanResults_.data());
#if LOGLEVEL >= VERBOSE
        for (int i = 0; i < num_found; i++)
        {
            LOG(VERBOSE, "SSID: %s, RSSI: %d, channel: %d"
                , ssidScanResults_[i].ssid
                , ssidScanResults_[i].rssi, ssidScanResults_[i].primary);
        }
#endif
    }
    if (ssidCompleteNotifiable_)
    {
        ssidCompleteNotifiable_->notify();
        ssidCompleteNotifiable_ = nullptr;
    }
}

} // namespace openmrn_arduino

/// Maximum number of milliseconds to wait for mDNS query responses.
static constexpr uint32_t MDNS_QUERY_TIMEOUT = 2000;

/// Maximum number of results to capture for mDNS query requests.
static constexpr size_t MDNS_MAX_RESULTS = 10;

// Advertises an mDNS service name.
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    if (Singleton<Esp32WiFiManager>::exists())
    {
        // The name parameter is unused today.
        Singleton<Esp32WiFiManager>::instance()->mdns_publish(service, port);
    }
}

// Removes advertisement of an mDNS service name.
void mdns_unpublish(const char *service)
{
    if (Singleton<Esp32WiFiManager>::exists())
    {
        Singleton<Esp32WiFiManager>::instance()->mdns_unpublish(service);
    }
}

// Splits an mDNS service name.
void split_mdns_service_name(string *service_name, string *protocol_name)
{
    HASSERT(service_name != nullptr);
    HASSERT(protocol_name != nullptr);

    // if the string is not blank and contains a period split it on the period.
    if (service_name->length() && service_name->find('.', 0) != string::npos)
    {
        string::size_type split_loc = service_name->find('.', 0);
        protocol_name->assign(service_name->substr(split_loc + 1));
        service_name->resize(split_loc);
    }
}

// EAI_AGAIN may not be defined on the ESP32
#ifndef EAI_AGAIN
#ifdef TRY_AGAIN
#define EAI_AGAIN TRY_AGAIN
#else
#define EAI_AGAIN -3
#endif
#endif // EAI_AGAIN

// Looks for an mDNS service name and converts the results of the query to an
// addrinfo struct.
int mdns_lookup(
    const char *service, struct addrinfo *hints, struct addrinfo **addr)
{
    unique_ptr<struct addrinfo> ai(new struct addrinfo);
    if (ai.get() == nullptr)
    {
        LOG_ERROR("[mDNS] Allocation failed for addrinfo.");
        return EAI_MEMORY;
    }
    bzero(ai.get(), sizeof(struct addrinfo));

    unique_ptr<struct sockaddr> sa(new struct sockaddr);
    if (sa.get() == nullptr)
    {
        LOG_ERROR("[mDNS] Allocation failed for sockaddr.");
        return EAI_MEMORY;
    }
    bzero(sa.get(), sizeof(struct sockaddr));

    struct sockaddr_in *sa_in = (struct sockaddr_in *)sa.get();
    ai->ai_flags = 0;
    ai->ai_family = hints->ai_family;
    ai->ai_socktype = hints->ai_socktype;
    ai->ai_protocol = hints->ai_protocol;
    ai->ai_addrlen = sizeof(struct sockaddr_in);
    sa_in->sin_len = sizeof(struct sockaddr_in);
    sa_in->sin_family = hints->ai_family;

    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);

    mdns_result_t *results = NULL;
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(
            mdns_query_ptr(service_name.c_str(),
                           protocol_name.c_str(),
                           MDNS_QUERY_TIMEOUT,
                           MDNS_MAX_RESULTS,
                           &results)))
    {
        // failed to find any matches
        return EAI_FAIL;
    }

    if (!results)
    {
        // failed to find any matches
        LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
            "[mDNS] No matches found for service: %s.",
            service);
        return EAI_AGAIN;
    }

    // make a copy of the results to preserve the original list for cleanup.
    mdns_result_t *res = results;
    // scan the mdns query results linked list, the first match with an IPv4
    // address will be returned.
    bool match_found = false;
    while (res && !match_found)
    {
        mdns_ip_addr_t *ipaddr = res->addr;
        while (ipaddr && !match_found)
        {
            // if this result has an IPv4 address process it
            if (ipaddr->addr.type == IPADDR_TYPE_V4)
            {
                LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
                    "[mDNS] Found %s as providing service: %s on port %d.",
                    res->hostname, service, res->port);
                inet_addr_from_ip4addr(
                    &sa_in->sin_addr, &ipaddr->addr.u_addr.ip4);
                sa_in->sin_port = htons(res->port);
                match_found = true;
            }
            ipaddr = ipaddr->next;
        }
        res = res->next;
    }

    // free up the query results linked list.
    mdns_query_results_free(results);

    if (!match_found)
    {
        LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
            "[mDNS] No matches found for service: %s.",
            service);
        return EAI_AGAIN;
    }

    // return the resolved data to the caller
    *addr = ai.release();
    (*addr)->ai_addr = sa.release();

    // successfully resolved an address, inform the caller
    return 0;
}

// The functions below are not available via the standard ESP-IDF provided
// API.

/// Retrieves the IPv4 address from the ESP32 station interface.
///
/// @param ifap will hold the IPv4 address for the ESP32 station interface when
/// successfully retrieved.
/// @return zero for success, -1 for failure.
int getifaddrs(struct ifaddrs **ifap)
{
    tcpip_adapter_ip_info_t ip_info;

    /* start with something "safe" in case we bail out early */
    *ifap = nullptr;

    if (!tcpip_adapter_is_netif_up(TCPIP_ADAPTER_IF_STA))
    {
        // Station TCP/IP interface is not up
        errno = ENODEV;
        return -1;
    }

    // allocate memory for various pieces of ifaddrs
    std::unique_ptr<struct ifaddrs> ia(new struct ifaddrs);
    if (ia.get() == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    bzero(ia.get(), sizeof(struct ifaddrs));
    std::unique_ptr<char[]> ifa_name(new char[6]);
    if (ifa_name.get() == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    strcpy(ifa_name.get(), "wlan0");
    std::unique_ptr<struct sockaddr> ifa_addr(new struct sockaddr);
    if (ifa_addr == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    bzero(ifa_addr.get(), sizeof(struct sockaddr));

    // retrieve TCP/IP address from the interface
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);

    // copy address into ifaddrs structure
    struct sockaddr_in *addr_in = (struct sockaddr_in *)ifa_addr.get();
    addr_in->sin_family = AF_INET;
    addr_in->sin_addr.s_addr = ip_info.ip.addr;
    ia.get()->ifa_next = nullptr;
    ia.get()->ifa_name = ifa_name.release();
    ia.get()->ifa_flags = 0;
    ia.get()->ifa_addr = ifa_addr.release();
    ia.get()->ifa_netmask = nullptr;
    ia.get()->ifa_ifu.ifu_broadaddr = nullptr;
    ia.get()->ifa_data = nullptr;

    // report results
    *ifap = ia.release();
    return 0;
}

/// Frees memory allocated as part of the call to @ref getifaddrs.
///
/// @param ifa is the ifaddrs struct to be freed.
void freeifaddrs(struct ifaddrs *ifa)
{
    while (ifa)
    {
        struct ifaddrs *next = ifa->ifa_next;

        HASSERT(ifa->ifa_data == nullptr);
        HASSERT(ifa->ifa_ifu.ifu_broadaddr == nullptr);
        HASSERT(ifa->ifa_netmask == nullptr);

        delete ifa->ifa_addr;
        delete[] ifa->ifa_name;
        delete ifa;

        ifa = next;
    }
}

/// @return the string equivalant of the passed error code.
const char *gai_strerror(int __ecode)
{
    switch (__ecode)
    {
        default:
            return "gai_strerror unknown";
        case EAI_AGAIN:
            return "temporary failure";
        case EAI_FAIL:
            return "non-recoverable failure";
        case EAI_MEMORY:
            return "memory allocation failure";
    }
}

#endif // ESP32

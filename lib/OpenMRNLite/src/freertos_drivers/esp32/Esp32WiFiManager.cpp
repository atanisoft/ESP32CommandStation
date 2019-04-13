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

#include "Esp32WiFiManager.hxx"
#include "os/MDNS.hxx"

#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_wifi_internal.h>
#include <mdns.h>
#include <rom/crc.h>
#include <tcpip_adapter.h>

using openlcb::NodeID;
using openlcb::SimpleCanStack;
using openlcb::TcpAutoAddress;
using openlcb::TcpClientConfig;
using openlcb::TcpClientDefaultParams;
using openlcb::TcpDefs;
using openlcb::TcpManualAddress;
using std::string;
using std::unique_ptr;

// Start of global namespace block.

// These must be declared *OUTSIDE* the openmrn_arduino namespace in order to
// be visible in the MDNS.cxx code.

/// Advertises an mDNS service name. This is a hook point for the MDNS class
/// and is used as part of the Esp32 WiFi HUB support.
void mdns_publish(const char *name, const char *service, uint16_t port);

/// Removes advertisement of an mDNS service name. This is not currently
/// exposed in the MDNS class but is supported on the ESP32.
void mdns_unpublish(const char *service);

// End of global namespace block.

namespace openmrn_arduino
{

/// Priority to use for the wifi_manager_task. This is currently set to one
/// level higher than the arduino-esp32 loopTask. The task will be in a sleep
/// state until woken up by Esp32WiFiManager::process_wifi_event or
/// Esp32WiFiManager::apply_configuration.
static constexpr UBaseType_t WIFI_TASK_PRIORITY = 2;

/// Stack size for the wifi_manager_task.
static constexpr uint32_t WIFI_TASK_STACK_SIZE = 2560L;

/// Interval at which to check the WiFi connection status.
static constexpr TickType_t WIFI_CONNECT_CHECK_INTERVAL = pdMS_TO_TICKS(5000);

/// Interval at which to check if the GcTcpHub has started or not.
static constexpr uint32_t HUB_STARTUP_DELAY_USEC = MSEC_TO_USEC(50);

/// Bit designator for wifi_status_event_group which indicates we are connected
/// to the SSID.
static constexpr int WIFI_CONNECTED_BIT = BIT0;

/// Bit designator for wifi_status_event_group which indicates we have an IPv4
/// address assigned.
static constexpr int DHCP_GOTIP_BIT = BIT1;

/// Allow up to 36 checks to see if we have connected to the SSID and
/// received an IPv4 address. This allows up to ~3 minutes for the entire
/// process to complete, in most cases this should be complete in under 30
/// seconds.
static constexpr uint8_t MAX_CONNECTION_CHECK_ATTEMPTS = 36;

/// Event handler for the ESP32 WiFi system. This will receive events from the
/// ESP-IDF event loop processor and pass them on to the Esp32WiFiManager for
/// possible processing. This is only used when Esp32WiFiManager is managing
/// both the WiFi and mDNS systems, if these are managed externally the
/// consumer is responsible for calling Esp32WiFiManager::process_wifi_event
/// when WiFi events occur.
static esp_err_t wifi_event_handler(void *context, system_event_t *event)
{
    auto wifi = static_cast<Esp32WiFiManager *>(context);
    wifi->process_wifi_event(event->event_id);
    return ESP_OK;
}

/// Adapter class to load/store configuration via CDI
class Esp32SocketParams : public DefaultSocketClientParams
{
public:
    Esp32SocketParams(
        int fd, const TcpClientConfig<TcpClientDefaultParams> &cfg)
        : configFd_(fd)
        , cfg_(cfg)
    {
        mdnsService_ = cfg_.auto_address().service_name().read(configFd_);
        staticHost_ = cfg_.manual_address().ip_address().read(configFd_);
        staticPort_ = CDI_READ_TRIMMED(cfg_.manual_address().port, configFd_);
    }

    /// @return search mode for how to locate the server.
    SearchMode search_mode() override
    {
        return (SearchMode)CDI_READ_TRIMMED(cfg_.search_mode, configFd_);
    }

    /// @return null or empty string if any mdns server is okay to connect
    /// to. If nonempty, then only an mdns server will be chosen that has the
    /// specific host name.
    string mdns_host_name() override
    {
        return cfg_.auto_address().host_name().read(configFd_);
    }

    /// @return true if first attempt should be to connect to
    /// last_host_name:last_port.
    bool enable_last() override
    {
        return CDI_READ_TRIMMED(cfg_.reconnect, configFd_);
    }

    /// @return the last successfully used IP address, as dotted
    /// decimal. Nullptr or empty if no successful connection has ever been
    /// made.
    string last_host_name() override
    {
        return cfg_.last_address().ip_address().read(configFd_);
    }

    /// @return the last successfully used port number.
    int last_port() override
    {
        return CDI_READ_TRIMMED(cfg_.last_address().port, configFd_);
    }

    /// Stores the last connection details for use when reconnect is enabled.
    ///
    /// @param hostname is the hostname that was connected to.
    /// @param port is the port that was connected to.
    void set_last(const char *hostname, int port) override
    {
        cfg_.last_address().ip_address().write(configFd_, hostname);
        cfg_.last_address().port().write(configFd_, port);
    }

    void log_message(LogMessage id, const string &arg) override
    {
        switch (id)
        {
            case CONNECT_RE:
                LOG(INFO, "[Uplink] Reconnecting to %s.", arg.c_str());
                break;
            case MDNS_SEARCH:
                LOG(INFO, "[Uplink] Starting mDNS searching for %s.",
                    arg.c_str());
                break;
            case MDNS_NOT_FOUND:
                LOG(INFO, "[Uplink] mDNS search failed.");
                break;
            case MDNS_FOUND:
                LOG(INFO, "[Uplink] mDNS search succeeded.");
                break;
            case CONNECT_MDNS:
                LOG(INFO, "[Uplink] mDNS connecting to %s.", arg.c_str());
                break;
            case CONNECT_MANUAL:
                LOG(INFO, "[Uplink] Connecting to %s.", arg.c_str());
                break;
            case CONNECT_FAILED_SELF:
                LOG(INFO,
                    "[Uplink] Rejecting attempt to connect to "
                    "localhost.");
                break;
            case CONNECTION_LOST:
                LOG(INFO, "[Uplink] Connection lost.");
                break;
            default:
                // ignore the message
                break;
        }
    }

    /// @return true if we should actively skip connections that happen to
    /// match our own IP address.
    bool disallow_local() override
    {
        return true;
    }

private:
    const int configFd_;
    const TcpClientConfig<TcpClientDefaultParams> cfg_;
};

// With this constructor being used the Esp32WiFiManager will manage the
// WiFi connection, mDNS system and the hostname of the ESP32.
Esp32WiFiManager::Esp32WiFiManager(const char *ssid, const char *password,
    SimpleCanStack *stack, const WiFiConfiguration &cfg)
    : DefaultConfigUpdateListener()
    , ssid_(ssid)
    , password_(password)
    , cfg_(cfg)
    , manageWiFi_(true)
    , stack_(stack)
{
    // Extend the capacity of the hostname to make space for the node-id and
    // underscore.
    hostname_.reserve(TCPIP_HOSTNAME_MAX_SIZE);

    // Generate the hostname for the ESP32 based on the provided node id.
    // node_id : 0x050101011425
    // hostname_ : esp32_050101011425
    NodeID node_id = stack_->node()->node_id();
    hostname_.append(uint64_to_string_hex(node_id, 0));

    // The maximum length hostname for the ESP32 is 32 characters so truncate
    // when necessary. Reference to length limitation:
    // https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
    if (hostname_.length() > TCPIP_HOSTNAME_MAX_SIZE)
    {
        LOG(WARNING, "ESP32 hostname is too long, original hostname: %s",
            hostname_.c_str());
        hostname_.resize(TCPIP_HOSTNAME_MAX_SIZE);
        LOG(WARNING, "truncated hostname: %s", hostname_.c_str());
    }

    // Release any extra capacity allocated for the hostname.
    hostname_.shrink_to_fit();
}

// With this constructor being used, it will be the responsibility of the
// application to manage the WiFi and mDNS systems.
Esp32WiFiManager::Esp32WiFiManager(
    SimpleCanStack *stack, const WiFiConfiguration &cfg)
    : DefaultConfigUpdateListener()
    , cfg_(cfg)
    , manageWiFi_(false)
    , stack_(stack)
{
    // Nothing to do here.
}

ConfigUpdateListener::UpdateAction Esp32WiFiManager::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    AutoNotify n(done);
    LOG(VERBOSE, "Esp32WiFiManager::apply_configuration(%d, %d)", fd,
        initial_load);

    // Cache the fd for later use by the wifi background task.
    configFd_ = fd;
    configReloadRequested_ = initial_load;

    // Load the CDI entry into memory to do an CRC-32 check against our last
    // loaded configuration so we can avoid reloading configuration when there
    // are no interesting changes.
    unique_ptr<uint8_t[]> crcbuf(new uint8_t[cfg_.size()]);

    // If we are unable to seek to the right position in the persistent storage
    // give up and request a reboot.
    if (lseek(fd, cfg_.offset(), SEEK_SET) != cfg_.offset())
    {
        LOG(WARNING, "lseek failed to reset fd offset, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // If we are unable to read the full configuration from persistent storage
    // give up and request a reboot.
    if (read(fd, crcbuf.get(), cfg_.size()) != cfg_.size())
    {
        LOG(WARNING, "read failed to fully read the config, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // Calculate CRC-32 from the loaded buffer.
    uint32_t configCrc32 = crc32_le(0, crcbuf.get(), cfg_.size());
    LOG(VERBOSE, "existing config CRC-32: \"%s\", new CRC-32: \"%s\"",
        integer_to_string(configCrc32_, 0).c_str(),
        integer_to_string(configCrc32, 0).c_str());

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

    // Hub specific configuration settings.
    CDI_FACTORY_RESET(cfg_.hub().enable);
    CDI_FACTORY_RESET(cfg_.hub().port);
    cfg_.hub().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);

    // Node link configuration settings.
    CDI_FACTORY_RESET(cfg_.uplink().search_mode);
    CDI_FACTORY_RESET(cfg_.uplink().reconnect);

    // Node link manual configuration settings.
    cfg_.uplink().manual_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.uplink().manual_address().port);

    // Node link automatic configuration settings.
    cfg_.uplink().auto_address().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
    cfg_.uplink().auto_address().host_name().write(fd, "");

    // Node link automatic last connected node address.
    cfg_.uplink().last_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.uplink().last_address().port);

    // Reconnect to last connected node.
    CDI_FACTORY_RESET(cfg_.uplink().reconnect);
}

// Processes a WiFi system event
void Esp32WiFiManager::process_wifi_event(int event_id)
{
    LOG(VERBOSE, "Esp32WiFiManager::process_wifi_event(%d)", event_id);

    switch (event_id)
    {
        case SYSTEM_EVENT_STA_START:
            // We only are interested in this event if we are managing the
            // WiFi and MDNS systems
            if (manageWiFi_)
            {
                // Set the generated hostname prior to connecting to the SSID
                // so that it shows up with the generated hostname instead of
                // the default "Espressif".
                LOG(INFO, "[WiFi] Setting ESP32 hostname to \"%s\".",
                    hostname_.c_str());
                ESP_ERROR_CHECK(tcpip_adapter_set_hostname(
                    TCPIP_ADAPTER_IF_STA, hostname_.c_str()));

                // Start the DHCP service before connecting to it hooks into
                // the flow early and provisions the IP automatically.
                LOG(INFO, "[WiFi] Starting DHCP services.");
                ESP_ERROR_CHECK(
                    tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA));

                LOG(INFO,
                    "[WiFi] Station started, attempting to connect "
                    "to SSID: %s.",
                    ssid_);
                // Start the SSID connection process.
                esp_wifi_connect();
            }
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            LOG(INFO, "[WiFi] Connected to SSID: %s", ssid_);
            // Set the flag that indictes we are connected to the SSID.
            xEventGroupSetBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            // Retrieve the configured IP address from the TCP/IP stack.
            tcpip_adapter_ip_info_t ip_info;
            tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
            LOG(INFO,
                "[WiFi] IP address is " IPSTR ", starting hub (if "
                "enabled) and uplink.",
                IP2STR(&ip_info.ip));

            // Set the flag that indictes we have an IPv4 address.
            xEventGroupSetBits(wifiStatusEventGroup_, DHCP_GOTIP_BIT);

            // Wake up the wifi_manager_task so it can start connections
            // creating connections, this will be a no-op for initial startup.
            xTaskNotifyGive(wifiTaskHandle_);
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            // clear the flag that indicates we are connected and have an
            // IPv4 address.
            xEventGroupClearBits(wifiStatusEventGroup_, DHCP_GOTIP_BIT);
            // Wake up the wifi_manager_task so it can clean up connections.
            xTaskNotifyGive(wifiTaskHandle_);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // check if we have already connected, this event can be raised
            // even before we have successfully connected during the SSID
            // connect process.
            if (xEventGroupGetBits(wifiStatusEventGroup_) & WIFI_CONNECTED_BIT)
            {
                LOG(INFO, "[WiFi] Lost connection to SSID: %s", ssid_);
                // clear the flag that indicates we are connected to the SSID.
                xEventGroupClearBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
                // clear the flag that indicates we have an IPv4 address.
                xEventGroupClearBits(wifiStatusEventGroup_, DHCP_GOTIP_BIT);

                // Wake up the wifi_manager_task so it can clean up
                // connections.
                xTaskNotifyGive(wifiTaskHandle_);
            }

            // If we are managing the WiFi and MDNS systems we need to
            // trigger the reconnection process at this point.
            if (manageWiFi_)
            {
                LOG(INFO, "[WiFi] Attempting to reconnect to SSID: %s.",
                    ssid_);
                esp_wifi_connect();
            }
            break;
    }
}

void Esp32WiFiManager::enable_verbose_logging()
{
    esp32VerboseLogging_ = true;
    enable_esp_wifi_logging();
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
// 8) Connect to WiFi and wait for DHCP IP assignment.
// 9) Verify that we connected and received a DHCP IP address, if not log a
// FATAL message and give up.
// 10) Initialize the mDNS system.
// 11) Set the mDNS hostname based on the generated hostname.
// 12) Set the default mDNS instance name based on the generated hostname.
void Esp32WiFiManager::start_wifi_system()
{
    // Create the event group used for tracking connected/disconnected status.
    // This is used internally regardless of if we manage the rest of the WiFi
    // or mDNS systems.
    wifiStatusEventGroup_ = xEventGroupCreate();

    // If we do not need to manage the WiFi and mDNS systems exit early.
    if (!manageWiFi_)
    {
        return;
    }

    // Initialize the TCP/IP adapter stack.
    LOG(INFO, "[WiFi] Starting TCP/IP stack");
    tcpip_adapter_init();

    // Install event loop handler.
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, this));

    // Start the WiFi adapter.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    LOG(INFO, "[WiFi] Initializing WiFi stack");
    cfg.nvs_enable = false;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (esp32VerboseLogging_)
    {
        enable_esp_wifi_logging();
    }

    // Set the WiFi mode to STATION.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // This disables storage of SSID details in NVS which has been shown to be
    // problematic at times for the ESP32, it is safer to always pass fresh
    // config and have the ESP32 resolve the details at runtime rather than
    // use a cached set from NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    // Configure the SSID details for the station based on the SSID and
    // password provided to the Esp32WiFiManager constructor.
    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));
    strcpy(reinterpret_cast<char *>(conf.sta.ssid), ssid_);
    if (password_)
    {
        strcpy(reinterpret_cast<char *>(conf.sta.password), password_);
    }

    LOG(INFO, "[WiFi] Configuring WiFi stack");
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &conf));

    // Attempt to connect to the SSID, this will block until the ESP32 starts
    // the connection process, note it may not have an IP address immediately
    // thus the need to check the connection result a few times before giving
    // up with a FATAL error.
    LOG(INFO, "[WiFi] Starting WiFi stack");
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t attempt = 0;
    EventBits_t bits;
    uint32_t bitMask = WIFI_CONNECTED_BIT;
    while (++attempt <= MAX_CONNECTION_CHECK_ATTEMPTS)
    {
        // If we have connected to the SSID we then are waiting for DHCP.
        if (bits & WIFI_CONNECTED_BIT)
        {
            LOG(INFO, "[DHCP] [%d/%d] Waiting for IP address assignment.",
                attempt, MAX_CONNECTION_CHECK_ATTEMPTS);
        }
        else
        {
            // Waiting for SSID connection
            LOG(INFO, "[WiFi] [%d/%d] Waiting for SSID connection.", attempt,
                MAX_CONNECTION_CHECK_ATTEMPTS);
        }
        bits = xEventGroupWaitBits(wifiStatusEventGroup_,
            bitMask, // bits we are interested in
            pdFALSE, // clear on exit
            pdTRUE,  // wait for all bits
            WIFI_CONNECT_CHECK_INTERVAL);
        // Check if have connected to the SSID
        if (bits & WIFI_CONNECTED_BIT)
        {
            // Since we have connected to the SSID we now need to track that we
            // get an IP via DHCP.
            bitMask |= DHCP_GOTIP_BIT;
        }
        // Check if we have received an IP via DHCP.
        if (bits & DHCP_GOTIP_BIT)
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
    if ((bits & DHCP_GOTIP_BIT) != DHCP_GOTIP_BIT)
    {
        LOG(FATAL, "[DHCP] Timeout waiting for an IP.");
    }

    // Initialize the mDNS system.
    LOG(INFO, "[mDNS] Initializing mDNS system");
    ESP_ERROR_CHECK(mdns_init());

    // Set the mDNS hostname based on our generated hostname so it can be found
    // by other nodes.
    LOG(INFO, "[mDNS] Setting mDNS hostname to \"%s\"", hostname_.c_str());
    ESP_ERROR_CHECK(mdns_hostname_set(hostname_.c_str()));

    // Set the default mDNS instance name to the generated hostname.
    ESP_ERROR_CHECK(mdns_instance_name_set(hostname_.c_str()));
}

// Starts a background task for the Esp32WiFiManager.
void Esp32WiFiManager::start_wifi_task()
{
    LOG(INFO, "[WiFiMgr] Starting WiFi Manager task");
    os_thread_create(&wifiTaskHandle_, "OpenMRN-WiFiMgr", WIFI_TASK_PRIORITY,
        WIFI_TASK_STACK_SIZE, wifi_manager_task, this);
}

// Background task for the Esp32WiFiManager. This handles all outbound
// connection attempts, configuration loading and making this node as a hub.
void *Esp32WiFiManager::wifi_manager_task(void *param)
{
    Esp32WiFiManager *wifi = static_cast<Esp32WiFiManager *>(param);

    // Start the WiFi system before proceeding with remaining tasks.
    wifi->start_wifi_system();

    while (true)
    {
        EventBits_t bits = xEventGroupGetBits(wifi->wifiStatusEventGroup_);
        if (bits & DHCP_GOTIP_BIT)
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

        // Check if there are configuration changes to pick up.
        if (wifi->configReloadRequested_)
        {
            // Since we are loading configuration data, shutdown the hub and
            // uplink if created previously.
            wifi->stop_hub();
            wifi->stop_uplink();

            if (CDI_READ_TRIMMED(wifi->cfg_.sleep, wifi->configFd_))
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

            if (CDI_READ_TRIMMED(wifi->cfg_.hub().enable, wifi->configFd_))
            {
                // Since hub mode is enabled start the HUB creation process.
                wifi->start_hub();
            }
            // Start the uplink connection process in the background.
            wifi->start_uplink();
            wifi->configReloadRequested_ = false;
        }

        // Sleep until we are woken up again for configuration update or WiFi
        // event.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    return nullptr;
}

// Shuts down the hub listener (if enabled and running) for this node.
void Esp32WiFiManager::stop_hub()
{
    if (hub_)
    {
        mdns_unpublish(hubServiceName_.c_str());
        LOG(INFO, "[HUB] Shutting down TCP/IP listener");
        hub_.reset(nullptr);
    }
}

// Creates a hub listener for this node after loading configuration details.
void Esp32WiFiManager::start_hub()
{
    hubServiceName_ = cfg_.hub().service_name().read(configFd_);
    uint16_t hub_port = CDI_READ_TRIMMED(cfg_.hub().port, configFd_);

    LOG(INFO, "[HUB] Starting TCP/IP listener on port %d", hub_port);
    hub_.reset(new GcTcpHub(stack_->can_hub(), hub_port));

    // wait for the hub to complete it's startup tasks
    while (!hub_->is_started())
    {
        usleep(HUB_STARTUP_DELAY_USEC);
    }
    mdns_publish(NULL, hubServiceName_.c_str(), hub_port);
}

// Disconnects and shuts down the uplink connector socket if running.
void Esp32WiFiManager::stop_uplink()
{
    if (uplink_)
    {
        LOG(INFO, "[UPLINK] Disconnecting from uplink.");
        uplink_->shutdown();
        uplink_.reset(nullptr);
    }
}

// Creates an uplink connector socket that will automatically add the uplink to
// the node's hub.
void Esp32WiFiManager::start_uplink()
{
    unique_ptr<SocketClientParams> params(
        new Esp32SocketParams(configFd_, cfg_.uplink()));
    uplink_.reset(new SocketClient(stack_->service(), stack_->executor(),
        stack_->executor(), std::move(params),
        std::bind(&Esp32WiFiManager::on_uplink_created, this,
            std::placeholders::_1, std::placeholders::_2)));
}

// Converts the passed fd into a GridConnect port and adds it to the stack.
void Esp32WiFiManager::on_uplink_created(int fd, Notifiable *on_exit)
{
    LOG(INFO, "[UPLINK] Connected to hub, configuring GridConnect port.");

    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);

    // create the GridConnect port from the provided socket fd.
    create_gc_port_for_can_hub(stack_->can_hub(), fd, on_exit, use_select);

    // restart the stack to kick off alias allocation and send node init
    // packets.
    stack_->restart_stack();
}

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

} // namespace openmrn_arduino

/// Maximum number of milliseconds to wait for mDNS query responses.
static constexpr uint32_t MDNS_QUERY_TIMEOUT = 2000;

/// Maximum number of results to capture for mDNS query requests.
static constexpr size_t MDNS_MAX_RESULTS = 10;

/// Splits a service name since the ESP32 mDNS library requires the service
/// name and service protocol to be passed in individually.
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

// Advertises an mDNS service name for this node.
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);

    LOG(INFO, "[mDNS] Advertising %s.%s:%d.", service_name.c_str(),
        protocol_name.c_str(), port);
    esp_err_t res = mdns_service_add(
        NULL, service_name.c_str(), protocol_name.c_str(), port, NULL, 0);
    LOG(VERBOSE, "[mDNS] mdns_service_add: %s.", esp_err_to_name(res));
}

// Removes advertisement of an mDNS service name.
void mdns_unpublish(const char *service)
{
    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);
    LOG(INFO, "[mDNS] Removing advertisement of %s.%s.", service_name.c_str(),
        protocol_name.c_str());
    esp_err_t res =
        mdns_service_remove(service_name.c_str(), protocol_name.c_str());
    LOG(VERBOSE, "[mDNS] mdns_service_remove: %s.", esp_err_to_name(res));
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
        LOG(WARNING, "[mDNS] Allocation failed for addrinfo.");
        return EAI_MEMORY;
    }
    memset(ai.get(), 0, sizeof(struct addrinfo));

    unique_ptr<struct sockaddr> sa(new struct sockaddr);
    if (sa.get() == nullptr)
    {
        LOG(WARNING, "[mDNS] Allocation failed for sockaddr.");
        return EAI_MEMORY;
    }
    memset(sa.get(), 0, sizeof(struct sockaddr));

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
    esp_err_t err = mdns_query_ptr(service_name.c_str(), protocol_name.c_str(),
        MDNS_QUERY_TIMEOUT, MDNS_MAX_RESULTS, &results);
    LOG(VERBOSE, "[mDNS] mdns_query_ptr: %s.", esp_err_to_name(err));
    if (err)
    {
        // failed to find any matches
        LOG(WARNING, "[mDNS] mDNS query failed: %s.", esp_err_to_name(err));
        return EAI_FAIL;
    }

    if (!results)
    {
        // failed to find any matches
        LOG(WARNING, "[mDNS] No matches found for service: %s.", service);
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
                LOG(INFO,
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
        LOG(WARNING, "[mDNS] No matches found for service: %s.", service);
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
    memset(ia.get(), 0, sizeof(struct ifaddrs));
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
    memset(ifa_addr.get(), 0, sizeof(struct sockaddr));

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

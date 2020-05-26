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
 * \file Esp32WiFiManager.hxx
 *
 * ESP32 WiFi Manager
 *
 * @author Mike Dunston
 * @date 4 February 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_

#include "freertos_drivers/esp32/Esp32WiFiConfiguration.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredTcpConnection.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TcpDefs.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/Singleton.hxx"
#include "utils/SocketClient.hxx"
#include "utils/SocketClientParams.hxx"
#include "utils/macros.h"

#include <freertos/event_groups.h>
#include <esp_event.h>
#include <esp_wifi_types.h>

namespace openmrn_arduino
{

/// This class provides a simple way for ESP32 nodes to manage the WiFi and
/// mDNS systems of the ESP32, the node being a hub and connecting to an
/// uplink node to participate in the CAN bus.
///
/// There are two modes of operation for this class:
/// 1) Full management of WiFi, mDNS, hub and uplink. In this mode of operation
/// the Esp32WiFiManager will start the WiFi and mDNS systems automatically and
/// connect to the configured SSID as part of node initialization. If the node
/// is configured to be a hub it will be started automatically. The node's
/// uplink connection will also be managed internally.
/// 2) Management of only hub and uplink. In this mode of operation it is the
/// responsibility of the consumer to ensure that both the WiFi and mDNS
/// systems have been initialized and are running prior to calling
/// OpenMRN::begin() which will trigger the loading of the node configuration
/// which will trigger the management of the hub and uplink functionality.
class Esp32WiFiManager : public DefaultConfigUpdateListener
                       , public Singleton<Esp32WiFiManager>
{
public:
    /// Constructor.
    ///
    /// With this constructor the ESP32 WiFi and MDNS systems will be managed
    /// automatically by the Esp32WiFiManager class in addition to the inbound
    /// and outbound connections. The WiFi and MDNS systems will only be
    /// started after the initial loading of the CDI which occurs only after
    /// the application code calls OpenMRN::begin().
    ///
    /// @param ssid is the WiFi AP to connect to. Must stay alive forever.
    /// @param password is the password for the WiFi AP being connected
    /// to. Must stay alive forever.
    /// @param stack is the SimpleCanStack for this node. Must stay alive
    /// forever.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    /// @param hostname_prefix is the hostname prefix to use for this node.
    /// The @ref NodeID will be appended to this value. The maximum length for
    /// final hostname is 32 bytes.
    /// @param wifi_mode is the WiFi operating mode. When set to WIFI_MODE_STA
    /// the Esp32WiFiManager will attempt to connect to the provided WiFi SSID.
    /// When the wifi_mode is WIFI_MODE_AP the Esp32WiFiManager will create an
    /// AP with the provided SSID and PASSWORD. When the wifi_mode is
    /// WIFI_MODE_APSTA the Esp32WiFiManager will connect to the provided WiFi
    /// AP and create an AP with the SSID of "<hostname>" and the provided
    /// password. Note, the password for the AP will not be used if
    /// soft_ap_auth is set to WIFI_AUTH_OPEN (default).
    /// @param station_static_ip is the static IP configuration to use for the
    /// Station WiFi connection. If not specified DHCP will be used instead.
    /// @param primary_dns_server is the primary DNS server to use when a
    /// static IP address is being used. If left as the default (ip_addr_any)
    /// the Esp32WiFiManager will use 8.8.8.8 if using a static IP address.
    /// @param soft_ap_channel is the WiFi channel to use for the SoftAP.
    /// @param soft_ap_auth is the authentication mode for the AP when
    /// wifi_mode is set to WIFI_MODE_AP or WIFI_MODE_APSTA.
    /// @param soft_ap_password will be used as the password for the SoftAP,
    /// if null and soft_ap_auth is not WIFI_AUTH_OPEN password will be used.
    /// If provided, this must stay alive forever.
    /// @param softap_static_ip is the static IP configuration for the SoftAP,
    /// when not specified the SoftAP will have an IP address of 192.168.4.1.
    ///
    /// Note: Both ssid and password must remain in memory for the duration of
    /// node uptime.
    Esp32WiFiManager(const char *ssid
                   , const char *password
                   , openlcb::SimpleCanStack *stack
                   , const WiFiConfiguration &cfg
                   , const char *hostname_prefix = "esp32_"
                   , wifi_mode_t wifi_mode = WIFI_MODE_STA
                   , tcpip_adapter_ip_info_t *station_static_ip = nullptr
                   , ip_addr_t primary_dns_server = ip_addr_any
                   , uint8_t soft_ap_channel = 1
                   , wifi_auth_mode_t soft_ap_auth = WIFI_AUTH_OPEN
                   , const char *soft_ap_password = nullptr
                   , tcpip_adapter_ip_info_t *softap_static_ip = nullptr
    );

    /// Constructor.
    ///
    /// With this constructor the ESP32 WiFi and MDNS systems will not be
    /// managed by the Esp32WiFiManager class, only the inbound and outbound
    /// connections will be managed. This variation should only be used when
    /// the application code starts the the WiFi and MDNS systems before
    /// calling OpenMRN::begin().
    ///
    /// @param stack is the SimpleCanStack for this node.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    Esp32WiFiManager(
        openlcb::SimpleCanStack *stack, const WiFiConfiguration &cfg);

    /// Destructor.
    ~Esp32WiFiManager();

    /// Updates the WiFiConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    /// @param initial_load is set to true when this node loads the
    /// configuration for the first time, otherwise it is an update to the
    /// configuration and may require a restart.
    /// @param done is the control used by the caller to track when all config
    /// consumers have completed their updates.
    ///
    /// @return UPDATED when the configuration has been successfully updated,
    /// or REBOOT_NEEDED if the node needs to reboot for configuration to take
    /// effect.
    ConfigUpdateListener::UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override;

    /// Resets the WiFiConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    void factory_reset(int fd) override;

    /// Processes an ESP-IDF WiFi event based on the event raised by the
    /// ESP-IDF event loop processor. This should be used when the
    /// Esp32WiFiManager is not managing the WiFi or MDNS systems so that
    /// it can react to WiFi events to cleanup or recreate the hub or uplink
    /// connections as required. When Esp32WiFiManager is managing the WiFi
    /// connection this method will be called automatically from the
    /// esp_event_loop. Note that ESP-IDF only supports one callback being
    /// registered. 
    ///
    /// @param event is the system_event_t raised by ESP-IDF.
    void process_wifi_event(system_event_t *event);

    /// Adds a callback to receive WiFi events as they are received/processed
    /// by the Esp32WiFiManager.
    ///
    /// @param callback is the callback to invoke when events are received,
    /// the only parameter is the system_event_t that was received.
    void add_event_callback(std::function<void(system_event_t *)> callback)
    {
        OSMutexLock l(&eventCallbacksLock_);
        eventCallbacks_.emplace_back(std::move(callback));
    }

    /// If called, sets the ESP32 wifi stack to log verbose information to the
    /// ESP32 serial port.
    void enable_verbose_logging();

    /// Starts a scan for available SSIDs.
    ///
    /// @param n is the @ref Notifiable to notify when the SSID scan completes.
    void start_ssid_scan(Notifiable *n);

    /// @return the number of SSIDs that were found via the scan.
    size_t get_ssid_scan_result_count();

    /// Returns one entry from the SSID scan.
    ///
    /// @param index is the index of the SSID to retrieve. If the index is
    /// invalid or no records exist a blank wifi_ap_record_t will be returned.
    wifi_ap_record_t get_ssid_scan_result(size_t index);

    /// Clears the SSID scan results.
    void clear_ssid_scan_results();

    /// Advertises a service via mDNS.
    ///
    /// @param service is the service name to publish.
    /// @param port is the port for the service to be published.
    ///
    /// Note: This will schedule a @ref CallbackExecutable on the @ref Executor
    /// used by the @ref SimpleCanStack.
    void mdns_publish(std::string service, uint16_t port);

    /// Removes the advertisement of a service via mDNS.
    ///
    /// @param service is the service name to remove from advertising.
    void mdns_unpublish(std::string service);

    /// Forces the Esp32WiFiManager to wait until SSID connection completes.
    ///
    /// By default the ESP32 will be restarted if the SSID connection attempt
    /// has not completed after approximately three minutes. When the SoftAP
    /// interface is also active an application may opt to disable this
    /// functionality to allow a configuration portal to be displayed instead
    /// of requiring the firmware to be rebuilt with a new SSID/PW.
    void wait_for_ssid_connect(bool enable);

private:
    /// Default constructor.
    Esp32WiFiManager();

    /// Starts the WiFi system and initiates the SSID connection process.
    ///
    /// Note: This is a blocking call and will reboot the node if the WiFi
    /// connection is not successful after ~3min.
    void start_wifi_system();

    /// Starts the Esp32WiFiManager, this manages the WiFi subsystem as well as
    /// all interactions with other nodes.
    void start_wifi_task();

    /// Background task used by the Esp32WiFiManager to maintain health of any
    /// connections to other nodes.
    /// @param param is a pointer to the Esp32WiFiManager instance.
    static void *wifi_manager_task(void *param);

    /// Shuts down the hub listener (if running) for this node.
    void stop_hub();

    /// Creates a hub listener for this node after loading configuration
    /// details.
    ///
    /// Note: This method will block until the hub is active.
    void start_hub();

    /// Disconnects and shuts down the uplink connector socket (if running).
    void stop_uplink();

    /// Creates an uplink connector socket that will automatically add the
    /// uplink to the node's hub.
    void start_uplink();

    /// Callback for the @ref SocketClient to handle a newly connected outbound
    /// socket connection.
    ///
    /// @param fd is the connected socket descriptor.
    /// @param on_exit is the Notifiable for when this socket has closed.
    void on_uplink_created(int fd, Notifiable *on_exit);

    /// Enables the esp_wifi logging, including the esp_wifi_internal APIs when
    /// available.
    void enable_esp_wifi_logging();

    /// Initializes the mDNS system if it hasn't already been initialized.
    void start_mdns_system();

    /// Handle for the wifi_manager_task that manages the WiFi stack, including
    /// periodic health checks of the connected hubs or clients.
    os_thread_t wifiTaskHandle_;

    /// Dynamically generated hostname for this node, esp32_{node-id}. This is
    /// also used for the SoftAP SSID name (if enabled).
    std::string hostname_;

    /// User provided SSID to connect to.
    const char *ssid_;

    /// User provided password for the SSID to connect to.
    const char *password_;

    /// Persistent configuration that will be used for this node's WiFi usage.
    const WiFiConfiguration cfg_;

    /// This is internally used to enable the management of the WiFi stack, in
    /// some environments this may be managed externally.
    const bool manageWiFi_;

    /// OpenMRN stack for the Arduino system.
    openlcb::SimpleCanStack *stack_;

    /// WiFi operating mode.
    wifi_mode_t wifiMode_{WIFI_MODE_STA};

    /// Static IP Address configuration for the Station connection.
    tcpip_adapter_ip_info_t *stationStaticIP_{nullptr};

    /// Primary DNS Address to use when configured for Static IP.
    ip_addr_t primaryDNSAddress_{ip_addr_any};

    /// Channel to use for the SoftAP interface.
    uint8_t softAPChannel_{1};

    /// Authentication mode to use for the SoftAP. If not set to WIFI_AUTH_OPEN
    /// @ref softAPPassword_ will be used.
    wifi_auth_mode_t softAPAuthMode_{WIFI_AUTH_OPEN};

    /// User provided password for the SoftAP when active, defaults to
    /// @ref password when null and softAPAuthMode_ is not WIFI_AUTH_OPEN.
    const char *softAPPassword_;

    /// Static IP Address configuration for the SoftAP.
    /// Default static IP provided by ESP-IDF is 192.168.4.1.
    tcpip_adapter_ip_info_t *softAPStaticIP_{nullptr};

    /// Cached copy of the file descriptor passed into apply_configuration.
    /// This is internally used by the wifi_manager_task to processed deferred
    /// configuration load.
    int configFd_{-1};

    /// Calculated CRC-32 of cfg_ data. Used to detect changes in configuration
    /// which may require the wifi_manager_task to reload config.
    uint32_t configCrc32_{0};

    /// Internal flag to request the wifi_manager_task reload configuration.
    bool configReloadRequested_{true};

    /// Internal flag to request the wifi_manager_task to shutdown.
    bool shutdownRequested_{false};

    /// If true, request esp32 wifi to do verbose logging.
    bool verboseLogging_{false};

    /// If true, the esp32 will block startup until the SSID connection has
    /// successfully completed and upon failure (or timeout) the esp32 will be
    /// restarted.
    bool waitForStationConnect_{true};

    /// @ref GcTcpHub for this node's hub if enabled.
    std::unique_ptr<GcTcpHub> hub_;

    /// mDNS service name being advertised by the hub, if enabled.
    std::string hubServiceName_;

    /// @ref SocketClient for this node's uplink.
    std::unique_ptr<SocketClient> uplink_;

    /// Collection of registered WiFi event callback handlers.
    std::vector<std::function<void(system_event_t *)>> eventCallbacks_;

    /// Protects eventCallbacks_ vector.
    OSMutex eventCallbacksLock_;

    /// Internal event group used to track the IP assignment events.
    EventGroupHandle_t wifiStatusEventGroup_;

    /// WiFi SSID scan results holder.
    std::vector<wifi_ap_record_t> ssidScanResults_;

    /// Protects ssidScanResults_ vector.
    OSMutex ssidScanResultsLock_;

    /// Notifiable to be called when SSID scan completes.
    Notifiable *ssidCompleteNotifiable_{nullptr};

    /// Protects the mdnsInitialized_ flag and mdnsDeferredPublish_ map.
    OSMutex mdnsInitLock_;

    /// Internal flag for tracking that the mDNS system has been initialized.
    bool mdnsInitialized_{false};

    /// Internal holder for mDNS entries which could not be published due to
    /// mDNS not being initialized yet.
    std::map<std::string, uint16_t> mdnsDeferredPublish_;

    DISALLOW_COPY_AND_ASSIGN(Esp32WiFiManager);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32WiFiManager;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_

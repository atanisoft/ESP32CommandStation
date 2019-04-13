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
#include "utils/SocketClient.hxx"
#include "utils/SocketClientParams.hxx"
#include "utils/macros.h"

#include <freertos/event_groups.h>

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
    ///
    /// Note: Both ssid and password must remain in memory for the duration of
    /// node uptime.
    Esp32WiFiManager(const char *ssid, const char *password,
        openlcb::SimpleCanStack *stack, const WiFiConfiguration &cfg);

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

    /// Processes an Esp32 WiFi event based on the event_id raised by the
    /// ESP-IDF event loop processor. This should be used when the
    /// Esp32WiFiManager is not managing the WiFi or MDNS systems so that it
    /// can react to WiFi events to cleanup or recreate the hub or uplink
    /// connections as required.
    ///
    /// @param event_id is the system_event_t.event_id value.
    void process_wifi_event(int event_id);

    /// If called, setsthe ESP32 wifi stack to log verbose information to the
    /// ESP32 serial port.
    void enable_verbose_logging();

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

    /// Handle for the wifi_manager_task that manages the WiFi stack, including
    /// periodic health checks of the connected hubs or clients.
    os_thread_t wifiTaskHandle_;

    /// Dynamically generated hostname for this node, esp32_{node-id}.
    std::string hostname_{"esp32_"};

    /// User provided SSID to connect to.
    const char *ssid_;

    /// User provided password for the SSID to connect to.
    const char *password_;

    /// Persistent configuration that will be used for this node's WiFi usage.
    const WiFiConfiguration cfg_;

    /// This is internally used to enable the management of the WiFi stack, in
    /// some environments this may be managed externally.
    const bool manageWiFi_;

    /// OpenMRN stack for the Arduino system
    openlcb::SimpleCanStack *stack_;

    /// Cached copy of the file descriptor passed into apply_configuration.
    /// This is internally used by the wifi_manager_task to processed deferred
    /// configuration load.
    int configFd_{-1};

    /// Calculated CRC-32 of cfg_ data. Used to detect changes in configuration
    /// which may require the wifi_manager_task to reload config.
    uint32_t configCrc32_{0};

    /// Internal flag to request the wifi_manager_task reload configuration.
    bool configReloadRequested_{true};

    /// if true, request esp32 wifi to do verbose logging.
    bool esp32VerboseLogging_{false};

    /// @ref GcTcpHub for this node's hub if enabled.
    std::unique_ptr<GcTcpHub> hub_;

    /// mDNS service name being advertised by the hub, if enabled.
    std::string hubServiceName_{""};

    /// @ref SocketClient for this node's uplink.
    std::unique_ptr<SocketClient> uplink_;

    /// Internal event group used to track the IP assignment events.
    EventGroupHandle_t wifiStatusEventGroup_;

    DISALLOW_COPY_AND_ASSIGN(Esp32WiFiManager);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32WiFiManager;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_

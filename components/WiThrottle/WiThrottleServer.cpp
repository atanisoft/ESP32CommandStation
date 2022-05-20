/** \copyright
 * Copyright (c) 2022, Mike Dunston
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
 * \file WiThrottleServer.cpp
 *
 * Implementation of the WiThrottle Server.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

#include "WiThrottle.hxx"
#include <arpa/inet.h>
#include <hardware.hxx>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/fcntl.h>
#include <sys/socket.h>

#ifdef ESP32

#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>

#endif // ESP32

namespace withrottle
{
    /// Callback for a newly accepted socket connection.
    ///
    /// @param fd is the socket handle.
    static void incoming_connection(int fd)
    {
        Singleton<WiThrottleServer>::instance()->new_connection(fd);
    }

    WiThrottleServer::WiThrottleServer(openlcb::Node *node, MDNS *mdns,
                                       dcc::TrackIf *track,
                                       const string &name,
                                       const string service_name)
        : Service(&executor_),
          node_(node),
          name_(name),
          mdns_(mdns),
          track_(track),
          mdns_service_(service_name),
          executor_(name.c_str(),
                    config_withrottle_server_priority(),
                    config_withrottle_server_stack_size())
    {
    }

#ifdef ESP32
    WiThrottleServer::WiThrottleServer(openlcb::Node *node,
                                       dcc::TrackIf *track,
                                       const string service_name)
        : Service(Singleton<Esp32WiFiManager>::instance()->executor()),
          node_(node),
          name_("withrottle"),
          track_(track),
          mdns_service_(service_name),
          executor_(NO_THREAD()),
          externalExecutor_(true)
    {
        // Hook into the Esp32WiFiManager to start/stop the listener
        // automatically based on the AP/Station interface status.
        Singleton<Esp32WiFiManager>::instance()->register_network_up_callback(
            [&](esp_network_interface_t interface, uint32_t ip)
            {
                start_listener();
            });
        Singleton<Esp32WiFiManager>::instance()->register_network_down_callback(
            [&](esp_network_interface_t interface)
            {
                stop_listener();
            });
    }
#endif // ESP32

    WiThrottleServer::~WiThrottleServer()
    {
        stop_listener();
        if (!externalExecutor_)
        {
            executor_.shutdown();
        }
    }

    void WiThrottleServer::new_connection(int fd)
    {
        struct sockaddr_in source;
        socklen_t source_len = sizeof(sockaddr_in);
        if (getpeername(fd, (sockaddr *)&source, &source_len))
        {
            source.sin_addr.s_addr = 0;
        }
        if (connectionCount_ > config_withrottle_max_connections())
        {
            LOG_ERROR("[%s fd:%d/%s] Connected but maximum concurrent "
                      "connections reached, disconnecting!",
                name_.c_str(), fd,
                ipv4_to_string(ntohl(source.sin_addr.s_addr)).c_str());
            ::close(fd);
            return;
        }
        LOG(INFO,
            "[%s fd:%d/%s] Connected", name_.c_str(), fd,
            ipv4_to_string(ntohl(source.sin_addr.s_addr)).c_str());

        // Set socket receive timeout
        ERRNOCHECK("setsockopt_recv_timeout",
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout_,
                sizeof(struct timeval)));

        // Set socket send timeout
        ERRNOCHECK("setsockopt_send_timeout",
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &socket_timeout_,
                sizeof(struct timeval)));

        // Reconfigure the socket for non-blocking operations
        ::fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);

        // Start the processing flow or close the socket if we fail to allocate
        // the request handler.
        auto *req = new (std::nothrow) WiThrottleClientFlow(
            this, fd, ntohl(source.sin_addr.s_addr), throttleHeartbeat_);
        if (req == nullptr)
        {
            LOG_ERROR("[%s fd:%d/%s] Failed to allocate request handler, closing!",
                name_.c_str(), fd,
                ipv4_to_string(ntohl(source.sin_addr.s_addr)).c_str());
            ::close(fd);
        }
    }

    void WiThrottleServer::configure_heartbeat(uint8_t period)
    {
        throttleHeartbeat_ = period;
    }

    void WiThrottleServer::start_listener()
    {
        if (active_)
        {
            return;
        }

        socket_timeout_.tv_sec = 0;
        socket_timeout_.tv_usec =
            MSEC_TO_USEC(config_withrottle_socket_timeout_ms());

        LOG(INFO, "[%s] Starting WiThrottle listener on port %d",
            name_.c_str(), config_withrottle_default_port());
        listener_.emplace(config_withrottle_default_port(),
            incoming_connection, "WiThrottleSocket");
        active_ = true;
#ifdef ESP32
        Singleton<Esp32WiFiManager>::instance()->mdns_publish(
            mdns_service_, config_withrottle_default_port());
#else // NOT ESP32
        if (mdns_)
        {
            mdns_->publish(name_.c_str(), mdns_service_.c_str(),
                config_withrottle_default_port());
        }
#endif // ESP32
    }

    void WiThrottleServer::stop_listener()
    {
        if (active_)
        {
            LOG(INFO, "[%s] Shutting down WiThrottle listener", name_.c_str());
            listener_.reset();
            active_ = false;
#ifdef ESP32
            Singleton<Esp32WiFiManager>::instance()->mdns_unpublish(mdns_service_.c_str());
#endif // ESP32
        }
    }

} // namespace withrottle
/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file ESPWifiClient.hxx
 *
 * Uses the ESPConn API to connect to a wifi accesspoint, and setup a TCP
 * connection to a gridconnect server.
 *
 * @author Balazs Racz
 * @date 10 Mar 2015
 */

#ifndef _UTILS_ESPWIFICLIENT_HXX_
#define _UTILS_ESPWIFICLIENT_HXX_

extern "C" {
#include "ets_sys.h"
#include <ip_addr.h>
#include <user_interface.h>
#include <osapi.h>
#include <espconn.h>
}

#include "utils/Singleton.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"

/// Uses the ESPConn API on the ESP8266 wifi-enabled MCU to connect to a wifi
/// base station, perform a DNS lookup to a given target, and connect to a
/// given port via TCP. Acts as a HubPort, meaning the data coming from the Hub
/// (usually gridconnect packets) are sent to the TCP connection.
class ESPWifiClient : public Singleton<ESPWifiClient>, private HubPort
{
public:
    /// Creates a wifi+TCP client connection via the ESPConn API.
    ///
    /// @param ssid wifi access point name
    /// @param password passphrase for the wifi access point, or empty string
    /// for open (unencrypted) connection.
    /// @param hub CAN hub to connect to the server
    /// @param hostname hostname of the gridconnect TCP hub server. IP address
    /// in dot format is not supported.
    /// @param port port number of the TCP hub server.
    /// @param send_buf_size in bytes, how much to buffer befone handing over to
    /// the TCP stack.
    /// @param connect_callback will be called after the wifi and the TCP
    /// connection is established. Usually used for starting the OpenLCB stack.
    ESPWifiClient(const string &ssid, const string &password, CanHubFlow *hub,
        const string &hostname, int port, unsigned send_buf_size,
        std::function<void()> connect_callback)
        : HubPort(hub->service())
        , host_(hostname)
        , port_(port)
        , sendPending_(0)
        , sendBlocked_(0)
        , timerPending_(0)
        , sendBuf_((uint8_t*)malloc(send_buf_size))
        , bufSize_(send_buf_size)
        , hub_(hub)
        , gcHub_(hub_->service())
        , connectCallback_(std::move(connect_callback))
    {

        wifi_set_opmode_current(STATION_MODE);
        memcpy(&stationConfig_.ssid, ssid.c_str(), ssid.size() + 1);
        memcpy(&stationConfig_.password, password.c_str(), password.size() + 1);
        wifi_station_set_config(&stationConfig_);
        wifi_set_event_handler_cb(&static_wifi_callback);
        wifi_station_connect(); // may be avoided if the constructor is still
                                // called in the user_init.
    }

    /// Callback when an event happens on the wifi port. @param evt describes
    /// event that happened.
    static void static_wifi_callback(System_Event_t *evt)
    {
        os_printf("%s: %d\n", __FUNCTION__, evt->event);

        switch (evt->event)
        {
            case EVENT_STAMODE_CONNECTED:
            {
                os_printf("connect to ssid %s, channel %d\n",
                    evt->event_info.connected.ssid,
                    evt->event_info.connected.channel);
                break;
            }

            case EVENT_STAMODE_DISCONNECTED:
            {
                os_printf("disconnect from ssid %s, reason %d\n",
                    evt->event_info.disconnected.ssid,
                    evt->event_info.disconnected.reason);

                system_deep_sleep_set_option(0);
                system_deep_sleep(60 * 1000 * 1000); // 60 seconds
                break;
            }

            case EVENT_STAMODE_GOT_IP:
            {
                /*
                os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                    IP2STR(evt->event_info.got_ip.ip),
                    IP2STR(evt->event_info.got_ip.mask),
                    IP2STR(evt->event_info.got_ip.gw));
                    os_printf("\n");*/

                Singleton<ESPWifiClient>::instance()->do_dns_lookup();
                break;
            }

            default:
            {
                break;
            }
        }
    }

    /// Initiates the DNS lookup of the target host.
    void do_dns_lookup()
    {
        espconn_gethostbyname(&conn_, host_.c_str(), &targetIp_, dns_done);
    }

    /// Callback when the DNS lookup is completed.
    ///
    /// @param name what we were looking up (ignored)
    /// @param ipaddr ip address of the host
    /// @param arg ignored.
    ///
    static void dns_done(const char *name, ip_addr_t *ipaddr, void *arg)
    {
        if (ipaddr == NULL)
        {
            os_printf("DNS lookup failed\n");
            wifi_station_disconnect();
            return;
        }
        Singleton<ESPWifiClient>::instance()->do_connect(ipaddr);
    }

    /// Connects to the specific IP address. @param ipaddr is the address of
    /// the host we wanted to connect to.
    void do_connect(ip_addr_t *ipaddr)
    {
        os_printf("Connecting to %s:%d...\n", host_.c_str(), port_);

        conn_.type = ESPCONN_TCP;
        conn_.state = ESPCONN_NONE;
        conn_.proto.tcp = &tcp_;
        conn_.proto.tcp->local_port = espconn_port();
        conn_.proto.tcp->remote_port = port_;
        memcpy(conn_.proto.tcp->remote_ip, &ipaddr->addr, 4);

        espconn_regist_connectcb(&conn_, static_tcp_connected);
        espconn_regist_disconcb(&conn_, static_tcp_disconnected);

        espconn_connect(&conn_);
    }

    /// Callback when the TCP connection is established. @param arg ignored.
    static void static_tcp_connected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_connected();
    }

    /// Callback when the TCP connection is established.
    void tcp_connected()
    {
        gcAdapter_.reset(
            GCAdapterBase::CreateGridConnectAdapter(&gcHub_, hub_, false));
        espconn_regist_recvcb(&conn_, static_data_received);
        espconn_regist_sentcb(&conn_, static_data_sent);
        gcHub_.register_port(this);
        connectCallback_();
    }

    /// Callback when the TCP connection is lost. @param arg ignored.
    static void static_tcp_disconnected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_disconnected();
    }

    /// Callback when the TCP connection is lost.
    void tcp_disconnected()
    {
        gcHub_.unregister_port(this);
        gcAdapter_->shutdown();
        wifi_station_disconnect();
    }

    /// Callback for incoming data.
    ///
    /// @param arg ignored
    /// @param pdata pointer to data received.
    /// @param len number of bytes received.
    static void static_data_received(void *arg, char *pdata, unsigned short len)
    {
        Singleton<ESPWifiClient>::instance()->data_received(pdata, len);
    }

    /// Callback when data that was requested to be sent has completed
    /// sending. @param arg ignored.
    static void static_data_sent(void *arg)
    {
        Singleton<ESPWifiClient>::instance()->data_sent();
    }

    /// Callback when incoming data is received.
    ///
    /// @param pdata incoming data
    /// @param len number of bytes received.
    ///
    void data_received(char *pdata, unsigned short len)
    {
        auto *b = gcHub_.alloc();
        b->data()->skipMember_ = this;
        b->data()->assign(pdata, len);
        gcHub_.send(b);
    }

private:
    /// Sending base state. @return next action.
    Action entry() override
    {
        if (sendPending_)
        {
            sendBlocked_ = 1;
            // Will call again once the wifi notification comes back.
            return wait();
        }
        if (message()->data()->size() > bufSize_)
        {
            if (bufEnd_ > 0)
            {
                // Cannot copy the data to the buffer. Must send separately.
                send_buffer();
                return again();
            }
            else
            {
                sendPending_ = 1;
                sendBlocked_ = 1; // will cause notify.
                espconn_sent(&conn_, (uint8 *)message()->data()->data(),
                    message()->data()->size());
                return wait_and_call(STATE(send_done));
            }
        }
        if (message()->data()->size() > bufSize_ - bufEnd_)
        {
            // Doesn't fit into the current buffer.
            send_buffer();
            return again();
        }
        // Copies the data into the buffer.
        memcpy(sendBuf_ + bufEnd_, message()->data()->data(),
            message()->data()->size());
        bufEnd_ += message()->data()->size();
        release();
        // Decides whether to send off the buffer now.
        if (!queue_empty())
        {
            // let's process more of the queue
            return exit();
        }
        if (!timerPending_)
        {
            timerPending_ = 1;
            bufferTimer_.start(MSEC_TO_NSEC(3));
        }
        return exit();
    }

    /// Called from the timer to signal sending off the buffer's contents.
    void timeout()
    {
        timerPending_ = 0;
        if (!sendPending_) {
            send_buffer();
        }
    }

    /// Callback state when we are sending directly off of the input buffer
    /// becuase the data payload is too much to fit into the send assembly
    /// buffer. Called when the send is completed and the input buffer can be
    /// releases. @return next action
    Action send_done()
    {
        return release_and_exit();
    }

    /// Writes all bytes that are in the send buffer to the TCP socket.
    void send_buffer()
    {
        espconn_sent(&conn_, sendBuf_, bufEnd_);
        sendPending_ = 1;
    }

    /// Callback from the TCP stack when the data send has been completed.
    void data_sent()
    {
        sendPending_ = 0;
        bufEnd_ = 0;
        if (sendBlocked_)
        {
            sendBlocked_ = 0;
            notify();
        }
    }

    /// Timer that triggers the parent flow when expiring. Used to flush the
    /// accumulated gridconnect bytes to the TCP socket.
    class BufferTimer : public ::Timer
    {
    public:
        /// Constructor. @param parent who owns *this.
        BufferTimer(ESPWifiClient *parent)
            : Timer(parent->service()->executor()->active_timers())
            , parent_(parent)
        {
        }

        /// callback when the timer expires. @return do not restart timer.
        long long timeout() override
        {
            parent_->timeout();
            return NONE;
        }
    private:
        ESPWifiClient *parent_; ///< parent who owns *this.
    } bufferTimer_{this}; ///< Instance of the timer we own.

    /// Configuration of the access pint we are connecting to.
    struct station_config stationConfig_;
    /// IP (including DNS) connection handle.
    struct espconn conn_;
    /// TCP connection handle.
    esp_tcp tcp_;

    /// IP address of the target host.
    ip_addr_t targetIp_;
    /// Target host we are trying to connect to.
    string host_;
    /// Port numer we are connecting to on the target host.
    int port_;
    /// True when the TCP stack is busy.
    uint16_t sendPending_ : 1;
    /// True when we are waiting for a notification from the TCP stack send done
    /// callback.
    uint16_t sendBlocked_ : 1;
    /// True when there is a send timer running with the assembly buffer being
    /// not full.
    uint16_t timerPending_ : 1;

    /// Offset in sendBuf_ of the first unused byte.
    uint16_t bufEnd_ : 16;
    /// Temporarily stores outgoing data until the TCP stack becomes free.
    uint8_t *sendBuf_;
    /// How many bytes are there in the send buffer.
    unsigned bufSize_;
    /// CAN hub to send incoming packets to and to receive outgoing packets
    /// from.
    CanHubFlow *hub_;
    /// String-typed hub for the gridconnect-rendered packets.
    HubFlow gcHub_;
    /// Transcoder bridge from CAN to GridConnect protocol.
    std::unique_ptr<GCAdapterBase> gcAdapter_;
    /// Application level callback function to call when the connection has
    /// been successfully established.
    std::function<void()> connectCallback_;
};

#endif // _UTILS_ESPWIFICLIENT_HXX_

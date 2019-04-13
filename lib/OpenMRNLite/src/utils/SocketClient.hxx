/** @copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * @file SocketClient.hxx
 *
 * Connects to a remote TCP socket.
 *
 * @author Stuart Baker
 * @date 11 March 2017
 */

#ifndef _UTILS_SOCKET_CLIENT_HXX_
#define _UTILS_SOCKET_CLIENT_HXX_

/// @todo(balazs.racz) remove this by moving all calls to usleep to the .cxx file.
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include <functional>
#include <netdb.h>
#ifndef ESP32 // this doesn't exist on the ESP32 with LWiP
#include <arpa/inet.h>
#endif
#include <fcntl.h>
#include <ifaddrs.h>

#include "executor/StateFlow.hxx"
#include "executor/Timer.hxx"
#include "os/MDNS.hxx"
#include "utils/Atomic.hxx"
#include "utils/SocketClientParams.hxx"
#include "utils/format_utils.hxx"

class SocketClient : public StateFlowBase, private Atomic
{
public:
    /** Constructor.
     * @param service service that the StateFlowBase will be bound to. This
     * service will never be blocked. Externally owned.
     * @param connect_executor is a thread on which DNS lookups and blocking
     * connect calls will be attempted. Externally owned. May be shared between
     * different SocketClients.
     * @param mdns_executor is a thread on which mdns lookups will be
     * attempted. May be null if mdns is never used (by the parameters) or may
     * be the same executor as connect_executor if connect and mdns attempts
     * ought to be serialized. Externally owned. May be shared between
     * different SocketClients.
     * @param params defines all the different parameters on whom to connect
     * to. Takes ownership.
     * @param connect_callback callback method to invoke when a client
     * connection is made successfully. It is unspecified which thread this
     * callback will be invoked upon.
     * - First param is the file descriptor of the resulting socket
     * - Second param is a notifiable (ownership is not transferred). The
     * callee must invoke this notifiable when the socket has been torn down in
     * order to restart the search with the same parameters.
     */
    SocketClient(Service *service, ExecutorBase *connect_executor,
        ExecutorBase *mdns_executor, std::unique_ptr<SocketClientParams> params,
        std::function<void(int, Notifiable *)> connect_callback)
        : StateFlowBase(service)
        , callback_(connect_callback)
        , connectExecutor_(connect_executor)
        , mdnsExecutor_(mdns_executor)
        , strategyOffset_(0)
        , mdnsPending_(false)
        , mdnsJoin_(false)
        , sleeping_(false)
        , requestShutdown_(false)
        , isConnected_(false)
        , fd_(-1)
    {
        reset_params(std::move(params));
        start_flow(STATE(start_connection));
    }

    /** Destructor.
     */
    ~SocketClient()
    {
        shutdown();
    }

    /// Helper structure for creating a unique_ptr for struct addrinfo pointers.
    struct AddrInfoDeleter
    {
        void operator()(struct addrinfo *s)
        {
            if (s)
            {
                freeaddrinfo(s);
            }
        }
    };

    /// Custom unique pointer that knows how to delete a struct addrinfo.
    typedef std::unique_ptr<struct addrinfo, AddrInfoDeleter> AddrinfoPtr;

    /// This enum represents individual states of this state flow that we can
    /// branch to. The configuration of connection strategy is a sequence of
    /// these enum values.
    enum class Attempt : uint8_t
    {
        /// Connect to the reconnect slot.
        RECONNECT,
        /// Start mDNS lookup.
        INITIATE_MDNS,
        /// Connect to mDNS lookup result.
        CONNECT_MDNS,
        /// Connect to static target.
        CONNECT_STATIC,
        /// Attempt complete. Start again.
        WAIT_RETRY,
    };

    /// Updates the parameter structure for this socket client.
    /// @param params is the parameter structure; ownership will be
    /// transferred.
    void reset_params(std::unique_ptr<SocketClientParams> params)
    {
        params_ = std::move(params);
        /// @todo (balazs.racz): do we need to somehow wake up the flow and
        /// make it attempt to reconnect?
    }

    /** Shutdown the client so that it can be deleted.
     */
    void shutdown()
    {
        LOG(VERBOSE,
            "socketclient: sync shutdown isconn %d slp %d rqshut %d mdnsP %d"
            "mdnsJ %d isShut %d",
            isConnected_, sleeping_, requestShutdown_, mdnsPending_, mdnsJoin_,
            is_shutdown());
        start_shutdown();
        while (true)
        {
            AtomicHolder h(this);
            if (!mdnsPending_)
            {
                break;
            }
        }
        while (!is_terminated())
        {
            usleep(1000);
        }
    }

    /// @return true if we have a working connection.
    bool is_connected()
    {
        AtomicHolder h(this);
        return isConnected_;
    }

    /// @return true if the shutdown has completed.
    bool is_shutdown()
    {
        return is_terminated();
    }

    /** Request that this client shutdown and exit the other thread.
     */
    void start_shutdown()
    {
        LOG(VERBOSE,
            "socketclient: start shutdown isconn %d slp %d rqshut %d mdnsP %d"
            "mdnsJ %d isShut %d",
            isConnected_, sleeping_, requestShutdown_, mdnsPending_, mdnsJoin_,
            is_shutdown());
        {
            AtomicHolder h(this);
            if (requestShutdown_)
            {
                return;
            }
            requestShutdown_ = true;
            if (sleeping_)
            {
                sleeping_ = false;
                timer_.ensure_triggered();
            }
            if (isConnected_)
            {
                isConnected_ = false;
                notify();
            }
        }
        // NOTE: It would be nice to abort any pending asynchronous tasks we
        // have such as a connect or a getaddrinfo call running on the
        // secondary executors. However, there is no API to do that right now.
    }

    /** Connects a tcp socket to the specified remote host:port. Returns -1 if
     *  unsuccessful; returns the fd if successful.
     *
     *  @param host hostname to connect to
     *  @param port TCP port number to connect to
     *
     *  @return fd of the connected socket.
     */
    static int connect(const char *host, int port)
    {
        return connect(host, integer_to_string(port).c_str());
    }

    /** Connects a tcp socket to the specified remote host:port. Returns -1 if
     *  unsuccessful; returns the fd if successful.
     *
     *  @param host hostname to connect to. Shall be null for mDNS target.
     *  @param port_str TCP port number or mDNS hostname to connect to.
     *
     *  @return fd of the connected socket.
     */
    static int connect(const char *host, const char* port_str);

    /** Connects a tcp socket to the specified remote address. Returns -1 if
     *  unsuccessful; returns the fd if successful.
     *
     *  @param addr IP(v4/v6) addrinfo structure describing the remote host. May
     *  be null. Ownership is not transferred. Will not be used after the
     *  funciton returns.
     *
     *  @return fd of the connected socket.
     */
    static int connect(struct addrinfo *addr);

    /// Converts a struct addrinfo to a dotted-decimal notation IP address.
    /// @param addr is an addrinfo returned by getaddrinfo or gethostbyname.
    /// @param host will be filled with dotted-decimal IP address.
    /// @param port will be filled with the port number.
    /// @return true on success.
    static bool address_to_string(
        struct addrinfo *addr, string *host, int *port);

    /// Converts a hostname string and port number to a struct addrinfo.
    /// @param host hostname to connect to.
    /// @param port port number to connect to.
    /// @return a struct addrinfo; ownership is transferred.
    static AddrinfoPtr string_to_address(const char *host, int port)
    {
        return string_to_address(host, integer_to_string(port).c_str());
    }

    /// Converts a hostname string (or null) and port number (or service name)
    /// to a struct addrinfo.
    /// @param host hostname to connect to.
    /// @param port port name or service name to connect to.
    /// @return a struct addrinfo; ownership is transferred to the caller.
    static AddrinfoPtr string_to_address(
        const char *host, const char *port_str);

private:
    /// Parses the params_ configuration and fills in strategyConfig_.
    void prepare_strategy()
    {
        unsigned ofs = 0;
        auto search = params_->search_mode();
        // If we only have one extra thread, we initiate mdns only at the time
        // we are trying to connect to it. If we have two, we start the lookup
        // at the beginning.
        bool mdns_ahead = (connectExecutor_ != mdnsExecutor_);
        if (mdns_ahead && search != SocketClientParams::MANUAL_ONLY)
        {
            strategyConfig_[ofs++] = Attempt::INITIATE_MDNS;
        }
        if (params_->enable_last())
        {
            strategyConfig_[ofs++] = Attempt::RECONNECT;
        }
        switch (search)
        {
            case SocketClientParams::AUTO_MANUAL:
                if (!mdns_ahead)
                {
                    strategyConfig_[ofs++] = Attempt::INITIATE_MDNS;
                }
                strategyConfig_[ofs++] = Attempt::CONNECT_MDNS;
                strategyConfig_[ofs++] = Attempt::CONNECT_STATIC;
                break;
            case SocketClientParams::MANUAL_AUTO:
                strategyConfig_[ofs++] = Attempt::CONNECT_STATIC;
                if (!mdns_ahead)
                {
                    strategyConfig_[ofs++] = Attempt::INITIATE_MDNS;
                }
                strategyConfig_[ofs++] = Attempt::CONNECT_MDNS;
                break;
            case SocketClientParams::MANUAL_ONLY:
                strategyConfig_[ofs++] = Attempt::CONNECT_STATIC;
                break;
            case SocketClientParams::AUTO_ONLY:
                if (!mdns_ahead)
                {
                    strategyConfig_[ofs++] = Attempt::INITIATE_MDNS;
                }
                strategyConfig_[ofs++] = Attempt::CONNECT_MDNS;
                break;
        }
        strategyConfig_[ofs++] = Attempt::WAIT_RETRY;
        HASSERT(ofs <= strategyConfig_.size());
    }

    /// Main entry point of the connection process.
    Action start_connection()
    {
        startTime_ = os_get_time_monotonic();
        prepare_strategy();
        {
            AtomicHolder h(this);
            strategyOffset_ = 0;
            mdnsPending_ = false;
            mdnsJoin_ = false;
            isConnected_ = false;
            mdnsAddr_.reset();
        }
        return call_immediately(STATE(next_step));
    }

    /// Execute the next step of the strategy.
    Action next_step()
    {
        Attempt a = Attempt::WAIT_RETRY;
        if (strategyOffset_ < strategyConfig_.size())
        {
            AtomicHolder h(this);
            if (requestShutdown_)
            {
                return exit();
            }
            a = strategyConfig_[strategyOffset_++];
        }
        switch (a)
        {
            default:
                DIE("Unexpected action");
            case Attempt::WAIT_RETRY:
                return wait_retry();
            case Attempt::RECONNECT:
                return try_schedule_connect(SocketClientParams::CONNECT_RE,
                    params_->last_host_name(), params_->last_port());
            case Attempt::INITIATE_MDNS:
                return start_mdns();
            case Attempt::CONNECT_MDNS:
                return wait_and_connect_mdns();
            case Attempt::CONNECT_STATIC:
                return try_schedule_connect(SocketClientParams::CONNECT_MANUAL,
                    params_->manual_host_name(), params_->manual_port());
        }
    }

    /// Helper function to schedule asynchronous connect on a separate
    /// executor. Never blocks. Will deliver exactly one notify to the barrier
    /// notifiable n_.
    /// @param log will be emitted to the params_ structure if connection is
    /// attempted.
    /// @param host hostname (or IP address in text form) to connect to. May be
    /// empty in which case no connection will be attempted.
    /// @param port port number to connect to.
    /// @return the connection wait action or next_state depending on whether
    /// connection was attempted.
    Action try_schedule_connect(
        SocketClientParams::LogMessage log, string host, int port)
    {
        if (port <= 0 || host.empty())
        {
            return call_immediately(STATE(next_step));
        }
        string v = host;
        v += ':';
        v += integer_to_string(port);
        params_->log_message(log, v);
        fd_ = -1;
        n_.reset(this);
        connectExecutor_->add(new CallbackExecutable(
            [this, host, port]() { connect_blocking(host, port); }));
        return wait_and_call(STATE(connect_complete));
    }

    /// Called on the connect executor.
    /// @param host hostname (or IP address in text form) to connect to
    /// @param port port number to connect to.
    void connect_blocking(const string &host, int port)
    {
        AutoNotify an(&n_);
        auto addr = SocketClient::string_to_address(host.c_str(), port);
        if (params_->disallow_local() && local_test(addr.get()))
        {
            params_->log_message(SocketClientParams::CONNECT_FAILED_SELF);
            return;
        }
        fd_ = SocketClient::connect(addr.get());
        if (fd_ >= 0)
        {
            params_->set_last(host.c_str(), port);
        }
    }

    /// State that gets invoked once the reconnect attempt is complete.
    Action connect_complete()
    {
        if (fd_ < 0)
        {
            // reconnect failed
            return next_step();
        }
        else
        {
            // we have a connection.
            return connected();
        }
    }

    /// State that gets called when we have a completed connection in fd_.
    Action connected()
    {
        {
            AtomicHolder h(this);
            isConnected_ = true;
        }
        callback_(fd_, this);
        return wait_and_call(STATE(start_connection));
    }

    /// Turns a parameter to a string.
    /// @param p is a parameter; may be nullptr or empty.
    /// @return empty string if p is null or empty, otherwise p (copied).
    string to_string(const char *p)
    {
        if (!p || !*p)
        {
            return string();
        }
        return p;
    }

    /// State that initiates the mdns lookup asynchronously.
    /// @return next step state.
    Action start_mdns()
    {
        HASSERT(mdnsExecutor_);
        mdnsAddr_.reset();
        string srv = params_->mdns_service_name();
        string host = params_->mdns_host_name();
        if (!srv.empty())
        {
            {
                AtomicHolder h(this);
                mdnsPending_ = true;
            }
            mdnsExecutor_->add(new CallbackExecutable(
                [this, host, srv]() { mdns_lookup(host, srv); }));
        }
        return call_immediately(STATE(next_step));
    }

    /// Synchronous function that runs on the mdns executor. Performs the
    /// lookup.
    /// @param mdns_service is the service name to look up.
    /// @param mdns_hostname is ignored for now.
    void mdns_lookup(string mdns_hostname, string mdns_service)
    {
        int ai_ret = -1;
        struct addrinfo hints;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = 0;
        hints.ai_protocol = IPPROTO_TCP;

        struct addrinfo *addr = nullptr;
        params_->log_message(SocketClientParams::MDNS_SEARCH, mdns_service);
        ai_ret = MDNS::lookup(mdns_service.c_str(), &hints, &addr);
        mdnsAddr_.reset(addr); // will take care of freeing it.
        if (ai_ret != 0 || addr == nullptr)
        {
            params_->log_message(SocketClientParams::MDNS_NOT_FOUND);
            // LOG(INFO, "mdns lookup for %s failed.", mdns_service.c_str());
        }
        else
        {
            params_->log_message(SocketClientParams::MDNS_FOUND);
        }
        {
            AtomicHolder h(this);
            mdnsPending_ = false;
            if (mdnsJoin_)
            {
                // Flow is waiting for mdns result.
                mdnsJoin_ = false;
                notify();
            }
        }
    }

    /// Blocks the flow until mdns lookup is complete, then connects to the
    /// resulting address.
    /// @return mdns connect step
    Action wait_and_connect_mdns()
    {
        {
            AtomicHolder h(this);
            if (mdnsPending_)
            {
                mdnsJoin_ = true;
                return wait_and_call(STATE(connect_mdns));
            }
        }
        return call_immediately(STATE(connect_mdns));
    }

    /// Takes the address from the mdns lookup and connects to it if it is
    /// valid.
    /// @return next step or pending connection step.
    Action connect_mdns()
    {
        string host;
        int port = -1;
        if (!mdnsAddr_.get() ||
            !SocketClient::address_to_string(mdnsAddr_.get(), &host, &port))
        {
            // no address to connect to.
            return call_immediately(STATE(next_step));
        }
        return try_schedule_connect(
            SocketClientParams::CONNECT_MDNS, std::move(host), port);
    }

    /// Last state in the connection sequence, when everything failed: sleeps
    /// until the timeout specified in the params, then goes back to the
    /// start_connection.
    Action wait_retry()
    {
        {
            AtomicHolder h(this);
            if (requestShutdown_)
            {
                return exit();
            }
            sleeping_ = true;
        }
        long long end_time = startTime_ + SEC_TO_NSEC(params_->retry_seconds());
        timer_.start_absolute(end_time);
        return wait_and_call(STATE(sleep_done));
    }

    Action sleep_done()
    {
        {
            AtomicHolder h(this);
            sleeping_ = false;
        }
        return call_immediately(STATE(start_connection));
    }

    /** Test if a given address is local.
     * @param addr address info to test
     * @return true if local, else false if not local
     */
    static bool local_test(struct addrinfo *addr);

    /// When the last connection attempt was started.
    long long startTime_;
    /// Helper for sleeping.
    StateFlowTimer timer_{this};

    /// Stores the parameter structure.
    std::unique_ptr<SocketClientParams> params_;

    /// callback to call on connection success
    std::function<void(int, Notifiable *)> callback_ = nullptr;

    /// Executor for synchronous (blocking) connect calls. Externally owned.
    ExecutorBase *connectExecutor_ = nullptr;
    /// Executor for synchronous (blocking) mDNS lookup calls. Externally
    /// owned, may be null.
    ExecutorBase *mdnsExecutor_ = nullptr;

    /// Stores the sequence of operations we need to try.
    std::array<Attempt, 5> strategyConfig_{{
        Attempt::WAIT_RETRY,
    }};
    /// What is the next step in the strategy. Index into the strategyConfig_
    /// array. Guarded by Atomic *this.
    uint8_t strategyOffset_ : 3;

    /// true if there is a pending mdns lookup operation. Guarded by
    /// Atomic *this.
    uint8_t mdnsPending_ : 1;

    /// true if the main flow is waiting for the mdns lookup to
    /// complete. Guarded by Atomic *this.
    uint8_t mdnsJoin_ : 1;

    /// true while we are waiting for the timer.
    uint8_t sleeping_ : 1;

    /// true if an external agent requested the flow to exit.
    uint8_t requestShutdown_ : 1;

    /// true if we are connected and waiting for a client notification to
    /// restart.
    uint8_t isConnected_ : 1;

    /// Holds the results of the mdns lookup. null if failed (or never ran).
    AddrinfoPtr mdnsAddr_;

    BarrierNotifiable n_;

    /** socket descriptor */
    int fd_;

    DISALLOW_COPY_AND_ASSIGN(SocketClient);
};

#endif /* _UTILS_SOCKET_CLIENT_HXX */


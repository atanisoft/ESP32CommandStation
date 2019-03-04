/** \copyright
 * Copyright (c) 2019, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file SocketClientParams.hxx
 *
 * Parameter structure for socket client.
 *
 * @author Balazs Racz
 * @date 6 Jan 2019
 */

#ifndef _UTILS_SOCKETCLIENTPARAMS_HXX_
#define _UTILS_SOCKETCLIENTPARAMS_HXX_

/// Abstract interface to represent parameters to the socket client. This
/// interface can be implemented just purely based on compiled-in values,
/// filled in from optargs or wrapping a configuration location in an openlcb
/// config space.
///
/// Implementations must be thread-safe (i.e. do locking internally; all
/// functions of this class may be called from unspecified threads including
/// also concurrently).
class SocketClientParams
{
public:
    /// @return a new socket client params instance that allows only manual
    /// connection.
    /// @param hostname is the manual hostname
    /// @param port is the manual port number
    static std::unique_ptr<SocketClientParams> from_static(
        string hostname, int port);

    /// @return a new socket client params instance that allows only manual
    /// connection.
    /// @param hostname is the manual hostname
    /// @param port is the manual port number
    /// @param mdns_service is the mdns service name
    static std::unique_ptr<SocketClientParams> from_static_and_mdns(
        string hostname, int port, string mdns_service);

    virtual ~SocketClientParams()
    {
    }

    /// Parameter that determines what order we will attempt to connect.
    enum SearchMode
    {
        /// Try mDNS first, then (if it failed) try manual address.
        AUTO_MANUAL = 0,
        /// Try manual address first, then (if it failed) try mDNS.
        MANUAL_AUTO = 1,
        /// Try only mDNS, ignore manual address.
        AUTO_ONLY = 2,
        /// Try only manual address, never try mDNS lookups.
        MANUAL_ONLY = 3
    };

    /// @return search mode for how to locate the server.
    virtual SearchMode search_mode() = 0;

    /// @return the service name to use for mDNS lookup; nullptr or empty
    /// string if mdns is not to be used.
    virtual string mdns_service_name() = 0;

    /// @return null or empty string if any mdns server is okay to connect
    /// to. If nonempty, then only an mdns server will be chosen that has the
    /// specific host name.
    virtual string mdns_host_name() = 0;

    /// @return null or empty string if no manual address is
    /// configured. Otherwise a dotted-decimal IP address or a DNS hostname
    /// (not mDNS) for manual address to connect to.
    virtual string manual_host_name() = 0;

    /// @return port number to use for manual connection.
    virtual int manual_port() = 0;

    /// @return true if first attempt should be to connect to
    /// last_host_name:last_port.
    virtual bool enable_last() = 0;

    /// @return the last successfully used IP address, as dotted
    /// decimal. Nullptr or empty if no successful connection has ever been
    /// made.
    virtual string last_host_name() = 0;

    /// @return the last successfully used port number.
    virtual int last_port() = 0;

    /// This function is called on an unspecified thread when a connection is
    /// successfully established.
    /// @param hostname is filled with a dotted decimal representation of the
    /// connected remote host when the connection succeeds.
    /// @param port is the TCP port number.
    virtual void set_last(const char *hostname, int port)
    {
    }

    /// Enum for sending connection status updates to the caller.
    enum LogMessage
    {
        CYCLE_START = 0,
        /// Attempting to reconnect. Arg is host:port.
        CONNECT_RE = 1,
        /// Starting mDNS lookup. Argument: mdns [hostname.]service_name
        MDNS_SEARCH = 2,
        /// mDNS lookup failed.
        MDNS_NOT_FOUND = 3,
        /// mDNS lookup suceeded.
        MDNS_FOUND = 4,
        /// Connecting to mDNS target. Arg is host:port.
        CONNECT_MDNS = 5,
        /// Connecting to manual target. Arg is hostname:port.
        CONNECT_MANUAL = 6,
        /// Connection dropped because target is localhost.
        CONNECT_FAILED_SELF = 7,
        CONNECTION_LOST
    };

    /// Notifies the caller about the current phase of the connection. This
    /// function will be called from an unspecified thread, so the callee is
    /// responsible for locking.
    /// @param id is the enum of the message to emit.
    /// @param arg is a parameter to the message (usually hostname, IP address
    /// etc. Sometimes empty).
    virtual void log_message(LogMessage id, const string &arg = string())
    {
    }

    /// @return Minimum time to spend between failed connection attempts.
    virtual int retry_seconds()
    {
        return 5;
    }

    /// @return How long to wait for connections to be established.
    virtual int timeout_seconds()
    {
        return 255;
    }

    /// @return true if we should actively skip connections that happen to
    /// match our own IP address.
    virtual bool disallow_local()
    {
        return false;
    }
};

/// Default implementation that supplies no connection method.
class EmptySocketClientParams : public SocketClientParams
{
public:
    /// @return search mode for how to locate the server.
    SearchMode search_mode() override
    {
        return AUTO_MANUAL;
    }

    /// @return the service name to use for mDNS lookup; nullptr or empty
    /// string if mdns is not to be used.
    string mdns_service_name() override
    {
        return string();
    }

    /// @return null or empty string if any mdns server is okay to connect
    /// to. If nonempty, then only an mdns server will be chosen that has the
    /// specific host name.
    string mdns_host_name() override
    {
        return string();
    }

    /// @return null or empty string if no manual address is
    /// configured. Otherwise a dotted-decimal IP address or a DNS hostname
    /// (not mDNS) for manual address to connect to.
    string manual_host_name() override
    {
        return string();
    }

    /// @return port number to use for manual connection.
    int manual_port() override
    {
        return -1;
    }

    /// @return true if first attempt should be to connect to
    /// last_host_name:last_port.
    bool enable_last() override
    {
        return false;
    }

    /// @return the last successfully used IP address, as dotted
    /// decimal. Nullptr or empty if no successful connection has ever been
    /// made.
    string last_host_name() override
    {
        return string();
    }

    /// @return the last successfully used port number.
    int last_port() override
    {
        return -1;
    }
};

/// Default implementation that supplies parametrized values for static and
/// mdns connection methods.
class DefaultSocketClientParams : public EmptySocketClientParams
{
public:
    /// @return the service name to use for mDNS lookup; nullptr or empty
    /// string if mdns is not to be used.
    string mdns_service_name() override
    {
        return mdnsService_;
    }

    /// @return null or empty string if no manual address is
    /// configured. Otherwise a dotted-decimal IP address or a DNS hostname
    /// (not mDNS) for manual address to connect to.
    string manual_host_name() override
    {
        return staticHost_;
    }

    /// @return port number to use for manual connection.
    int manual_port() override
    {
        return staticPort_;
    }

protected:
    friend std::unique_ptr<SocketClientParams> SocketClientParams::from_static(
        string hostname, int port);
    friend std::unique_ptr<SocketClientParams>
    SocketClientParams::from_static_and_mdns(
        string hostname, int port, string mdns_service);

    /// what to return for manual_host_name
    string staticHost_;
    /// what to return for manual_port
    int staticPort_ = -1;
    /// what to return for mdns_service_name
    string mdnsService_;
};

#endif // _UTILS_SOCKETCLIENTPARAMS_HXX_

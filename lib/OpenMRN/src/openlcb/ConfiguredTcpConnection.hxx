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
 * \file ConfiguredTcpConnection.hxx
 *
 * Persistent configuration for an (outgoing) TCP connection.
 *
 * @author Balazs Racz
 * @date 6 Jan 2019
 */

#ifndef _OPENLCB_CONFIGUREDTCPCONNCTION_HXX_
#define _OPENLCB_CONFIGUREDTCPCONNCTION_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace openlcb
{

/// This structure shows what parameters are configurable in the
/// TcpClientConfig CDI group definition. The parameters here will be used in
/// the CDI rendering of the group; many of them are part of the UI presented
/// by the XML and thus subject to localization possibilities.
class TcpClientDefaultParams
{
public:
    /// Visible name of the search_mode field.
    static constexpr const char *SEARCH_MODE_NAME = "Search mode";
    /// Visible description of the search_mode field.
    static constexpr const char *SEARCH_MODE_DESCR =
        "Defines the order of how to locate the server to connect to. 'auto' "
        "uses the mDNS protocol to find the IP address automatically. 'manual' "
        "uses the IP address entered in this settings.";
    /// <map> of possible keys and descriptive values to show to the user for
    /// the search_mode field.
    static constexpr const char *SEARCH_MODE_MAP =
        "<relation><property>0</property><value>auto, manual</value></relation>"
        "<relation><property>1</property><value>manual, auto</value></relation>"
        "<relation><property>2</property><value>auto only</value></relation>"
        "<relation><property>3</property><value>manual only</value></relation>";

    /// Visible name of the manual address group.
    static constexpr const char *MANUAL_ADDRESS_NAME = "Manual address";
    /// Visible description of the manual address group.
    static constexpr const char *MANUAL_ADDRESS_DESCR =
        "Set IP address here if auto-detection does not work.";

    /// Visible name of the ip address field inside the manual address group.
    static constexpr const char *IP_ADDRESS_NAME = "IP address";
    /// Visible description of the ip address field inside the manual address
    /// group.
    static constexpr const char *IP_ADDRESS_DESCR =
        "Enter the server IP address. Example: 192.168.0.55";

    /// Default value for the port number field. This will also be used for
    /// factory reset code, not just for user documentation.
    static constexpr int DEFAULT_PORT = 12021;
    /// Visible name of the port number field.
    static constexpr const char *PORT_NAME = "Port number";
    /// Visible description of the port number field.
    static constexpr const char *PORT_DESCR =
        "TCP port number of the server. Most of the time this does not need to "
        "be changed.";

    /// Visible name of the auto address group.
    static constexpr const char *AUTO_ADDRESS_NAME = "Auto address";
    /// Visible description of the auto address group.
    static constexpr const char *AUTO_ADDRESS_DESCR =
        "Advanced settings for the server IP address auto-detection (mDNS).";

    /// Visible name of the service name field in the auto address group.
    static constexpr const char *SERVICE_NAME = "mDNS service";
    /// Visible description of the service name field in the auto address
    /// group.
    static constexpr const char *SERVICE_DESCR =
        "mDNS or Bonjour service name, such as _openlcb-can._tcp";

    /// Visible name of the host name field in the auto address group.
    static constexpr const char *HOST_NAME = "Only hostname";
    /// Visible description of the host name field in the auto address group.
    static constexpr const char *HOST_DESCR =
        "Use when multiple servers provide the same service on the network. If "
        "set, selects this specific host name; the connection will fail if "
        "none of the servers have this hostname (use correct capitalization!). "
        "Example: My JMRI Railroad";

    /// Visible name of the reconnect enable field.
    static constexpr const char *RECONNECT_NAME = "Reconnect";
    /// Visible description of the reconnect enable field.
    static constexpr const char *RECONNECT_DESCR =
        "If enabled, tries the last known good IP address before searching for "
        "the server.";
    /// <map> of possible keys and descriptive values to show to the user for
    /// the reconnect field.
    static constexpr const char *RECONNECT_MAP =
        "<relation><property>0</property><value>disabled</value></relation>"
        "<relation><property>1</property><value>enabled</value></relation>";
};

template <class LocalParams> CDI_GROUP(TcpManualAddress);
CDI_GROUP_ENTRY(ip_address, StringConfigEntry<32>,
    Name(LocalParams::IP_ADDRESS_NAME),
    Description(LocalParams::IP_ADDRESS_DESCR));
CDI_GROUP_ENTRY(port, Uint16ConfigEntry, Name(LocalParams::PORT_NAME),
    Description(LocalParams::PORT_DESCR), Min(1), Max(65535),
    Default(LocalParams::DEFAULT_PORT));
CDI_GROUP_END();

template <class LocalParams> CDI_GROUP(TcpAutoAddress);
CDI_GROUP_ENTRY(service_name, StringConfigEntry<48>,
    Name(LocalParams::SERVICE_NAME), Description(LocalParams::SERVICE_DESCR));
CDI_GROUP_ENTRY(host_name, StringConfigEntry<48>, Name(LocalParams::HOST_NAME),
    Description(LocalParams::HOST_DESCR));
CDI_GROUP_END();

template <class LocalParams> CDI_GROUP(TcpClientConfig);
// This is of type SocketClientParams::SearchMode.
CDI_GROUP_ENTRY(search_mode, Uint8ConfigEntry,
    Name(LocalParams::SEARCH_MODE_NAME),
    Description(LocalParams::SEARCH_MODE_DESCR), Min(0), Max(3), Default(0),
    MapValues(LocalParams::SEARCH_MODE_MAP));
CDI_GROUP_ENTRY(manual_address, TcpManualAddress<LocalParams>,
    Name(LocalParams::MANUAL_ADDRESS_NAME),
    Description(LocalParams::MANUAL_ADDRESS_DESCR));
CDI_GROUP_ENTRY(auto_address, TcpAutoAddress<LocalParams>,
    Name(LocalParams::AUTO_ADDRESS_NAME),
    Description(LocalParams::AUTO_ADDRESS_DESCR));
CDI_GROUP_ENTRY(reconnect, Uint8ConfigEntry, Name(LocalParams::RECONNECT_NAME),
    Description(LocalParams::RECONNECT_DESCR), Min(0), Max(1), Default(1),
    MapValues(LocalParams::RECONNECT_MAP));
/// Internal storage for the last working address. If the IP address field is
/// clear, there is no last known good address.
CDI_GROUP_ENTRY(last_address, TcpManualAddress<LocalParams>, Hidden(true));
CDI_GROUP_END();

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDTCPCONNCTION_HXX_

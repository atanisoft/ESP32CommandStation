/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file IfTcp.hxx
 *
 * OpenLCB interface implementation for native TCP connection.
 *
 * @author Balazs Racz
 * @date 16 Apr 2017
 */

#ifndef _OPENLCB_IFTCP_HXX_
#define _OPENLCB_IFTCP_HXX_

#include "openlcb/If.hxx"
#include "utils/Hub.hxx"

namespace openlcb
{

/// This is not used at the moment, as TCP packet routing is not supported yet.
struct TcpMessage
{
    /// Destination node, or {0,0} for broadcast. Helper function for routing.
    NodeHandle dst;
    /// Rendered string for the message payload. Includes the header (flags and
    /// length as well).
    string data;
};

class ClockBaseSequenceNumberGenerator;
class TcpSendFlow;
class TcpRecvFlow;

/// Network interface class for a character stream link that speaks the
/// (point-to-point) TcpTransfer protocol. This is the class that needs to be
/// given to the virtual node (Node*) on the constructor. The If class provides
/// the API and owns all of the implementation for sending and receiving
/// messages to/from this network link.
///
/// Today the IfTcp class supports a dumb hub on the device end. This allows
/// creating servers although without any advanced routing logic.
class IfTcp : public If
{
public:
    /// Creates a TCP interface.
    /// @param gateway_node_id will be stamped on outgoing messages as the
    /// gateway's node ID.
    /// @param device is a Hub to send the TCP packets to / receive input
    /// from. The interface will register itself into this hub. The executor of
    /// this device will be used for processing the packets in this interface.
    /// @param local_nodes_count is the maximum number of virtual nodes that
    /// this interface will support.
    IfTcp(NodeID gateway_node_id, HubFlow *device, int local_nodes_count);

    /// Destructor.
    ~IfTcp();

    /** Transfers ownership of a module to the interface. It will be brought
     * down in the destructor. The destruction order is guaranteed such that
     * all supporting structures are still available when the flow is destryed,
     * but incoming messages can not come in anymore.
     *
     * @todo(balazs.racz) revise whether this needs to be virtual. */
    void add_owned_flow(Executable *e) override;
    /** Transfers ownership of a module to the interface. It will be brought
     * down in the destructor. The destruction order is guaranteed such that
     * all supporting structures are still available when the flow is destryed,
     * but incoming messages can not come in anymore. */
    void add_owned_flow(Destructable *e)
    {
        ownedFlows_.emplace_back(e);
    }
    /// Finds a given pointer in the owned flows list, deletes it and removes
    /// it from the list.
    void delete_owned_flow(Destructable *d);
    /** Removes a local node from this interface. This function must be called
     * from the interface's executor.
     *
     * @param node is the node to delete. The node will not be freed, just
     * removed from the data structures.
     */
    void delete_local_node(Node *node) override;
    /** @return true if the two node handles match as far as we can tell
     * without doing any network traffic. */
    bool matching_node(NodeHandle expected, NodeHandle actual) override;

    /// Adds a network client connection to the device.
    /// @param fd is the socket going towards the client
    /// @param on_error will be invoked when a socket error is encountered.
    void add_network_fd(int fd, Notifiable *on_error = nullptr);

private:
    /// Where to send traffic to.
    HubFlow *device_;
    /// Various implementation control flows that this interface owns.
    std::vector<std::unique_ptr<Destructable>> ownedFlows_;
    /// Sequence number generator for outgoing TCP packets.
    ClockBaseSequenceNumberGenerator *seq_;
    /// Flow used for converting GenMessage into the binary
    /// representation. Owned by ownedFlows_.
    TcpSendFlow *sendFlow_;
    /// Flow for parsing incoming messages. Owned by ownedFlows_.
    TcpRecvFlow *recvFlow_;
};

} // namespace openlcb

#endif // _OPENLCB_IFTCP_HXX_

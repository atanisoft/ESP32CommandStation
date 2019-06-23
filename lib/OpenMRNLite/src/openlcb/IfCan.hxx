/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file IfCan.hxx
 *
 * Asynchronous NMRAnet interface implementation for CANbus.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _OPENLCB_IFCAN_HXX_
#define _OPENLCB_IFCAN_HXX_

#include <memory>

#include "executor/StateFlow.hxx"
#include "openlcb/If.hxx"
#include "openlcb/AliasCache.hxx"
#include "openlcb/Defs.hxx"
#include "utils/CanIf.hxx"

namespace openlcb
{

class IfCan;

/** Counts the number of alias conflicts that we see for aliases that we
 * already reserved. */
extern size_t g_alias_use_conflicts;

class AliasAllocator;
class IfCan;

/// Implementation of the OpenLCB interface abstraction for the CAN-bus
/// interface standard. This contains the parsers for CAN frames, dispatcher
/// for the different frame types, the alias mapping tables (both local and
/// remote nodes), and the flows responsible for sending outgoing
/// messages. Inherits handling of the MTI-indexed messages from the base class
/// @ref If.
class IfCan : public If, public CanIf
{
public:
    /**
     * Creates a CAN interface.
     *
     * @param executor will be used to process incoming (and outgoing) messages.
     *
     * @param device is a CanHub. The interface will add a member to this pipe
     * to handle incoming and outgoing traffic. The caller should add the
     * necessary hardware device, GridConnect bridge or mock interface to this
     * pipe (before this constructor or else outgoing packets might be lost).
     *
     * @param local_alias_cache_size tells the number of aliases to keep track
     * of for nocal virtual nodes and proxied nodes.
     *
     * @param remote_alias_cache_size tells the number of aliases to keep track
     * of for remote nodes on the bus.
     *
     * @param local_nodes_count is the maximum number of virtual nodes that
     * this interface will support. */
    IfCan(ExecutorBase *executor, CanHubFlow *device,
        int local_alias_cache_size, int remote_alias_cache_size,
        int local_nodes_count);

    ~IfCan();

    /** Adds support to this interface for addressed NMRAnet messages (both
     * sending and receiving). */
    void add_addressed_message_support();

    /// @returns the alias cache for local nodes (vnodes and proxies)
    AliasCache *local_aliases()
    {
        executor()->assert_current();
        return &localAliases_;
    }

    /// @returns the alias cache for remote nodes on this IF
    AliasCache *remote_aliases()
    {
        executor()->assert_current();
        return &remoteAliases_;
    }

    /// @returns the alias cache for remote nodes on this IF
    AliasAllocator *alias_allocator()
    {
        return aliasAllocator_.get();
    }

    /// Sets the alias allocator for this If. Takes ownership of pointer.
    void set_alias_allocator(AliasAllocator *a);

    void add_owned_flow(Executable *e) override;

    bool matching_node(NodeHandle expected, NodeHandle actual) override;

    void delete_local_node(Node *node) override;

    Node *lookup_local_node_handle(NodeHandle handle) override;

private:
    void canonicalize_handle(NodeHandle *h) override;

    friend class CanFrameWriteFlow; // accesses the device and the hubport.

    /** Aliases we know are owned by local (virtual or proxied) nodes.
     *
     *  This member must only be accessed from the If's executor.
     */
    AliasCache localAliases_;
    /** Aliases we know are owned by remote nodes on this If.
     *
     *  This member must only be accessed from the If's executor.
     */
    AliasCache remoteAliases_;

    /// Various implementation control flows that this interface owns.
    std::vector<std::unique_ptr<Executable>> ownedFlows_;

    /// Owns the alias allocator module.
    std::unique_ptr<AliasAllocator> aliasAllocator_;

    DISALLOW_COPY_AND_ASSIGN(IfCan);
};

/** Base class for incoming CAN frame handlers. */
class CanFrameStateFlow : public StateFlow<Buffer<CanMessageData>, QList<1>>
{
public:
    CanFrameStateFlow(IfCan *service)
        : StateFlow<Buffer<CanMessageData>, QList<1>>(service)
    {
    }

    IfCan *if_can()
    {
        return static_cast<IfCan *>(service());
    }
};

/// Request object for the NodeIdLookupFlow.
struct NodeCanonicalizeRequest
{
    /// Requests a NodeHandle to be canonicalized, i.e. look up node ID from
    /// alias.
    void reset(Node *node, NodeHandle handle)
    {
        srcNode = node;
        this->handle = handle;
        resultCode = 0;
    }

    /// Caller node (in order to talk to the bus)
    Node *srcNode;
    /// Destination node handle to canonicalize. At request time the alias
    /// should vbe filled in; at response time the id will be filled in as
    /// well.
    NodeHandle handle;

    /// Needed for receiveing replies by the customer.
    BarrierNotifiable done;
    /// Set to 0 on success, or an OpenLCB or OpenMRN error code in case of a
    /// failure.
    int resultCode;
};

/// Child flow to be used in parents that need translation from node alias to
/// node id.
class NodeIdLookupFlow
    : public StateFlow<Buffer<NodeCanonicalizeRequest>, QList<1>>
{
public:
    /// Constructor. @param iface is the CAN Interface that the source node is
    /// connecting to.
    NodeIdLookupFlow(IfCan *iface)
        : StateFlow<Buffer<NodeCanonicalizeRequest>, QList<1>>(iface)
    {
    }

    /// Starts processing an incoming request. @return next action.
    Action entry() override
    {
        if (input()->handle.id != 0)
        {
            return return_ok();
        }
        if (input()->handle.alias == 0)
        {
            // This is an empty handle, say that's OK.
            return return_ok();
        }
        NodeID id = iface()->remote_aliases()->lookup(input()->handle.alias);
        if (!id)
        {
            id = iface()->local_aliases()->lookup(input()->handle.alias);
        }
        if (id)
        {
            input()->handle.id = id;
            return return_ok();
        }
        return allocate_and_call(
            iface()->addressed_message_write_flow(), STATE(send_request));
    }

private:
    /// Send out a ping request to the destination alias. @return next action.
    Action send_request()
    {
        auto *b =
            get_allocation_result(iface()->addressed_message_write_flow());
        b->data()->reset(Defs::MTI_VERIFY_NODE_ID_ADDRESSED,
            input()->srcNode->node_id(), input()->handle, EMPTY_PAYLOAD);
        replyHandler_.set_alias_waiting(input()->handle.alias);
        iface()->addressed_message_write_flow()->send(b);

        return sleep_and_call(&timer_, MSEC_TO_NSEC(700), STATE(reply_timeout));
    }

    /// Called when a reply arrives or the timeout expires. @return next
    /// action.
    Action reply_timeout()
    {
        replyHandler_.set_alias_waiting(0);
        if (!timer_.is_triggered())
        {
            return return_with_error(Defs::ERROR_OPENLCB_TIMEOUT);
        }
        if (!input()->handle.id)
        {
            return return_with_error(Defs::ERROR_OPENMRN_NOT_FOUND);
        }
        return return_ok();
    }

    /// Class for listening to the ping response packets.
    class ReplyHandler : public MessageHandler
    {
    public:
        /// Constructor. @param parent is the lookup flow that owns this
        /// handler.
        ReplyHandler(NodeIdLookupFlow *parent)
            : parent_(parent)
        {
            parent_->iface()->dispatcher()->register_handler(
                this, Defs::MTI_VERIFIED_NODE_ID_NUMBER, Defs::MTI_EXACT);
        }

        /// Destructor. Unregisters the handler.
        ~ReplyHandler()
        {
            parent_->iface()->dispatcher()->unregister_handler(
                this, Defs::MTI_VERIFIED_NODE_ID_NUMBER, Defs::MTI_EXACT);
        }

        /// Handler callback for incoming messages.
        void send(Buffer<GenMessage> *message, unsigned priority) OVERRIDE
        {
            AutoReleaseBuffer<GenMessage> ab(message);
            if (aliasWaiting_ == 0)
                return;

            GenMessage *msg = message->data();
            if (msg->src.alias != aliasWaiting_)
                return;

            if (msg->src.id != 0)
            {
                parent_->input()->handle.id = msg->src.id;
            }
            else if (msg->payload.size() == 6)
            {
                parent_->input()->handle.id =
                    data_to_node_id(msg->payload.data());
            }
            else
            {
                // Node ID verified with no source data. Problem.
            }
            set_alias_waiting(0);
            parent_->timer_.trigger();
        }

        /// The parent flow calls this function when the handler needs to be
        /// activated.
        void set_alias_waiting(NodeAlias a)
        {
            aliasWaiting_ = a;
        }

    private:
        /// Which node alias we are listening for a reply from.
        NodeAlias aliasWaiting_{0};
        /// Flow owning *this.
        NodeIdLookupFlow *parent_;
    } replyHandler_{this};

    friend class ReplyHandler;

    /// Terminates the current subflow with no error.
    Action return_ok()
    {
        return return_with_error(0);
    }

    /// Terminates the current subflow with an error code.
    Action return_with_error(int error)
    {
        input()->resultCode = error;
        return_buffer();
        return exit();
    }

    /// @return the current input request.
    NodeCanonicalizeRequest *input()
    {
        return message()->data();
    }

    /// @return the stored interface
    IfCan *iface()
    {
        return static_cast<IfCan *>(service());
    }

    /// Helper object for calling subflows.
    BarrierNotifiable bn_;
    /// Helper object for timed wait.
    StateFlowTimer timer_{this};
};

} // namespace openlcb

#endif // _OPENLCB_IFCAN_HXX_

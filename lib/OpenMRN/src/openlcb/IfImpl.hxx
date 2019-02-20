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
 * \file IfImpl.hxx
 *
 * Implementation details for the asynchronous NMRAnet interfaces. This file
 * should only be needed in hardware interface implementations.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#ifndef _OPENLCB_IFIMPL_HXX_
#define _OPENLCB_IFIMPL_HXX_

#include "openlcb/If.hxx"

namespace openlcb
{

/** Implementation of the hardware-independent parts of the write flows. */
class WriteFlowBase : public StateFlow<Buffer<GenMessage>, QList<4>>
{
public:
    WriteFlowBase(If *async_if)
        : StateFlow<Buffer<GenMessage>, QList<4>>(async_if)
    {
    }

protected:
    /** This function will be called (on the main executor) to initiate sending
     * this message to the hardware. The flow will then execute the returned
     * action.
     *
     * NOTE: it is possible that this functon will never be called for a given
     * flow. */
    virtual Action send_to_hardware() = 0;

    /** This state is called when an addressed message's destination is a node
     * that is local to this interface. The dstNode will already be filled. The
     * implementation should perform the send and then transition to the
     * send_finished state. */
    virtual Action send_to_local_node();

    /** Virtual method called after the send is completed, i.e., all the frames
     * are generated and sent to the hardware. Various flows might need to take
     * additional steps afterwards. */
    virtual Action send_finished()
    {
        return release_and_exit();
    }

    /// @returns the interface that this flow is assigned to.
    If *async_if()
    {
        return static_cast<If *>(service());
    }

    /** Implementations shall call this function when they are done with
     * sending the packet.
     */
    // void cleanup();

    /// Returns the NMRAnet message we are trying to send.
    GenMessage *nmsg()
    {
        return message()->data();
    }

protected:
    /*    /// Entry point for external callers.
        virtual void WriteAddressedMessage(Defs::MTI mti, NodeID src, NodeHandle
       dst,
                                           Buffer* data, Notifiable* done)
        {
            HASSERT(IsNotStarted());
            Restart(done);
            mti_ = mti;
            src_ = src;
            dst_ = dst;
            dstNode_ = nullptr;
            data_ = data;
            StartFlowAt(STATE(maybe_send_to_local_node));
        }

        /// Entry point for external callers.
        virtual void WriteGlobalMessage(Defs::MTI mti, NodeID src, Buffer* data,
                                        Notifiable* done)
        {
            HASSERT(IsNotStarted());
            Restart(done);
            mti_ = mti;
            src_ = src;
            dst_.id = 0;
            dst_.alias = 0;
            dstNode_ = nullptr;
            data_ = data;
            StartFlowAt(STATE(send_to_local_nodes));
        }
    */

    /** Addressed write flows should call this state BEFORE sending to the
     * hardware. They may get back control at the send_to_hardware state if
     * needed.
     * NOTE: datagram write flow cannot use this because it won't get back. */
    Action addressed_entry();
    /** Global write flows should return to this state AFTER sending the
     * message to the hardware. They should ensure the message is still
     * intact. They will not get back control. */
    Action global_entry();
};

/** This handler handles VerifyNodeId messages (both addressed and global) on
 * the interface level. Each interface implementation will want to create one
 * of these. */
class VerifyNodeIdHandler : public IncomingMessageStateFlow
{
public:
    VerifyNodeIdHandler(If *service) : IncomingMessageStateFlow(service)
    {
        iface()->dispatcher()->register_handler(
            this, Defs::MTI_VERIFY_NODE_ID_GLOBAL &
                      Defs::MTI_VERIFY_NODE_ID_ADDRESSED,
            0xffff & ~(Defs::MTI_VERIFY_NODE_ID_GLOBAL ^
                       Defs::MTI_VERIFY_NODE_ID_ADDRESSED));
    }

    ~VerifyNodeIdHandler()
    {
        iface()->dispatcher()->unregister_handler(
            this, Defs::MTI_VERIFY_NODE_ID_GLOBAL &
                      Defs::MTI_VERIFY_NODE_ID_ADDRESSED,
            0xffff & ~(Defs::MTI_VERIFY_NODE_ID_GLOBAL ^
                       Defs::MTI_VERIFY_NODE_ID_ADDRESSED));
    }

    /// Handler callback for incoming messages.
    Action entry() override
    {
        GenMessage *m = message()->data();
        if (m->dst.id)
        {
            // Addressed message.
            srcNode_ = m->dstNode;
            it_ = iface()->localNodes_.end();
        }
        else if (!m->payload.empty() && m->payload.size() == 6)
        {
            // Global message with a node id included
            NodeID id = buffer_to_node_id(m->payload);
            srcNode_ = iface()->lookup_local_node(id);
            if (!srcNode_)
            {
                // Someone looking for a node that's not on this interface.
                return release_and_exit();
            }
#ifndef SIMPLE_NODE_ONLY
            it_ = iface()->localNodes_.end();
#endif
        }
        else
        {
// Global message. Everyone should respond.
#ifdef SIMPLE_NODE_ONLY
            // We assume there can be only one local node.
            If::VNodeMap::Iterator it = iface()->localNodes_.begin();
            if (it == iface()->localNodes_.end())
            {
                // No local nodes.
                return release_and_exit();
            }
            srcNode_ = it->second;
            ++it;
            HASSERT(it == iface()->localNodes_.end());
#else
            // We need to do an iteration over all local nodes.
            it_ = iface()->localNodes_.begin();
            if (it_ == iface()->localNodes_.end())
            {
                // No local nodes.
                return release_and_exit();
            }
            srcNode_ = it_->second;
            ++it_;
#endif // not simple node.
        }
        if (srcNode_)
        {
            release();
            return allocate_and_call(iface()->global_message_write_flow(),
                                     STATE(send_response));
        }
        LOG(WARNING, "node pointer not found.");
        return release_and_exit();
    }

#ifdef SIMPLE_NODE_ONLY
    Action send_response()
    {
        auto *b =
            get_allocation_result(iface()->global_message_write_flow());
        GenMessage *m = b->data();
        NodeID id = srcNode_->node_id();
        m->reset(Defs::MTI_VERIFIED_NODE_ID_NUMBER, id, node_id_to_buffer(id));
        iface()->global_message_write_flow()->send(b);
        return exit();
    }
#else
    Action send_response()
    {
        auto *b =
            get_allocation_result(iface()->global_message_write_flow());
        GenMessage *m = b->data();
        // This is called on the main executor of the interface, this we are
        // allowed to access the local nodes cache.
        NodeID id = srcNode_->node_id();
        LOG(VERBOSE, "Sending verified reply from node %012" PRIx64, id);
        m->reset(Defs::MTI_VERIFIED_NODE_ID_NUMBER, id, node_id_to_buffer(id));
        iface()->global_message_write_flow()->send(b);

        /** Continues the iteration over the nodes.
         *
         * @TODO(balazs.racz): we should probably wait for the outgoing message
         * to be sent. */
        if (it_ != iface()->localNodes_.end())
        {
            srcNode_ = it_->second;
            ++it_;
            return allocate_and_call(iface()->global_message_write_flow(),
                                     STATE(send_response));
        }
        else
        {
            return exit();
        }
    }
#endif // not simple node

private:
    Node *srcNode_;

#ifndef SIMPLE_NODE_ONLY
    If::VNodeMap::Iterator it_;
#endif
};
} // namespace openlcb

#endif // _OPENLCB_IFIMPL_HXX_

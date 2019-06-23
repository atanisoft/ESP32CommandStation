/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file NodeInitializeFlow.cxx
 *
 * Default AsyncNode implementation for a fat virtual node.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#ifndef _OPENLCB_NODEINITIALIZEFLOW_HXX_
#define _OPENLCB_NODEINITIALIZEFLOW_HXX_

#include "openlcb/DefaultNode.hxx"
#include "openlcb/If.hxx"
#include "nmranet_config.h"
#include "utils/Singleton.hxx"

namespace openlcb
{

/// Request to send to instruct @ref InitializeFlow to perform the
/// initialization function of a single virtual node.
struct InitializeRequest
{
    InitializeRequest()
        : node(nullptr)
    {
    }
    Node *node;
};

typedef StateFlow<Buffer<InitializeRequest>, QList<1>> InitializeFlowBase;

/// Performs upon-startup initialization of virtual nodes.
///
/// Usage: Create a global static instance of InitializeFlow. Allocate a
/// Buffer<INitializerequest> and fill in the node pointer. Send the buffer via
/// Singleton<InitializeFlow>::instance()->send(buffer)
class InitializeFlow : public InitializeFlowBase,
                       public Singleton<InitializeFlow>
{
public:
    InitializeFlow(Service *service)
        : InitializeFlowBase(service)
    {
    }

    ~InitializeFlow();

private:
    Node *node()
    {
        return message()->data()->node;
    }

    Action entry() OVERRIDE
    {
        HASSERT(message()->data()->node);
        return allocate_and_call(
            node()->iface()->global_message_write_flow(),
            STATE(send_initialized));
    }

    Action send_initialized()
    {
        auto *b = get_allocation_result(
            node()->iface()->global_message_write_flow());
        done_.reset(this);
        NodeID id = node()->node_id();
        b->data()->reset(
            Defs::MTI_INITIALIZATION_COMPLETE, id, node_id_to_buffer(id));
        b->data()->set_flag_dst(GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
        b->set_done(&done_);
        node()->iface()->global_message_write_flow()->send(
            b, b->data()->priority());
        return wait_and_call(STATE(initialization_complete));
    }

    Action initialization_complete()
    {
        node()->set_initialized();
        return call_immediately(STATE(identify_events));
    }

    Action identify_events()
    {
        if (config_node_init_identify() != CONSTANT_TRUE)
        {
            return release_and_exit();
        }
        // Get the dispatch flow.
        return allocate_and_call(
            node()->iface()->dispatcher(), STATE(initiate_local_identify));
    }

    Action initiate_local_identify()
    {
        auto *b = get_allocation_result(node()->iface()->dispatcher());
        b->set_done(done_.reset(this));
        GenMessage *m = b->data();
        m->mti = Defs::MTI_EVENTS_IDENTIFY_ADDRESSED;
        m->payload.clear();
        m->dst.id = node()->node_id();
        m->dstNode = node();
        m->src.alias = 0;
        m->src.id = node()->node_id();
        node()->iface()->dispatcher()->send(b, b->data()->priority());
        return wait_and_call(STATE(wait_for_local_identify));
    }

    Action wait_for_local_identify()
    {
        return release_and_exit();
    }

    BarrierNotifiable done_;
};

/// Helper function that sends a local virtual node to the static
/// InitializeFlow.
void StartInitializationFlow(Node *node);

/// StateFlow that iterates through all local nodes and sends out node
/// initialization complete for each of them. Used when a TCP disconnect event
/// causes us to lose network connectivity and later the connection gets
/// reestablished.
class ReinitAllNodes : public StateFlowBase {
public:
    ReinitAllNodes(If* iface) : StateFlowBase(iface) {
        nextNode_ = iface->first_local_node();
        start_flow(STATE(allocate_entry));
    }

private:
    Action allocate_entry() {
        if (!nextNode_) {
            return delete_this();
        }
        return allocate_and_call(tgt(), STATE(send_init_request));
    }

    Action send_init_request() {
        auto* b = get_allocation_result(tgt());
        b->data()->node = nextNode_;
        b->set_done(bn_.reset(this));
        tgt()->send(b);
        return wait_and_call(STATE(init_done));
    }

    Action init_done() {
        nextNode_ = iface()->next_local_node(nextNode_->node_id());
        return call_immediately(STATE(allocate_entry));
    }

    InitializeFlow* tgt() {
        return Singleton<InitializeFlow>::instance();
    }

    If* iface() {
        return static_cast<If*>(service());
    }

    /// Which node to send identify next. If nullptr, we're done.
    Node* nextNode_;
    BarrierNotifiable bn_;
};

} // namespace openlcb

#endif // _OPENLCB_NODEINITIALIZEFLOW_HXX_

/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file NodeBrowser.cxx
 * Module that allows monitoring the list of nodes on the network.
 *
 * @author Balazs Racz
 * @date 8 March 2019
 */

#include "openlcb/NodeBrowser.hxx"

namespace openlcb
{

NodeBrowser::NodeBrowser(Node *node, CallbackFunction cb)
    : node_(node)
    , callback_(std::move(cb))
{
    register_callbacks();
}

NodeBrowser::~NodeBrowser()
{
    unregister_callbacks();
}

void NodeBrowser::refresh()
{
    auto b = node_->iface()->global_message_write_flow()->alloc();
    b->data()->reset(
        Defs::MTI_VERIFY_NODE_ID_GLOBAL, node_->node_id(), EMPTY_PAYLOAD);
    node_->iface()->global_message_write_flow()->send(b);
}

void NodeBrowser::register_callbacks()
{
    node_->iface()->dispatcher()->register_handler(
        &handler_, Defs::MTI_VERIFIED_NODE_ID_NUMBER, Defs::MTI_EXACT);
    node_->iface()->dispatcher()->register_handler(
        &handler_, Defs::MTI_INITIALIZATION_COMPLETE, Defs::MTI_EXACT);
}

void NodeBrowser::unregister_callbacks()
{
    node_->iface()->dispatcher()->unregister_handler_all(&handler_);
}

NodeBrowser::VerifiedHandler::VerifiedHandler(NodeBrowser *parent)
    : parent_(parent)
{
}

void NodeBrowser::VerifiedHandler::send(Buffer<GenMessage> *b, unsigned)
{
    auto d = get_buffer_deleter(b);
    if (b->data()->payload.size() != 6)
    {
        return;
    }
    NodeID tgt = buffer_to_node_id(b->data()->payload);
    parent_->callback_(tgt);
}

} // namespace openlcb

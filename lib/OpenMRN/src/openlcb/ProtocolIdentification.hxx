/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file ProtocolIdentification.hxx
 * This file provides an implementation of the OpenLCB Protocol Identification
 * Protocol.
 *
 * @author Stuart W. Baker
 * @date 2 February 2015
 */

#ifndef _OPENLCB_PROTOCOLIDENTIFICATION_HXX_
#define _OPENLCB_PROTOCOLIDENTIFICATION_HXX_

#include "openlcb/Defs.hxx"
//#include "openlcb/

namespace openlcb
{

/** An instance of this class will add Protocol Identification Protocol to
 * an NMRAnet Node.
 */
class ProtocolIdentificationHandler : public IncomingMessageStateFlow
{
public:
    /** Constructor.
     * @param node node for which to add protocol to.
     * @param supported bit mask of supported @ref Defs::Protocols for the node
     */
    ProtocolIdentificationHandler(Node* node, uint64_t supported)
        : IncomingMessageStateFlow(node->iface())
        , node_(node)
        , payload_()
    {
        /* store the supported protocol as a payload */
        payload_ = node_id_to_buffer(supported);

        /* register our interest in the Protocol Identification Protocol */
        node_->iface()->dispatcher()->register_handler(
            this, Defs::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI_EXACT);
    }

    /** Destructor.
     */
    ~ProtocolIdentificationHandler()
    {
        /* register our interest in the Protocol Identification Protocol */
        node_->iface()->dispatcher()->unregister_handler(
            this, Defs::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI_EXACT);
    }

private:
    /** Entry point to state flow
     * @return release_and_exit() if not for this node
     * @return allocate_and_call() for @ref fill_response_buffer() to responde
     *         with an outgoing message
     */
    Action entry() OVERRIDE
    {
        if (nmsg()->dstNode != node_)
        {
            /* not for me */
            return release_and_exit();
        }
        /* hanlde messager */
        //LOG(INFO, "PIP Handle");
        return allocate_and_call(
            node_->iface()->addressed_message_write_flow(),
            STATE(fill_response_buffer));
    }

    /** Respond to incoming request with an outgoing message
     * @return
     */
    Action fill_response_buffer()
    {
        /* grab our allocated buffer */
        auto *b = get_allocation_result(
            node_->iface()->addressed_message_write_flow());
        /* fill in response. */
        b->data()->reset(Defs::MTI_PROTOCOL_SUPPORT_REPLY, node_->node_id(), nmsg()->src, payload_);

        /* pass the response to the addressed message write flow */
        node_->iface()->addressed_message_write_flow()->send(b);

        return release_and_exit();
    }

    /** local copy of the node we are binding to */
    Node *node_;

    /** response payload */
    Payload payload_;

    DISALLOW_COPY_AND_ASSIGN(ProtocolIdentificationHandler);
};

} /* namespace openlcb */

#endif // _OPENLCB_PROTOCOLIDENTIFICATION_HXX_

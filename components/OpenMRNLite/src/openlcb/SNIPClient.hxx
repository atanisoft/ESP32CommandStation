/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file SNIPClient.hxx
 *
 * A client library for talking to an arbitrary openlcb Node and ask it for the
 * Simple Node Ident Info data.
 *
 * @author Balazs Racz
 * @date 18 Oct 2020
 */

#ifndef _OPENLCB_SNIPCLIENT_HXX_
#define _OPENLCB_SNIPCLIENT_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/If.hxx"

namespace openlcb
{

/// Buffer contents for invoking the SNIP client.
struct SNIPClientRequest : public CallableFlowRequestBase
{
    /// Helper function for invoke_subflow.
    /// @param src the openlcb node to call from
    /// @param dst the openlcb node to target
    void reset(Node *src, NodeHandle dst)
    {
        reset_base();
        resultCode = OPERATION_PENDING;
        src_ = src;
        dst_ = dst;
    }

    enum
    {
        OPERATION_PENDING = 0x20000, //< cleared when done is called.
        ERROR_REJECTED = 0x200000,   //< Target node has rejected the request.
        OPENMRN_TIMEOUT = 0x80000,   //< Timeout waiting for ack/nack.
    };

    /// Source node where to send the request from.
    Node *src_;
    /// Destination node to query.
    NodeHandle dst_;
    /// Response payload if successful.
    Payload response;
};

#if !defined(GTEST) || !defined(SNIP_CLIENT_TIMEOUT_NSEC)
/// Specifies how long to wait for a SNIP request to get a response. Writable
/// for unittesting purposes.
static constexpr long long SNIP_CLIENT_TIMEOUT_NSEC = MSEC_TO_NSEC(1500);
#endif

class SNIPClient : public CallableFlow<SNIPClientRequest>
{
public:
    /// Constructor.
    /// @param s service of the openlcb executor.
    SNIPClient(Service *s)
        : CallableFlow<SNIPClientRequest>(s)
    {
    }

    Action entry() override
    {
        request()->resultCode = SNIPClientRequest::OPERATION_PENDING;
        return allocate_and_call(
            iface()->addressed_message_write_flow(), STATE(write_request));
    }

private:
    enum
    {
        MTI_1a = Defs::MTI_TERMINATE_DUE_TO_ERROR,
        MTI_1b = Defs::MTI_OPTIONAL_INTERACTION_REJECTED,
        MASK_1 = ~(MTI_1a ^ MTI_1b),
        MTI_1 = MTI_1a,

        MTI_2 = Defs::MTI_IDENT_INFO_REPLY,
        MASK_2 = Defs::MTI_EXACT,
    };

    /// Called once the allocation is complete. Sends out the SNIP request to
    /// the bus.
    Action write_request()
    {
        auto *b =
            get_allocation_result(iface()->addressed_message_write_flow());
        b->data()->reset(Defs::MTI_IDENT_INFO_REQUEST,
            request()->src_->node_id(), request()->dst_, EMPTY_PAYLOAD);

        iface()->dispatcher()->register_handler(
            &responseHandler_, MTI_1, MASK_1);
        iface()->dispatcher()->register_handler(
            &responseHandler_, MTI_2, MASK_2);

        iface()->addressed_message_write_flow()->send(b);

        return sleep_and_call(
            &timer_, SNIP_CLIENT_TIMEOUT_NSEC, STATE(response_came));
    }

    /// Callback from the response handler.
    /// @param message the incoming response message from the bus
    void handle_response(Buffer<GenMessage> *message)
    {
        auto rb = get_buffer_deleter(message);
        if (request()->src_ != message->data()->dstNode ||
            !iface()->matching_node(request()->dst_, message->data()->src))
        {
            // Not from the right place.
            return;
        }
        if (message->data()->mti == Defs::MTI_OPTIONAL_INTERACTION_REJECTED ||
            message->data()->mti == Defs::MTI_TERMINATE_DUE_TO_ERROR)
        {
            uint16_t mti, error_code;
            buffer_to_error(
                message->data()->payload, &error_code, &mti, nullptr);
            LOG(INFO, "rejection err %04x mti %04x", error_code, mti);
            if (mti && mti != Defs::MTI_IDENT_INFO_REQUEST)
            {
                // Got error response for a different interaction. Ignore.
                return;
            }
            request()->resultCode =
                error_code | SNIPClientRequest::ERROR_REJECTED;
        }
        else if (message->data()->mti == Defs::MTI_IDENT_INFO_REPLY)
        {
            request()->response = std::move(message->data()->payload);
            request()->resultCode = 0;
        }
        else
        {
            // Dunno what this MTI is. Ignore.
            LOG(INFO, "Unexpected MTI for SNIP response handler: %04x",
                message->data()->mti);
            return;
        }
        // Wakes up parent flow.
        request()->resultCode &= ~SNIPClientRequest::OPERATION_PENDING;
        timer_.trigger();
    }

    Action response_came()
    {
        iface()->dispatcher()->unregister_handler_all(&responseHandler_);
        if (request()->resultCode & SNIPClientRequest::OPERATION_PENDING)
        {
            return return_with_error(SNIPClientRequest::OPENMRN_TIMEOUT);
        }
        return return_with_error(request()->resultCode);
    }

    /// @return openlcb source interface.
    If *iface()
    {
        return request()->src_->iface();
    }

    /// Handles the timeout feature.
    StateFlowTimer timer_ {this};
    /// Registered handler for response messages.
    IncomingMessageStateFlow::GenericHandler responseHandler_ {
        this, &SNIPClient::handle_response};
};

} // namespace openlcb

#endif // _OPENLCB_SNIPCLIENT_HXX_

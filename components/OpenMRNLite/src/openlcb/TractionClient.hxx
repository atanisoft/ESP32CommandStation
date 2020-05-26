/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TractionClient.hxx
 *
 * Client API for the Traction protocol.
 *
 * @author Balazs Racz
 * @date 20 May 2014
 */

#ifndef _OPENLCB_TRACTIONCLIENT_HXX_
#define _OPENLCB_TRACTIONCLIENT_HXX_

#include "openlcb/If.hxx"
#include "openlcb/Defs.hxx"

namespace openlcb
{

/** This class helps waiting for traction responses.
 *
 * It listens for traction control response messages, matches them to an expected destination/responsetype pair, and triggers a timeout when the matching response arrives.
 *
 * Synchronous usage:
 *   auto* b = ... // allocate and fill reuqest buffer.
 *   SyncTimeout t(g_executor.active_timers());
 *   t.start(MSEC_TO_NSEC(300));
 *   response_handler.wait_for_response(b->data()->dst, b->data()->payload[0], &t);
 *   iface->addressed_message_write_flow()->send(b);
 *   t.wait_for_notification();
 *   response_handler.wait_timeout();  // Stops listening.
 *   if (response_handler.response())
 *   {
 *      // we have a response -- use it from response_handler->response()
 *      response_handler.response()->unref();
 *   } else {
 *      // timeout
 *   }
 *
 * Asynchronous usage:
 *
 * FlowClass : public StateFlow... { 
 *
 *   Action send_request()
 *   {
 *      auto* b = get_allocation_result(iface->addressed_message_write_flow());
 *      // fill b
 *      responseHandler_.wait_for_response(b->data()->dst, b->data()->payload[0], &t_);
 *      iface->addressed_message_write_flow()->send(b);
 *      return sleep_and_call(&t_, MSEC_TO_NSEC(), STATE(parse_response));
 *   }
 *
 *   Action parse_response()
 *   {
 *     responseHandler_.wait_timeout();
 *     if (response_handler.response())
 *     {
 *       // parse response  ... do useful stuff.
 *       responseHandler_.response()->unref();
 *     }
 *     else
 *     {
 *       return gone_to_error();
 *     }
 *   }
 *
 *   TractionResponseHandler responseHandler_;
 *   StateFlowTimer t_;
 * };
 */
class TractionResponseHandler : public IncomingMessageStateFlow
{
public:
    TractionResponseHandler(openlcb::If *iface,
                            openlcb::Node *local_node)
        : IncomingMessageStateFlow(iface)
        , expectedDst_(local_node)
        , trigger_(nullptr)
        , response_(nullptr)
    {
    }

    /** Starts waiting for a traction control reply from a given node with the
     * first byte 'expected_type'. Will wake up the given timer if a reply
     * arrives. */
    void wait_for_response(NodeHandle target_node, uint8_t expected_type,
                           ::Timer *trigger)
    {
        response_ = nullptr;
        expectedSrc_ = target_node;
        expectedType_ = expected_type;
        trigger_ = trigger;
        start_listening();
    }

    /** Caller must unref this buffer when done with it. */
    Buffer<GenMessage> *response()
    {
        return response_;
    }

    /** Call this if the timeout has expired. Call only on the service
     * executor. If the caller got notified due to a response, it is still okay
     * to call this. @TODO(balazs.racz) rename to something like
     * wait_done_or_timeout() */
    void wait_timeout()
    {
        if (trigger_)
        {
            trigger_ = nullptr;
            stop_listening();
        }
    }

private:
    void start_listening()
    {
        iface()->dispatcher()->register_handler(
            this, openlcb::Defs::MTI_TRACTION_CONTROL_REPLY,
            openlcb::Defs::MTI_EXACT);
    }

    void stop_listening()
    {
        iface()->dispatcher()->unregister_handler(
            this, openlcb::Defs::MTI_TRACTION_CONTROL_REPLY,
            openlcb::Defs::MTI_EXACT);
    }

    Action entry() OVERRIDE
    {
        LOG_ERROR("response came");

        if (!trigger_)
        {
            // We already matched -- drop all packets to the floor.
            LOG_ERROR("no trigger");
            return release_and_exit();
        }
        if (nmsg()->dstNode != expectedDst_) {
            LOG_ERROR("dst not match");
            return release_and_exit();
        }
        /// @TODO(balazs.racz) factor out this code into a helper function that
        /// allows matching an incoming response frame to an expected
        /// NodeHandle.
        if (expectedSrc_.id && nmsg()->src.id)
        {
            if (expectedSrc_.id != nmsg()->src.id)
            {
                LOG_ERROR("src.id not match");
                return release_and_exit();
            }
        }
        else if (expectedSrc_.alias && nmsg()->src.alias)
        {
            if (expectedSrc_.alias != nmsg()->src.alias)
            {
                LOG_ERROR("src.alias not match");
                return release_and_exit();
            }
        }
        else
        {
            /// @TODO(balazs.racz) we should start an alias resolution process
            /// here.
            DIE("Unable to decide whether the incoming response is coming from "
                "the right place.");
        }
        // Now: message is from the right source node, to the right destination
        // node.
        if (nmsg()->payload.size() < 1) {
            LOG_ERROR("no payload");
            return release_and_exit();
        }
        if (nmsg()->payload[0] != expectedType_) {
            LOG_ERROR("payload type no match");
            return release_and_exit();
        }
        // Now: we matched!
        stop_listening();
        response_ = transfer_message();
        trigger_->trigger();
        trigger_ = nullptr;
        return exit();
    }

    openlcb::NodeHandle expectedSrc_;
    openlcb::Node *expectedDst_;
    // First byte of the response message.
    uint8_t expectedType_;
    ::Timer *trigger_;
    Buffer<GenMessage> *response_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONCLIENT_HXX_

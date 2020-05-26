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
 * \file WriteHelper.hxx
 *
 * Class that allows enqueing an outgoing message.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#ifndef _OPENLCB_WRITEHELPER_HXX_
#define _OPENLCB_WRITEHELPER_HXX_

#include <string>

#include "nmranet_config.h"

#include "openlcb/If.hxx"
#include "openlcb/Node.hxx"

namespace openlcb
{

/// A statically allocated buffer for sending one message to the OpenLCB
/// bus. This buffer is reusable, as soon as the done notifiable is called, the
/// buffer is free for sending the next packet.
class WriteHelper : public Executable
{
public:
    typedef Node *node_type;
    typedef string payload_type;

    static NodeHandle global()
    {
        return {0, 0};
    }

    WriteHelper()
        : waitForLocalLoopback_(0)
    {
    }

    const payload_type &last_payload()
    {
        return buffer_;
    }

    void clear_last_payload()
    {
        buffer_.clear();
    }

    void set_wait_for_local_loopback(bool wait = true)
    {
        waitForLocalLoopback_ = (wait ? 1 : 0);
    }

    /** Originates an NMRAnet message from a particular node.
     *
     * @param node is the originating node.
     * @param mti is the message to send
     * @param dst is the destination node to send to (may be global())
     * @param buffer is the message payload.
     * @param done will be notified when the packet has been enqueued to the
     * physical layer. If done == nullptr, the sending is invoked synchronously.
     */
    void WriteAsync(Node *node, Defs::MTI mti, NodeHandle dst,
                    const payload_type &buffer, Notifiable *done)
    {
        if (done)
        {
            done_.reset(done);
        }
        else
        {
            // We don't support synchronous sending anymore.
            HASSERT(0);
        }
        if (!node ||
            (!node->is_initialized() && mti != Defs::MTI_INITIALIZATION_COMPLETE))
        {
            done_.notify();
            return;
        }
        node_ = node;
        mti_ = mti;
        dst_ = dst;
        buffer_ = buffer;
        if (dst == global())
        {
            node->iface()->global_message_write_flow()->alloc_async(this);
        }
        else
        {
            node->iface()->addressed_message_write_flow()->alloc_async(
                this);
        }
    }

private:
    // Callback from the allocator.
    void alloc_result(QMember *entry) override
    {
        /* NOTE(balazs.racz): We could choose not to pass on the done_
         * callback. That will allow the current write flow to be released
         * earlier for reuse, but breaks the assumption that done means that
         * the current packet is enqueued on the physical layer. */
        if (dst_ == global())
        {
            auto *f = node_->iface()->global_message_write_flow();
            Buffer<GenMessage> *b = f->cast_alloc(entry);
            b->data()->reset(mti_, node_->node_id(), buffer_);
            if (waitForLocalLoopback_)
            {
                b->data()->set_flag_dst(
                    GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
            }
            b->set_done(&done_);
            f->send(b, b->data()->priority());
        }
        else
        {
            auto *f = node_->iface()->addressed_message_write_flow();
            auto *b = f->cast_alloc(entry);
            b->data()->reset(mti_, node_->node_id(), dst_, buffer_);
            if (waitForLocalLoopback_)
            {
                b->data()->set_flag_dst(
                    GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
            }
            b->set_done(&done_);
            f->send(b, b->data()->priority());
        }
    }

    void run() override
    {
        HASSERT(0);
    }

    unsigned waitForLocalLoopback_ : 1;
    NodeHandle dst_;
    Defs::MTI mti_;
    Node *node_;
    payload_type buffer_;
    BarrierNotifiable done_;
};

}; /* namespace openlcb */

#endif // _OPENLCB_WRITEHELPER_HXX_

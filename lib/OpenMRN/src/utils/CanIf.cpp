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
 * \file CanIf.cxx
 *
 * Defines a service for interfacing with a CANbus. The interface receives
 * incoming frames from the bus device and proxies it to various handlers that
 * the user may register for specific ranges of CAN identifiers.
 *
 * @author Balazs Racz
 * @date 11 May 2014
 */

#include "utils/CanIf.hxx"

Pool *CanFrameWriteFlow::pool()
{
    return ifCan_->device()->pool();
}

void CanFrameWriteFlow::send(Buffer<CanHubData> *message, unsigned priority)
{
    LOG(VERBOSE, "outgoing message %" PRIx32 ".",
        GET_CAN_FRAME_ID_EFF(message->data()->frame()));
    message->data()->skipMember_ = ifCan_->hub_port();
    ifCan_->device()->send(message, priority);
}

Pool *CanFrameReadFlow::pool()
{
    /* NOTE(balazs.racz) This pool should rather be the application-level
     * buffer pool (for example openlcb::AsyncIf::dispatcher()::pool(), but
     * that pointer is not available here. At the moment applications use the
     * mainBufferPool. */
    return ifCan_->frame_dispatcher()->pool();
}

void CanFrameReadFlow::send(Buffer<CanHubData> *message, unsigned priority)
{
    const struct can_frame &frame = message->data()->frame();
    if (IS_CAN_FRAME_ERR(frame) || IS_CAN_FRAME_RTR(frame))
    {
        // Ignores these frames.
        message->unref();
        return;
    }

    // We typecast the incoming buffer to a different buffer type that should be
    // the subset of the data.
    Buffer<CanMessageData> *incoming_buffer;

    // Checks that it fits.
    HASSERT(sizeof(*incoming_buffer) <= sizeof(*message));
    // Does the cast.
    incoming_buffer = static_cast<Buffer<CanMessageData> *>(
        static_cast<BufferBase *>(message));
    // Checks that the frame is still in the same place (by pointer).
    HASSERT(incoming_buffer->data()->mutable_frame() ==
            message->data()->mutable_frame());

    /// @todo(balazs.racz): Figure out what priority the new message should be
    /// at.
    ifCan_->frame_dispatcher()->send(incoming_buffer, priority);
}

CanIf::CanIf(Service* service, CanHubFlow* device)
    : device_(device)
    , frameWriteFlow_(this)
    , frameReadFlow_(this)
    , frameDispatcher_(service) {
    this->device()->register_port(hub_port());
}

CanIf::~CanIf() {
    this->device()->unregister_port(hub_port());
}

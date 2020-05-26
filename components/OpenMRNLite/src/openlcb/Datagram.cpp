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
 * \file Datagram.cxx
 *
 * Implementation for the node-agnostic datagram functionality.
 *
 * @author Balazs Racz
 * @date 25 Jan 2013
 */

#include "openlcb/Datagram.hxx"

namespace openlcb
{

/// Defines how long the datagram client flow should wait for the datagram
/// ack/nack response message.
long long DATAGRAM_RESPONSE_TIMEOUT_NSEC = SEC_TO_NSEC(3);

DatagramService::DatagramService(If* iface,
                                 size_t num_registry_entries)
    : Service(iface->executor()), iface_(iface), dispatcher_(iface_, num_registry_entries)
{
    iface_->dispatcher()->register_handler(&dispatcher_, Defs::MTI_DATAGRAM, 0xffff
                                              );
}

DatagramService::~DatagramService()
{
    iface_->dispatcher()->unregister_handler(&dispatcher_, Defs::MTI_DATAGRAM, 0xffff
                                                );
}

StateFlowBase::Action DatagramService::DatagramDispatcher::entry()
{
    if (!nmsg()->dstNode)
    {
        return release_and_exit();
    }
    return allocate_and_call<IncomingDatagram>(nullptr, STATE(incoming_datagram_allocated), g_incoming_datagram_allocator);
}

StateFlowBase::Action
DatagramService::DatagramDispatcher::incoming_datagram_allocated()
{
    Buffer<IncomingDatagram>* b =
        get_allocation_result<IncomingDatagram>(nullptr);
    IncomingDatagram* d = b->data();
    d->src = nmsg()->src;
    d->dst = nmsg()->dstNode;

    // Takes over ownership of payload.
    /// @TODO(balazs.racz) Implement buffer refcounting.
    d->payload.swap(nmsg()->payload);

    release();

    // Saves the datagram buffer pointer.
    d_ = b;

    unsigned datagram_id = -1;
    if (d->payload.empty())
    {
        LOG(WARNING,
            "Invalid arguments: incoming datagram from node %012" PRIx64
            " alias %x has no payload.",
            d->src.id, d->src.alias);
        resultCode_ = DatagramClient::PERMANENT_ERROR;
        return allocate_and_call(iface()->addressed_message_write_flow(),
                                 STATE(respond_rejection));
    }
    datagram_id = *reinterpret_cast<const uint8_t*>(d->payload.data());

    // Looks up the datagram handler.
    DatagramHandler* h = registry_.lookup(d->dst, datagram_id);

    if (!h)
    {
        LOG(VERBOSE, "No datagram handler found for node %p id %x", d->dst,
            datagram_id);
        resultCode_ = DatagramClient::PERMANENT_ERROR;
        return allocate_and_call(iface()->addressed_message_write_flow(),
                                 STATE(respond_rejection));
    }

    h->send(d_);
    d_ = nullptr;
    return exit();
}

StateFlowBase::Action
DatagramService::DatagramDispatcher::respond_rejection()
{
    auto* f = get_allocation_result(iface()->addressed_message_write_flow());

    f->data()->reset(Defs::MTI_DATAGRAM_REJECTED, d_->data()->dst->node_id(),
                     d_->data()->src, error_to_buffer(resultCode_));
    
    iface()->addressed_message_write_flow()->send(f);
    
    d_->unref();
    d_ = nullptr;
    return exit();
}

} // namespace openlcb

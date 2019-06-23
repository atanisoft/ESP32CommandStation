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
 * \file DatagramCan.cxx
 *
 * CANbus datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 27 Jan 2013
 */

#include "openlcb/DatagramCan.hxx"

#include "openlcb/DatagramDefs.hxx"
#include "openlcb/DatagramImpl.hxx"
#include "openlcb/IfCanImpl.hxx"

namespace openlcb
{

/// Defines how long the datagram client flow should wait for the datagram
/// ack/nack response message.
extern long long DATAGRAM_RESPONSE_TIMEOUT_NSEC;

/// Datagram client implementation for CANbus-based datagram protocol.
///
/// This flow is responsible for the outgoing CAN datagram framing, and listens
/// for incoming datagram response messages.
///
/// The base class of AddressedCanMessageWriteFlow is responsible for the
/// discovery and address resolution of the destination node.
class CanDatagramWriteFlow : public AddressedCanMessageWriteFlow
{
public:
    CanDatagramWriteFlow(IfCan *iface)
        : AddressedCanMessageWriteFlow(iface)
    {
    }


private:
    Action fill_can_frame_buffer() OVERRIDE
    {
        LOG(VERBOSE, "fill can frame buffer");
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        struct can_frame *f = b->data()->mutable_frame();
        HASSERT(nmsg()->mti == Defs::MTI_DATAGRAM);

        // Sets the CAN id.
        uint32_t can_id = 0x1A000000;
        CanDefs::set_src(&can_id, srcAlias_);
        LOG(VERBOSE, "dst alias %x", dstAlias_);
        CanDefs::set_dst(&can_id, dstAlias_);

        bool need_more_frames = false;
        unsigned len = nmsg()->payload.size() - dataOffset_;
        if (len > 8)
        {
            len = 8;
            // This is not the last frame.
            need_more_frames = true;
            if (dataOffset_)
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_MIDDLE_FRAME);
            }
            else
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_FIRST_FRAME);
            }
        }
        else
        {
            // No more data after this frame.
            if (dataOffset_)
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_FINAL_FRAME);
            }
            else
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_ONE_FRAME);
            }
        }

        memcpy(f->data, &nmsg()->payload[dataOffset_], len);
        dataOffset_ += len;
        f->can_dlc = len;

        SET_CAN_FRAME_ID_EFF(*f, can_id);
        if_can()->frame_write_flow()->send(b);

        if (need_more_frames)
        {
            return call_immediately(STATE(get_can_frame_buffer));
        }
        else
        {
            return call_immediately(STATE(send_finished));
        }
    }
}; // CanDatagramWriteFlow

/** Frame handler that assembles incoming datagram fragments into a single
 * datagram message. (That is, datagrams addressed to local nodes.) */
class CanDatagramParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::FRAME_TYPE_MASK | CanDefs::PRIORITY_MASK |
            CanDefs::CAN_FRAME_TYPE_MASK,
    };

    CanDatagramParser(IfCan *iface);
    ~CanDatagramParser();

    /// Handler callback for incoming frames.
    Action entry() override
    {
        errorCode_ = 0;
        const struct can_frame *f = &message()->data()->frame();

        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        unsigned can_frame_type = (id & CanDefs::CAN_FRAME_TYPE_MASK) >>
                                  CanDefs::CAN_FRAME_TYPE_SHIFT;

        if (can_frame_type < 2 || can_frame_type > 5)
        {
            // Not datagram frame.
            return release_and_exit();
        }

        srcAlias_ = (id & CanDefs::SRC_MASK) >> CanDefs::SRC_SHIFT;

        uint64_t buffer_key = id & (CanDefs::DST_MASK | CanDefs::SRC_MASK);

        dst_.alias = buffer_key >> (CanDefs::DST_SHIFT);
        dstNode_ = nullptr;
        dst_.id = if_can()->local_aliases()->lookup(NodeAlias(dst_.alias));
        if (dst_.id)
        {
            dstNode_ = if_can()->lookup_local_node(dst_.id);
        }
        if (!dstNode_)
        {
            // Destination not local node.
            return release_and_exit();
        }

        DatagramPayload *buf = nullptr;
        bool last_frame = true;

        switch (can_frame_type)
        {
            case 2:
                // Single-frame datagram. Let's allocate one small buffer for
                // it.
                localBuffer_.clear();
                localBuffer_.reserve(f->can_dlc);
                buf = &localBuffer_;
                break;
            case 3:
            {
                // Datagram first frame
                auto it = pendingBuffers_.find(buffer_key);
                if (it != pendingBuffers_.end())
                {
                    pendingBuffers_.erase(it);
                    /** Frames came out of order or more than one datagram is
                     * being sent to the same dst. */
                    errorCode_ = DatagramClient::RESEND_OK |
                                 DatagramClient::OUT_OF_ORDER;
                    break;
                }

                buf = &pendingBuffers_[buffer_key];
                buf->clear();

                // Datagram first frame. Get a full buffer.
                buf->reserve(72);
                last_frame = false;
                break;
            }
            case 4:
                // Datagram middle frame
                last_frame = false;
            // Fall through
            case 5:
            {
                // Datagram last frame
                auto it = pendingBuffers_.find(buffer_key);
                if (it != pendingBuffers_.end())
                {
                    buf = &it->second;
                    if (last_frame)
                    {
                        localBuffer_.clear();
                        // Moves ownership of the allocated data to the local
                        // buffer.
                        localBuffer_.swap(*buf);
                        buf = &localBuffer_;
                        pendingBuffers_.erase(it);
                    }
                }
                break;
            }
            default:
                // Not datagram frame.
                return release_and_exit();
        }

        if (!buf)
        {
            errorCode_ =
                DatagramClient::RESEND_OK | DatagramClient::OUT_OF_ORDER;
        }
        else if (buf->size() + f->can_dlc > DatagramDefs::MAX_SIZE)
        {
            // Too long datagram arrived.
            LOG(WARNING, "AsyncDatagramCan: too long incoming datagram arrived."
                         " Size: %d",
                (int)(buf->size() + f->can_dlc));
            errorCode_ = DatagramClient::PERMANENT_ERROR;
            // Since we reject the datagram, let's not keep the buffer
            // around. This call should not crash if the buffer was already
            // deleted.
            pendingBuffers_.erase(buffer_key);
        }

        if (errorCode_)
        {
            release();
            // Gets the send flow to send rejection.
            return allocate_and_call(if_can()->addressed_message_write_flow(),
                                     STATE(send_rejection));
        }

        // Copies new data into buf.
        buf->append(reinterpret_cast<const char *>(&f->data[0]), f->can_dlc);
        release();
        if (last_frame)
        {
            HASSERT(buf == &localBuffer_);
            // Datagram is complete; let's send it to higher level If.
            return allocate_and_call(if_can()->dispatcher(),
                                     STATE(datagram_complete));
        }
        else
        {
            return exit();
        }
    }

    /** Sends a datagram rejection. The lock_ is held and must be
     * released. entry is an If::addressed write flow. errorCode_ != 0. */
    Action send_rejection()
    {
        HASSERT(errorCode_);
        HASSERT(dstNode_);
        auto *f =
            get_allocation_result(if_can()->addressed_message_write_flow());
        f->data()->reset(Defs::MTI_DATAGRAM_REJECTED, dst_.id, {0, srcAlias_},
                         error_to_buffer(errorCode_));
        if_can()->addressed_message_write_flow()->send(f);
        return exit();
    }

    /** Requests the datagram in buf_, dstNode_ etc... to be sent to the
     * AsyncIf for processing. The lock_ is held and must be released. entry is
     * the dispatcher. */
    Action datagram_complete()
    {
        HASSERT(!errorCode_);
        auto *f = get_allocation_result(if_can()->dispatcher());
        GenMessage *m = f->data();
        m->mti = Defs::MTI_DATAGRAM;
        m->payload.swap(localBuffer_);
        m->dst = dst_;
        m->dstNode = dstNode_;
        m->src.alias = srcAlias_;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            // It's unlikely to have a datagram coming in on the interface with
            // a local alias and still framed into CAN frames. But we still
            // handle it.
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(f);
        return exit();
    }

private:
    /// A local buffer that owns the datagram payload bytes after we took the
    /// entry from the pending buffers map.
    DatagramPayload localBuffer_;

    Node *dstNode_;
    NodeHandle dst_;
    unsigned short srcAlias_ : 12;
    /// If non-zero, contains a Rejection error code and the datagram should not
    /// be forwarded to the upper layer in this case.
    uint16_t errorCode_;

    /** Open datagram buffers. Keyed by (dstid | srcid), value is a datagram
     * payload. When a payload is finished, it should be moved into the final
     * datagram message using swap() to avoid memory copies.
     * @TODO(balazs.racz) we need some kind of timeout-based release mechanism
     * in here. */
    StlMap<uint64_t, DatagramPayload> pendingBuffers_;
};
CanDatagramService::CanDatagramService(IfCan *iface,
                                       int num_registry_entries,
                                       int num_clients)
    : DatagramService(iface, num_registry_entries)
{
    if_can()->add_owned_flow(new CanDatagramParser(if_can()));
    auto* dg_send = new CanDatagramWriteFlow(if_can());
    if_can()->add_owned_flow(dg_send);
    for (int i = 0; i < num_clients; ++i)
    {
        auto *client_flow = new DatagramClientImpl(if_can(), dg_send);
        if_can()->add_owned_flow(client_flow);
        client_allocator()->insert(static_cast<DatagramClient *>(client_flow));
    }
}

Executable *TEST_CreateCanDatagramParser(IfCan *if_can)
{
    return new CanDatagramParser(if_can);
}

CanDatagramService::~CanDatagramService()
{
}

CanDatagramParser::CanDatagramParser(IfCan *iface)
    : CanFrameStateFlow(iface)
{
    if_can()->frame_dispatcher()->register_handler(this,
        CAN_FILTER |
            (CanDefs::DATAGRAM_ONE_FRAME << CanDefs::CAN_FRAME_TYPE_SHIFT),
        CAN_MASK &
            ~((CanDefs::DATAGRAM_ONE_FRAME ^ CanDefs::DATAGRAM_FIRST_FRAME)
                << CanDefs::CAN_FRAME_TYPE_SHIFT));
    if_can()->frame_dispatcher()->register_handler(this,
        CAN_FILTER |
            (CanDefs::DATAGRAM_MIDDLE_FRAME << CanDefs::CAN_FRAME_TYPE_SHIFT),
        CAN_MASK &
            ~((CanDefs::DATAGRAM_MIDDLE_FRAME ^ CanDefs::DATAGRAM_FINAL_FRAME)
                << CanDefs::CAN_FRAME_TYPE_SHIFT));
}

CanDatagramParser::~CanDatagramParser()
{
    if_can()->frame_dispatcher()->unregister_handler_all(this);
}

} // namespace openlcb

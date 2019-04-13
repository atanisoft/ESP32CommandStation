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
 * \file IfCanImpl.hxx
 *
 * Implementation flows for writing to the CANbus.
 *
 * @author Balazs Racz
 * @date 27 Jan 2013
 */

#ifndef _OPENLCB_IFCANIMPL_HXX_
#define _OPENLCB_IFCANIMPL_HXX_

#include "openlcb/CanDefs.hxx"
#include "executor/StateFlow.hxx"
#include "openlcb/IfImpl.hxx"
#include "openlcb/AliasAllocator.hxx"

namespace openlcb
{

extern long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC;

/** Implements the write-side conversion logic from generic messages to CAN
 * frames. */
class CanMessageWriteFlow : public WriteFlowBase
{
public:
    CanMessageWriteFlow(IfCan *if_can) : WriteFlowBase(if_can)
    {
    }

    IfCan *if_can()
    {
        return static_cast<IfCan *>(async_if());
    }

protected:
    unsigned srcAlias_ : 12;  ///< Source node alias.
    unsigned dstAlias_ : 12;  ///< Destination node alias.
    unsigned dataOffset_ : 8; /**< for continuation frames: which offset in
                                * the Buffer should we start the payload at. */

    Action send_to_hardware() override
    {
        dataOffset_ = 0;
        srcAlias_ = 0;
        dstAlias_ = 0;
        if (nmsg()->payload.size())
        {
            // We have limited space for counting offsets. In practice this
            // value will be max 10 for certain traction control protocol
            // messages. Longer data usually travels via datagrams or streams.
            HASSERT(nmsg()->payload.size() < 256);
        }
        return call_immediately(STATE(find_local_alias));
    }

    /** Performs the local alias lookup and branches depending on whether we
     * found a local alias or not. */
    Action find_local_alias()
    {
        // We are on the IF's executor, so we can access the alias caches.
        srcAlias_ = if_can()->local_aliases()->lookup(nmsg()->src.id);
        if (!srcAlias_)
        {
            return call_immediately(STATE(allocate_new_alias));
        }
        return src_alias_lookup_done();
    }

private:
    Action allocate_new_alias()
    {
        /** At this point we assume that there will always be at least one
         * alias that is reserved but not yet used. If we want to break that
         * assumption we need to have a method to kick off an alias allocation
         * right here.
         *
         * @TODO(balazs.racz): implement proper local alias reclaim/reuse
         * mechanism. */
        HASSERT(if_can()->alias_allocator());
        return allocate_and_call(
            STATE(take_new_alias),
            if_can()->alias_allocator()->reserved_aliases());
    }

    Action take_new_alias()
    {
        /* In this function we do a number of queries to the local alias
         * cache. It is important that these queries show a consistent state;
         * we do not need to hold any mutex though because only the current
         * executor is allowed to access that object. */
        NodeAlias alias = 0;
        {
            Buffer<AliasInfo> *new_alias =
                full_allocation_result(if_can()->alias_allocator());
            HASSERT(new_alias->data()->alias);
            alias = new_alias->data()->alias;
            /* Sends the alias back for reallocating. This will trigger the
             * alias allocator flow. */
            if (new_alias->data()->return_to_reallocation)
            {
                new_alias->data()->reset();
                if_can()->alias_allocator()->send(new_alias);
            }
            else
            {
                new_alias->unref();
            }
        }
        LOG(INFO, "Allocating new alias %03X for node %012" PRIx64, alias,
            nmsg()->src.id);

        // Checks that there was no conflict on this alias.
        if (if_can()->local_aliases()->lookup(alias) !=
            AliasCache::RESERVED_ALIAS_NODE_ID)
        {
            LOG(INFO, "Alias has seen conflict: %03X", alias);
            // Problem. Let's take another alias.
            return call_immediately(STATE(allocate_new_alias));
        }

        srcAlias_ = alias;
        /** @TODO(balazs.racz): We leak aliases here in case of eviction by the
         * AliasCache object. */
        if_can()->local_aliases()->add(nmsg()->src.id, alias);
        // Take a CAN frame to send off the AMD frame.
        return allocate_and_call(if_can()->frame_write_flow(),
                                 STATE(send_amd_frame));
    }

    Action send_amd_frame()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        struct can_frame *f = b->data()->mutable_frame();
        CanDefs::control_init(*f, srcAlias_, CanDefs::AMD_FRAME, 0);
        f->can_dlc = 6;
        uint64_t rd = htobe64(nmsg()->src.id);
        memcpy(f->data, reinterpret_cast<uint8_t *>(&rd) + 2, 6);
        if_can()->frame_write_flow()->send(b);
        // Source alias is settled.
        return src_alias_lookup_done();
    }

    Action src_alias_lookup_done()
    {
        return call_immediately(STATE(get_can_frame_buffer));
    }

protected:
    Action get_can_frame_buffer()
    {
        return allocate_and_call(if_can()->frame_write_flow(),
                                 STATE(fill_can_frame_buffer));
    }

private:
    virtual Action fill_can_frame_buffer()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        b->set_done(message()->new_child());
        struct can_frame *f = b->data()->mutable_frame();
        if (nmsg()->mti & (Defs::MTI_DATAGRAM_MASK | Defs::MTI_SPECIAL_MASK |
                           Defs::MTI_RESERVED_MASK))
        {
            // We don't know how to handle such an MTI in a generic way.
            b->unref();
            return call_immediately(STATE(send_finished));
        }
        // CAN has only 12 bits of MTI field, so we better fit.
        HASSERT(!(nmsg()->mti & ~0xfff));

        // Sets the CAN id.
        uint32_t can_id = 0;
        CanDefs::set_fields(&can_id, srcAlias_, nmsg()->mti,
                          CanDefs::GLOBAL_ADDRESSED, CanDefs::NMRANET_MSG,
                          CanDefs::NORMAL_PRIORITY);
        SET_CAN_FRAME_ID_EFF(*f, can_id);

        const string &data = nmsg()->payload;
        bool need_more_frames = false;
        // Sets the destination bytes if needed. Adds the payload.
        if (Defs::get_mti_address(nmsg()->mti))
        {
            f->data[0] = dstAlias_ >> 8;
            f->data[1] = dstAlias_ & 0xff;
            if (data.empty())
            {
                f->can_dlc = 2;
            }
            else
            {
                if (dataOffset_)
                {
                    // This is not the first frame.
                    f->data[0] |= CanDefs::NOT_FIRST_FRAME;
                }
                const char *b = data.data();
                unsigned len = data.size() - dataOffset_;
                if (len > 6)
                {
                    len = 6;
                    // This is not the last frame.
                    need_more_frames = true;
                    f->data[0] |= CanDefs::NOT_LAST_FRAME;
                }
                memcpy(f->data + 2, b + dataOffset_, len);
                dataOffset_ += len;
                f->can_dlc = 2 + len;
            }
        }
        else
        {
            if (data.size())
            {
                HASSERT(data.size() <= 8); // too big frame for global msg
                memcpy(f->data, data.data(), data.size());
                f->can_dlc = data.size();
            }
        }
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
};

/** The addressed write flow is responsible for sending addressed messages to
 * the CANbus. It uses some shared states from the generic CAN write flow base
 * class, and extends it with destination alias lookup states.  */
class AddressedCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    AddressedCanMessageWriteFlow(IfCan *if_can)
        : CanMessageWriteFlow(if_can)
        , timer_(this)
    {
    }

protected:
    /** Entry point of message handling flow. We start with the shared
     * implementation. We may get back control at the send_to_hardware
     * action. */
    Action entry() override
    {
        return call_immediately(STATE(addressed_entry));
    }

    Action send_to_hardware() override
    {
        dataOffset_ = 0;
        srcAlias_ = 0;
        dstAlias_ = 0;
        if (nmsg()->payload.size())
        {
            // We have limited space for counting offsets. In practice this
            // value will be max 10 for certain traction control protocol
            // messages. Longer data usually travels via datagrams or streams.
            HASSERT(nmsg()->payload.size() < 256);
        }
        NodeHandle &dst_ = nmsg()->dst;
        HASSERT(dst_.id || dst_.alias); // We must have some kind of address.
        if (dst_.id)
        {
            dstAlias_ = if_can()->remote_aliases()->lookup(dst_.id);
            if (dstAlias_ == NOT_RESPONDING)
            {
                LOG(INFO, "AddressedWriteFlow: Could not resolve destination "
                          "address %012" PRIx64
                          " to an alias on the bus. Dropping packet.",
                    nmsg()->dst.id);
                return call_immediately(STATE(send_finished));
            }
        }
        else if (dst_.alias)
        {
            // Check if this is a local node being called by alias.
            NodeID id = if_can()->local_aliases()->lookup(dst_.alias);
            if (id)
            {
                Node *dst_node = if_can()->lookup_local_node(id);
                if (dst_node)
                {
                    dst_.id = id;
                    nmsg()->dstNode = dst_node;
                    return call_immediately(STATE(send_to_local_node));
                }
            }
        }
        if (dst_.alias && dstAlias_ && dst_.alias != dstAlias_)
        {
            /** @TODO(balazs.racz) what should we do here? The caller said
             * something different than the cache. */
            LOG(INFO, "AddressedWriteFlow: Caller supplied outdated alias. "
                      "Node id %012" PRIx64 ", Result from cache %03x, "
                      "alias from caller %03x.",
                dst_.id, dstAlias_, dst_.alias);
        }
        else if (!dstAlias_)
        {
            dstAlias_ = dst_.alias;
        }
        if (dstAlias_)
        {
            return call_immediately(STATE(find_local_alias));
        }
        else
        {
            return call_immediately(STATE(find_remote_alias));
        }
    }

    Action find_remote_alias()
    {
        aliasListener_.RegisterLocalHandler();
        srcAlias_ =
            if_can()->local_aliases()->lookup(nmsg()->src.id);
        if (srcAlias_)
        {
            // We can try to send an AME (alias mapping enquiry) frame.
            return allocate_and_call(if_can()->frame_write_flow(),
                                     STATE(send_ame_frame));
        }
        // No local alias -- we need to jump straight to local nodeid
        // verify. That will allocate a new local alias.
        return call_immediately(STATE(send_verify_nodeid_global));
    }

    Action send_ame_frame()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        struct can_frame *f = b->data();
        CanDefs::control_init(*f, srcAlias_, CanDefs::AME_FRAME, 0);
        f->can_dlc = 6;
        uint64_t rd = htobe64(nmsg()->dst.id);
        memcpy(f->data, reinterpret_cast<uint8_t *>(&rd) + 2, 6);
        if_can()->frame_write_flow()->send(b);
        // We wait for a remote response to come via the handler input. If it
        // doesn't come for 1 sec, go to more global and send a global
        // unaddressed message inquiring the node id.
        return sleep_and_call(&timer_, ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC,
                              STATE(send_verify_nodeid_global));
    }

    Action send_verify_nodeid_global()
    {
        if (dstAlias_)
        {
            return call_immediately(STATE(remote_alias_found));
        }
        return allocate_and_call(if_can()->global_message_write_flow(),
                                 STATE(fill_verify_nodeid_global));
    }

    Action fill_verify_nodeid_global()
    {
        auto *b = get_allocation_result(if_can()->global_message_write_flow());
        b->data()->reset(Defs::MTI_VERIFY_NODE_ID_GLOBAL,
                         nmsg()->src.id,
                         node_id_to_buffer(nmsg()->dst.id));
        if_can()->global_message_write_flow()->send(b);
        return sleep_and_call(&timer_, ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC,
                              STATE(wait_looking_for_dst));
    }

    Action wait_looking_for_dst()
    {
        if (dstAlias_ && dstAlias_ <= 0xFFF)
        {
            return call_immediately(STATE(remote_alias_found));
        }
        LOG(INFO, "AddressedWriteFlow: Could not resolve destination "
                  "address %012" PRIx64 " to an alias on the bus. Dropping packet.",
            nmsg()->dst.id);
        aliasListener_.UnregisterLocalHandler();
        if_can()->remote_aliases()->add(nmsg()->dst.id, NOT_RESPONDING);
        return call_immediately(STATE(timeout_looking_for_dst));
    }

    virtual Action timeout_looking_for_dst()
    {
        /// @todo: generate a terminate due to error response message.
        return call_immediately(STATE(send_finished));
    }

    Action remote_alias_found()
    {
        HASSERT(dstAlias_);
        return call_immediately(STATE(find_local_alias));
    }

    /** This helper class listend for incoming CAN frames looking for alias
     * mapping definitions. */
    class AliasDefListener : public IncomingFrameHandler
    {
    public:
        AliasDefListener(AddressedCanMessageWriteFlow* parent)
            : parent_(parent) {}

        enum
        {
            // AMD frames
            CAN_FILTER1 = CanMessageData::CAN_EXT_FRAME_FILTER | 0x10701000,
            CAN_MASK1 = CanMessageData::CAN_EXT_FRAME_MASK | 0x1FFFF000,
            // Initialization complete
            CAN_FILTER2 = CanMessageData::CAN_EXT_FRAME_FILTER | 0x19100000,
            CAN_MASK2 = CanMessageData::CAN_EXT_FRAME_MASK | 0x1FFFF000,
            // Verified node ID number
            CAN_FILTER3 = CanMessageData::CAN_EXT_FRAME_FILTER | 0x19170000,
            CAN_MASK3 = CanMessageData::CAN_EXT_FRAME_MASK | 0x1FFFF000,
        };

        IfCan* if_can() { return parent_->if_can(); }

        // Registers *this to the ifcan to receive alias resolution messages.
        void RegisterLocalHandler()
        {
            if_can()->frame_dispatcher()->register_handler(
                this, CAN_FILTER1, CAN_MASK1);
            if_can()->frame_dispatcher()->register_handler(
                this, CAN_FILTER2, CAN_MASK2);
            if_can()->frame_dispatcher()->register_handler(
                this, CAN_FILTER3, CAN_MASK3);
        }

        // Unregisters *this from the ifcan to not receive alias resolution
        // messages.
        void UnregisterLocalHandler()
        {
            if_can()->frame_dispatcher()->unregister_handler(
                this, CAN_FILTER1, CAN_MASK1);
            if_can()->frame_dispatcher()->unregister_handler(
                this, CAN_FILTER2, CAN_MASK2);
            if_can()->frame_dispatcher()->unregister_handler(
                this, CAN_FILTER3, CAN_MASK3);
        }

        /// Handler callback for incoming messages.
        void send(Buffer<CanMessageData> *message, unsigned priority) OVERRIDE
        {
            struct can_frame *f = message->data();
            uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
            if (f->can_dlc != 6)
            {
                // Not sending a node ID.
                message->unref();
                return;
            }
            uint64_t nodeid_be = htobe64(parent_->nmsg()->dst.id);
            uint8_t *nodeid_start = reinterpret_cast<uint8_t *>(&nodeid_be) + 2;
            if (memcmp(nodeid_start, f->data, 6))
            {
                // Node id does not match.
                message->unref();
                return;
            }
            // Now: we have an alias.
            parent_->dstAlias_ = id & CanDefs::SRC_MASK;
            if (!parent_->dstAlias_)
            {
                LOG_ERROR("Incoming alias definition message with zero alias. "
                           "CAN frame id %08x",
                    (unsigned)id);
                message->unref();
                return;
            }
            UnregisterLocalHandler();
            if_can()->remote_aliases()->add(parent_->nmsg()->dst.id, parent_->dstAlias_);
            parent_->timer_.trigger();
            message->unref();
        }

    private:
        AddressedCanMessageWriteFlow* parent_;
    } aliasListener_{this};

    friend class AliasDefListener;

    StateFlowTimer timer_;
};
} // namespace openlcb

#endif // _OPENLCB_IFCANIMPL_HXX_

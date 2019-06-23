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
 * \file IfCan.cxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#include "openlcb/IfCan.hxx"

#include "utils/StlMap.hxx"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/IfImpl.hxx"
#include "openlcb/IfCanImpl.hxx"
#include "openlcb/CanDefs.hxx"
#include "can_frame.h"

namespace openlcb
{

size_t g_alias_use_conflicts = 0;

/** Specifies how long to wait for a response to an alias mapping enquiry
 * message when trying to send an addressed message to a destination. The final
 * timeout will be twice this time, because after the first timeout a global
 * message for verify node id request will also be sent out.
 *
 * This value is writable for unittesting purposes. We might consider moving it
 * to flash with a weak definition instead. */
extern long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC;
long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC = SEC_TO_NSEC(1);

/** This write flow inherits all the business logic from the parent, just
 * maintains a separate allocation queue. This allows global messages to go out
 * even if addressed messages are waiting for destination address
 * resolution. */
class GlobalCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    GlobalCanMessageWriteFlow(IfCan *if_can)
        : CanMessageWriteFlow(if_can)
    {
    }

protected:
    Action entry() override
    {
        return call_immediately(STATE(send_to_hardware));
    }

    Action send_finished() override
    {
        return call_immediately(STATE(global_entry));
    }
};

/** This class listens for incoming CAN messages, and if it sees a local alias
 * conflict, then takes the appropriate action:
 *
 * . if the conflict happened in alias check, it responds with an AMD frame.
 *
 * . if the conflict is with an allocated alias, kicks it out of the local
 * cache, forcing an alias reallocation for that node.
 *
 * . if the conflict is with a reserved but unused alias, kicks it out of the
 * cache. This condition will be detected when a new node tries using that
 * alias.
 */
class AliasConflictHandler : public CanFrameStateFlow
{
public:
    AliasConflictHandler(IfCan *service)
        : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, 0, ~((1 << 30) - 1));
    }

    ~AliasConflictHandler()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, 0, ~((1 << 30) - 1));
    }

    /// Handler callback for incoming messages.
    Action entry() override
    {
        uint32_t id = GET_CAN_FRAME_ID_EFF(*message()->data());
        release();
        if (CanDefs::get_priority(id) != CanDefs::NORMAL_PRIORITY)
        {
            // Probably not an OpenLCB frame.
            /// @TODO(balazs.racz) this is wrong. it IS an openlcb frame.
            return exit();
        }
        NodeAlias alias = CanDefs::get_src(id);
        // If the caller comes with alias 000, we ignore that.
        NodeID node = alias ? if_can()->local_aliases()->lookup(alias) : 0;
        if (!node)
        {
            // This is not a local alias of ours.
            return exit();
        }
        if (CanDefs::is_cid_frame(id))
        {
            // This is a CID frame. We own the alias, let them know.
            alias_ = alias;
            return allocate_and_call(
                if_can()->frame_write_flow(), STATE(send_reserved_alias));
        }
        /* Removes the alias from the local alias cache.  If it was assigned to
         * a node, the node will grab a new alias next time it tries to send
         * something.  If it was reserved but not used, then whoever tries to
         * use it will realize due to the RESERVED_ALIAS_NODE_ID guard missing
         * from the cache.  We do not aggressively start looking for a new
         * alias in place of the missing one. If we want to do that, we would
         * have to find the entry in the list of
         * ifCan_->alias_allocator()->reserved_alias_allocator() and put it
         * into the empty_alias_allocator() instead to be picked up by the
         * alias allocator flow.  */
        g_alias_use_conflicts++;
        if_can()->local_aliases()->remove(alias);
        return exit();
    }

    Action send_reserved_alias()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        // We are sending an RID frame.
        struct can_frame *f = b->data();
        CanDefs::control_init(*f, alias_, CanDefs::RID_FRAME, 0);
        if_can()->frame_write_flow()->send(b);
        return exit();
    }

private:
    /// Alias being checked.
    unsigned alias_ : 12;
};

/** This class listens for alias mapping frames and updates the remote alias
 * cache with the incoming information. */
class RemoteAliasCacheUpdater : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::CONTROL_MSG << CanDefs::FRAME_TYPE_SHIFT),
        CAN_MASK =
            CanMessageData::CAN_EXT_FRAME_MASK | CanDefs::FRAME_TYPE_MASK,
    };

    RemoteAliasCacheUpdater(IfCan *service)
        : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    ~RemoteAliasCacheUpdater()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    Action entry() OVERRIDE
    {
        struct can_frame *f = message()->data();
        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        if (CanDefs::get_frame_type(id) != CanDefs::CONTROL_MSG)
            return release_and_exit();
        auto control_field = CanDefs::get_control_field(id);
        NodeAlias alias = CanDefs::get_src(id);
        if (!alias)
        {
            return release_and_exit();
        }
        NodeID node_id = 0;
        if (f->can_dlc == 6)
        {
            node_id = data_to_node_id(f->data);
        }
        switch (control_field)
        {
            case CanDefs::AMD_FRAME:
            {
                if (!node_id)
                {
                    return release_and_exit();
                }
                NodeAlias old_alias =
                    if_can()->remote_aliases()->lookup(node_id);
                if (old_alias == alias)
                {
                    // No change.
                    return release_and_exit();
                }
                if (old_alias)
                {
                    if_can()->remote_aliases()->remove(old_alias);
                }
                if_can()->remote_aliases()->add(node_id, alias);
                return release_and_exit();
            }
            case CanDefs::AMR_FRAME:
            {
                if (node_id)
                {
                    NodeAlias old_alias =
                        if_can()->remote_aliases()->lookup(node_id);
                    if (old_alias && old_alias != alias)
                    {
                        if_can()->remote_aliases()->remove(old_alias);
                    }
                }
                if_can()->remote_aliases()->remove(alias);
                return release_and_exit();
            }
            default: // ignore
                ;
        }

        return release_and_exit();
    }
};

/** This class listens for alias mapping enquiry frames targeted for local
 * nodes, and replies with AMD frames. */
class AMEQueryHandler : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::CONTROL_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::AME_FRAME << CanDefs::CONTROL_FIELD_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::FRAME_TYPE_MASK | CanDefs::CONTROL_FIELD_MASK,
    };

    AMEQueryHandler(IfCan *service)
        : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    ~AMEQueryHandler()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    Action entry() OVERRIDE
    {
        struct can_frame *f = message()->data();
        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        if (CanDefs::get_frame_type(id) != CanDefs::CONTROL_MSG) {
            return release_and_exit();}
        if (CanDefs::get_control_field(id) != CanDefs::AME_FRAME) {
            return release_and_exit();}
        NodeID node_id = 0;
        if (f->can_dlc == 6)
        {
            node_id = data_to_node_id(f->data);
        }
        else
        {
            return release_and_exit();
        }
        NodeAlias local_alias = if_can()->local_aliases()->lookup(node_id);
        if (!node_id || !local_alias)
        {
            return release_and_exit();
        }
        auto* b = reinterpret_cast<Buffer<CanHubData>*>(transfer_message());
        id = CanDefs::set_control_fields(local_alias, CanDefs::AMD_FRAME, 0);
        SET_CAN_FRAME_ID_EFF(*b->data()->mutable_frame(), id);
        if_can()->frame_write_flow()->send(b);
        return exit();
    }
};

/** This class listens for Alias Mapping Enquiry frames with no destination
 * node ID (aka global alias enquiries) and sends back as many frames as wel
 * have local aliases mapped. */
class AMEGlobalQueryHandler : public StateFlowBase,
                              private FlowInterface<Buffer<CanMessageData>>
{
public:
    AMEGlobalQueryHandler(IfCan *service)
        : StateFlowBase(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    ~AMEGlobalQueryHandler()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    using MessageType = Buffer<CanMessageData>;

private:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::CONTROL_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::AME_FRAME << CanDefs::CONTROL_FIELD_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::FRAME_TYPE_MASK | CanDefs::CONTROL_FIELD_MASK,
    };

    IfCan *if_can()
    {
        return static_cast<IfCan *>(service());
    }

    /** Sends a message to the state flow for processing. This function never
     * blocks.
     *
     * Must be called from the main executor of the interface.
     *
     * @param msg Message to enqueue
     * @param priority the priority at which to enqueue this message.
     */
    void send(MessageType *msg, unsigned priority = UINT_MAX) override
    {
        AutoReleaseBuffer<CanMessageData> rb(msg);
        struct can_frame *f = msg->data();
        if (f->can_dlc != 0)
        {
            return;
        }
        needRerun_ = true;
        if (is_terminated())
        {
            start_flow(STATE(rerun));
        }
    }

    Action rerun()
    {
        needRerun_ = false;
        nextIndex_ = 0;
        return call_immediately(STATE(find_next));
    }

    Action find_next()
    {
        while (nextIndex_ < if_can()->local_aliases()->size())
        {
            if (if_can()->local_aliases()->retrieve(
                    nextIndex_, nullptr, nullptr))
            {
                return allocate_and_call(
                    if_can()->frame_write_flow(), STATE(fill_response));
            }
            nextIndex_++;
        }
        if (needRerun_)
        {
            return call_immediately(STATE(rerun));
        }
        else
        {
            return exit();
        }
    }

    Action fill_response()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        NodeID node;
        NodeAlias alias;
        if (if_can()->local_aliases()->retrieve(nextIndex_, &node, &alias))
        {
            struct can_frame *f = b->data()->mutable_frame();
            SET_CAN_FRAME_ID_EFF(
                *f, CanDefs::set_control_fields(alias, CanDefs::AMD_FRAME, 0));
            f->can_dlc = 6;
            node_id_to_data(node, f->data);
            b->set_done(n_.reset(this));
            if_can()->frame_write_flow()->send(b);
            nextIndex_++;
            return wait_and_call(STATE(find_next));
        }
        else
        {
            // The alias disappeared in the mean time. Release.
            b->unref();
            return call_immediately(STATE(find_next));
        }
    }

    /// This boolean will be set to true when a full re-run of all sent frames
    /// is necessary.
    bool needRerun_ = false;
    /// Which alias entry index we take next.
    unsigned nextIndex_;
    /// Helper object to wait for frame to be sent.
    BarrierNotifiable n_;
};

/** This class listens for incoming CAN frames of regular unaddressed global
 * OpenLCB messages, then translates it in a generic way into a message,
 * computing its MTI. The resulting message is then passed to the generic If
 * for dispatching. */
class FrameToGlobalMessageParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::GLOBAL_ADDRESSED << CanDefs::CAN_FRAME_TYPE_SHIFT) |
            (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::CAN_FRAME_TYPE_MASK | CanDefs::FRAME_TYPE_MASK |
            CanDefs::PRIORITY_MASK |
            (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT)
    };

    FrameToGlobalMessageParser(IfCan *service)
        : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    ~FrameToGlobalMessageParser()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    /// Handler entry for incoming messages.
    Action entry() OVERRIDE
    {
        struct can_frame *f = message()->data();
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        if (f->can_dlc)
        {
            buf_.assign((const char *)(&f->data[0]), f->can_dlc);
        }
        else
        {
            buf_.clear();
        }
        release();
        // Get the dispatch flow.
        return allocate_and_call(if_can()->dispatcher(), STATE(send_to_if));
    }

    Action send_to_if()
    {
        auto *b = get_allocation_result(if_can()->dispatcher());
        GenMessage *m = b->data();
        m->mti = static_cast<Defs::MTI>(
            (id_ & CanDefs::MTI_MASK) >> CanDefs::MTI_SHIFT);
        m->payload = buf_;
        m->dst = {0, 0};
        m->dstNode = nullptr;
        m->src.alias = id_ & CanDefs::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(b, b->data()->priority());
        return exit();
    }

private:
    /// CAN frame ID, saved from the incoming frame.
    uint32_t id_;
    /// Payload for the MTI message.
    string buf_;
};

/** This class listens for incoming CAN frames of regular addressed OpenLCB
 * messages destined for local nodes, then translates them in a generic way into
 * a message, computing its MTI. The resulting message is then passed to the
 * generic If for dispatching. */
class FrameToAddressedMessageParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::GLOBAL_ADDRESSED << CanDefs::CAN_FRAME_TYPE_SHIFT) |
            (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT) |
            (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::CAN_FRAME_TYPE_MASK | CanDefs::FRAME_TYPE_MASK |
            CanDefs::PRIORITY_MASK |
            (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT)
    };

    FrameToAddressedMessageParser(IfCan *service)
        : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    ~FrameToAddressedMessageParser()
    {
        if_can()->frame_dispatcher()->unregister_handler(
            this, CAN_FILTER, CAN_MASK);
    }

    /// Handler entry for incoming messages.
    Action entry() override
    {
        struct can_frame *f = message()->data();
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        // Do we have enough payload for the destination address?
        if (f->can_dlc < 2)
        {
            LOG(WARNING, "Incoming can frame addressed message without payload."
                         " can ID %08x data length %d",
                (unsigned)id_, f->can_dlc);
            // Drop the frame.
            return release_and_exit();
        }
        // Gets the destination address and checks if it is our node.
        dstHandle_.alias = (((unsigned)f->data[0] & 0xf) << 8) | f->data[1];
        dstHandle_.id = if_can()->local_aliases()->lookup(dstHandle_.alias);
        if (!dstHandle_.id) // Not destined for us.
        {
            LOG(VERBOSE, "Dropping addressed message not for local destination."
                         "id %08x Alias %03x",
                (unsigned)id_, dstHandle_.alias);
            // Drop the frame.
            return release_and_exit();
        }
        // Checks the continuation bits.
        if (f->data[0] & (CanDefs::NOT_FIRST_FRAME | CanDefs::NOT_LAST_FRAME))
        {
            uint64_t buffer_key = dstHandle_.alias;
            buffer_key <<= 12;
            buffer_key |= CanDefs::get_src(id_);
            buffer_key <<= 12;
            buffer_key |= CanDefs::get_mti(id_);
            /** @todo (balazs.racz): handle the error cases here, like when we
             * get a middle frame out of the blue etc. */
            Payload *mapped_buffer = &pendingBuffers_[buffer_key];
            if ((f->data[0] & CanDefs::NOT_FIRST_FRAME) == 0)
            {
                // First frame. Make sure the pending buffer is empty.
                if (!mapped_buffer->empty())
                {
                    LOG(WARNING, "Received multi-frame message when a previous "
                                 "multi-frame message has not been flushed "
                                 "yet. frame ID=%08x, fddd=%02x%02x",
                        (unsigned)id_, f->data[0], f->data[1]);
                }
                mapped_buffer->clear();
            }
            if (f->can_dlc > 2)
            {
                mapped_buffer->append(
                    (const char *)(f->data + 2), f->can_dlc - 2);
            }
            if (f->data[0] & CanDefs::NOT_LAST_FRAME)
            {
                // We are done for now
                return release_and_exit();
            }
            else
            {
                // Frame complete.
                mapped_buffer->swap(buf_);
                pendingBuffers_.erase(buffer_key);
            }
        }
        else
        {
            // Saves the payload.
            if (f->can_dlc > 2)
            {
                buf_.assign((const char *)(f->data + 2), f->can_dlc - 2);
            }
            else
            {
                buf_.clear();
            }
        }
        /** Frame not needed anymore. If we want to save the reserved bits from
         *  the first octet, we need to revise this. */
        release();
        return allocate_and_call(if_can()->dispatcher(), STATE(send_to_if));
    }

    Action send_to_if()
    {
        auto *b = get_allocation_result(if_can()->dispatcher());
        GenMessage *m = b->data();
        m->mti = static_cast<Defs::MTI>(
            (id_ & CanDefs::MTI_MASK) >> CanDefs::MTI_SHIFT);
        m->payload.swap(buf_);
        m->dst = dstHandle_;
        // This might be NULL if dst is a proxied node in a router.
        m->dstNode = if_can()->lookup_local_node(dstHandle_.id);
        m->src.alias = id_ & CanDefs::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(b, b->data()->priority());
        return exit();
    }

private:
    uint32_t id_;
    string buf_;
    NodeHandle dstHandle_;
    /// Reassembly buffers for multi-frame messages.
    StlMap<uint32_t, Payload> pendingBuffers_;
};

IfCan::IfCan(ExecutorBase *executor, CanHubFlow *device,
    int local_alias_cache_size, int remote_alias_cache_size,
    int local_nodes_count)
    : If(executor, local_nodes_count)
    , CanIf(this, device)
    , localAliases_(0, local_alias_cache_size)
    , remoteAliases_(0, remote_alias_cache_size)
{
    auto *gflow = new GlobalCanMessageWriteFlow(this);
    globalWriteFlow_ = gflow;
    add_owned_flow(gflow);

    add_owned_flow(new AliasConflictHandler(this));
    add_owned_flow(new FrameToGlobalMessageParser(this));
    add_owned_flow(new VerifyNodeIdHandler(this));
    add_owned_flow(new RemoteAliasCacheUpdater(this));
    add_owned_flow(new AMEQueryHandler(this));
    add_owned_flow(new AMEGlobalQueryHandler(this));
    add_addressed_message_support();
    /*pipe_member_.reset(new CanReadFlow(device, this, executor));
    for (int i = 0; i < hw_write_flow_count; ++i)
    {
        CanFrameWriteFlow *f = new CanWriteFlow(this, executor);
        write_allocator_.Release(f);
        owned_flows_.push_back(std::unique_ptr<ControlFlow>(f));
    }
    for (int i = 0; i < global_can_write_flow_count; ++i)
    {
        owned_flows_.push_back(
            std::unique_ptr<Executable>(new GlobalCanMessageWriteFlow(this)));
            }*/
}

IfCan::~IfCan()
{
}

void IfCan::add_owned_flow(Executable *e)
{
    ownedFlows_.push_back(std::unique_ptr<Executable>(e));
}

void IfCan::set_alias_allocator(AliasAllocator *a)
{
    aliasAllocator_.reset(a);
}

void IfCan::add_addressed_message_support()
{
    if (addressedWriteFlow_)
        return;
    add_owned_flow(new FrameToAddressedMessageParser(this));
    auto *f = new AddressedCanMessageWriteFlow(this);
    addressedWriteFlow_ = f;
    add_owned_flow(f);
}

void IfCan::delete_local_node(Node *node) {
    remove_local_node_from_map(node);
    auto alias = localAliases_.lookup(node->node_id());
    if (alias) {
        // The node had a local alias.
        localAliases_.remove(alias);
        localAliases_.add(AliasCache::RESERVED_ALIAS_NODE_ID, alias);
        // Sends AMR & returns alias to pool.
        aliasAllocator_->return_alias(node->node_id(), alias);
    }
}


void IfCan::canonicalize_handle(NodeHandle *h)
{
    if (!h->id & !h->alias)
        return;
    if (!h->id)
    {
        h->id = local_aliases()->lookup(h->alias);
    }
    if (!h->id)
    {
        h->id = remote_aliases()->lookup(h->alias);
    }
    if (!h->alias)
    {
        h->alias = local_aliases()->lookup(h->id);
    }
    if (!h->alias)
    {
        h->alias = remote_aliases()->lookup(h->id);
    }
}

bool IfCan::matching_node(NodeHandle expected, NodeHandle actual)
{
    canonicalize_handle(&expected);
    canonicalize_handle(&actual);
    if (expected.id && actual.id)
    {
        return expected.id == actual.id;
    }
    if (expected.alias && actual.alias)
    {
        return expected.alias == actual.alias;
    }
    // Cannot reconcile.
    LOG(VERBOSE, "Cannot reconcile expected and actual NodeHandles for "
                 "equality testing.");
    return false;
}

Node *IfCan::lookup_local_node_handle(NodeHandle h)
{
    if (!h.id)
    {
        h.id = local_aliases()->lookup(h.alias);
    }
    return lookup_local_node(h.id);
}

} // namespace openlcb

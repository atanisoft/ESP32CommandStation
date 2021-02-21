/** \copyright
 * Copyright (c) 2020 Balazs Racz
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
 * \file BulkAliasAllocator.cxx
 *
 * State flow for allocating many aliases at the same time.
 *
 * @author Balazs Racz
 * @date 14 Nov 2020
 */

#include "openlcb/BulkAliasAllocator.hxx"
#include "openlcb/CanDefs.hxx"
#include "utils/MakeUnique.hxx"

namespace openlcb
{

/// Implementation of the BulkAliasAllocatorInterface to allocate many aliases
/// at the same time.
class BulkAliasAllocator : public CallableFlow<BulkAliasRequest>
{
public:
    /// Constructor
    /// @param iface the openlcb CAN interface
    BulkAliasAllocator(IfCan *iface)
        : CallableFlow<BulkAliasRequest>(iface)
    {
    }

    /// Start of flow when a request arrives to allocate many aliases. Resets
    /// the internal state and goes on to start the allocation process.
    Action entry() override
    {
        startTime_ = os_get_time_monotonic();
        pendingAliasesByTime_.clear();
        pendingAliasesByKey_.clear();
        nextToStampTime_ = 0;
        nextToClaim_ = 0;
        if_can()->frame_dispatcher()->register_handler(&conflictHandler_, 0, 0);
        return call_immediately(STATE(send_cid_frames));
    }

    /// Picks a bunch of random aliases, sends CID frames for them to the bus.
    Action send_cid_frames()
    {
        unsigned needed = std::min(request()->numAliases_,
            (unsigned)(config_bulk_alias_num_can_frames() + 3) / 4);
        if (!needed)
        {
            return call_immediately(STATE(wait_for_results));
        }
        bn_.reset(this);
        for (unsigned i = 0; i < needed; ++i)
        {
            NodeAlias next_alias = if_can()->alias_allocator()->get_new_seed();
            auto if_id = if_can()->alias_allocator()->if_node_id();
            send_can_frame(next_alias, (if_id >> 36) & 0xfff, 7);
            send_can_frame(next_alias, (if_id >> 24) & 0xfff, 6);
            send_can_frame(next_alias, (if_id >> 12) & 0xfff, 5);
            send_can_frame(next_alias, (if_id >> 0) & 0xfff, 4);
            --request()->numAliases_;
            pendingAliasesByTime_.push_back({next_alias});
            pendingAliasesByKey_.insert({next_alias});
        }
        bn_.notify();
        return wait_and_call(STATE(stamp_time));
    }

    /// Adds the timestamps when the CID requests were sent out.
    Action stamp_time()
    {
        auto ctime = relative_time();
        for (unsigned i = nextToStampTime_; i < pendingAliasesByTime_.size();
             ++i)
        {
            pendingAliasesByTime_[i].cidTime_ = ctime;
        }
        nextToStampTime_ = pendingAliasesByTime_.size();
        // Go back to sending more CID frames as needed.
        return call_immediately(STATE(send_cid_frames));
    }

    /// Sends out the RID frames for any alias that the 200 msec has already
    /// elapsed, then waits a bit and tries again.
    Action wait_for_results()
    {
        if (nextToClaim_ == pendingAliasesByTime_.size())
        {
            return complete();
        }
        if (request()->numAliases_)
        {
            // Some conflicts were identified, go and allocate more.
            return call_immediately(STATE(send_cid_frames));
        }
        auto ctime = relative_time();
        unsigned num_sent = 0;
        bn_.reset(this);
        while ((nextToClaim_ < pendingAliasesByTime_.size()) &&
            (num_sent < (unsigned)(config_bulk_alias_num_can_frames())) &&
            (pendingAliasesByTime_[nextToClaim_].cidTime_ + ALLOCATE_DELAY <
                ctime))
        {
            NodeAlias a =
                (NodeAlias)(pendingAliasesByTime_[nextToClaim_].alias_);
            ++nextToClaim_;
            auto it = pendingAliasesByKey_.find(a);
            if (it->hasConflict_)
            {
                // we skip this alias because there was a conflict.
                continue;
            }
            if_can()->alias_allocator()->add_allocated_alias(a);
            ++num_sent;
            send_can_frame(a, CanDefs::RID_FRAME, 0);
        }
        if (bn_.abort_if_almost_done())
        {
            // no frame sent
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(10), STATE(wait_for_results));
        }
        else
        {
            bn_.notify();
            // Wait for outgoing frames to be gone and call this again.
            return wait();
        }
    }

    /// Called when all RID frames are sent out.
    Action complete()
    {
        if_can()->frame_dispatcher()->unregister_handler_all(&conflictHandler_);
        pendingAliasesByTime_.clear();
        pendingAliasesByKey_.clear();
        return return_ok();
    }

private:
    /// Callback from the stack for all incoming frames while we are
    /// operating. We sniff the alias uot of it and record any conflicts we
    /// see.
    /// @param message an incoming CAN frame.
    void handle_conflict(Buffer<CanMessageData> *message)
    {
        auto rb = get_buffer_deleter(message);
        auto alias = CanDefs::get_src(GET_CAN_FRAME_ID_EFF(*message->data()));
        auto it = pendingAliasesByKey_.find(alias);
        if (it != pendingAliasesByKey_.end() && !it->hasConflict_)
        {
            it->hasConflict_ = 1;
            ++request()->numAliases_;
        }
    }

    /// Listens to incoming CAN frames and handles alias conflicts.
    IncomingFrameHandler::GenericHandler conflictHandler_ {
        this, &BulkAliasAllocator::handle_conflict};

    /// How many count to wait before sending out the RID frames. One count is
    /// 10 msec (see { \link relative_time } ).
    static constexpr unsigned ALLOCATE_DELAY = 20;

    /// Sends a CAN control frame to the bus. Take a share of the barrier bn_
    /// to send with the frame.
    /// @param src source alias to use on the frame.
    /// @param control_field 16-bit control value (e.g. RID_FRAME, or 0 top
    /// nibble and a chunk of the unique node ID in the middle).
    /// @param sequence used for CID messages.
    void send_can_frame(NodeAlias src, uint16_t control_field, int sequence)
    {
        auto *b = if_can()->frame_write_flow()->alloc();
        b->set_done(bn_.new_child());
        CanDefs::control_init(*b->data(), src, control_field, sequence);
        if_can()->frame_write_flow()->send(b, 0);
    }

    /// @return the openlcb CAN interface
    IfCan *if_can()
    {
        return static_cast<IfCan *>(service());
    }

    /// @return the time elapsed from start time in 10 msec units.
    unsigned relative_time()
    {
        return (os_get_time_monotonic() - startTime_) / MSEC_TO_NSEC(10);
    }

    /// We store this type in the time-ordered aliases structure.
    struct PendingAliasInfo
    {
        /// Constructor
        /// @param alias the openlcb alias that is being represented here.
        PendingAliasInfo(NodeAlias alias)
            : alias_(alias)
            , cidTime_(0)
        {
        }

        /// The value of the alias
        unsigned alias_ : 12;
        /// The time when the CID requests were sent. Counter in
        /// relative_time(), i.e. 10 msec per increment.
        unsigned cidTime_ : 8;
    };
    static_assert(sizeof(PendingAliasInfo) == 4, "memory bloat");

    /// We store this type in the sorted map lookup structure.
    struct AliasLookupInfo
    {
        AliasLookupInfo(NodeAlias alias)
            : alias_(alias)
            , hasConflict_(0)
        {
        }

        /// The value of the alias
        uint16_t alias_ : 12;
        /// 1 if we have seen a conflict
        uint16_t hasConflict_ : 1;
    };
    static_assert(sizeof(AliasLookupInfo) == 2, "memory bloat");
    /// Comparator function on AliasLookupInfo objects.
    struct LookupCompare
    {
        bool operator()(AliasLookupInfo a, AliasLookupInfo b)
        {
            return a.alias_ < b.alias_;
        }
    };

    /// Helper object for sleeping.
    StateFlowTimer timer_ {this};
    /// Helper object to determine when the CAN frames have flushed from the
    /// system.
    BarrierNotifiable bn_;
    /// We measure time elapsed relative to this point.
    long long startTime_;
    /// Stores the aliases we are trying to allocate in time order of picking
    /// them.
    std::vector<PendingAliasInfo> pendingAliasesByTime_;
    /// Stores the aliases we are trying to allocate in the alias order.
    SortedListSet<AliasLookupInfo, LookupCompare> pendingAliasesByKey_;
    /// Index into the pendingAliasesByTime_ vector where we need to stmap
    /// time.
    uint16_t nextToStampTime_;
    /// Index into the pendingAliasesByTime_ vector where we need to send out
    /// the reserve frame.
    uint16_t nextToClaim_;
};

std::unique_ptr<BulkAliasAllocatorInterface> create_bulk_alias_allocator(
    IfCan *can_if)
{
    return std::make_unique<BulkAliasAllocator>(can_if);
}

} // namespace openlcb
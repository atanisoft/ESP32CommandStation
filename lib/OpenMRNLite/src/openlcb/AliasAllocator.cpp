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
 * \file AliasAllocator.cxx
 *
 * Asynchronous implementation of NMRAnet alias reservation algorithms.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#include "openlcb/AliasAllocator.hxx"
#include "openlcb/CanDefs.hxx"

namespace openlcb
{

size_t g_alias_test_conflicts = 0;

AliasAllocator::AliasAllocator(NodeID if_id, IfCan *if_can)
    : StateFlow<Buffer<AliasInfo>, QList<1>>(if_can)
    , conflictHandler_(this)
    , timer_(this)
    , if_id_(if_id)
    , cid_frame_sequence_(0)
    , conflict_detected_(0)
{
    reinit_seed();
    // Moves all the allocated alias buffers over to the input queue for
    // allocation.
}

void AliasAllocator::reinit_seed()
{
    seed_ = if_id_ >> 30;
    seed_ ^= if_id_ >> 18;
    seed_ ^= if_id_ >> 6;
    seed_ ^= uint16_t(if_id_ >> 42) | uint16_t(if_id_ << 6);
}

/** Helper function to instruct the async alias allocator to pre-allocate N
 * aliases.
 *
 * @param aliases is the async alias allocator to use.
 * @param pool is the pool from where we take the buffers for the aliases.
 * @param n is how many aliases we pre-allocate.
 */
void seed_alias_allocator(AliasAllocator* aliases, Pool* pool, int n) {
    for (int i = 0; i < n; i++)
    {
        Buffer<AliasInfo> *b;
        pool->alloc(&b);
        aliases->send(b);
    }
}

void AliasAllocator::return_alias(NodeID id, NodeAlias alias)
{
    // This is synchronous allocation, which is not nice.
    {
        auto *b = if_can()->frame_write_flow()->alloc();
        struct can_frame *f = b->data()->mutable_frame();
        CanDefs::control_init(*f, alias, CanDefs::AMR_FRAME, 0);
        f->can_dlc = 6;
        node_id_to_data(id, f->data);
        if_can()->frame_write_flow()->send(b);
    }

    // This is synchronous allocation, which is not nice.
    {
        auto* b = alloc();
        b->data()->reset();
        b->data()->alias = alias;
        b->data()->do_not_reallocate();
        b->data()->state = AliasInfo::STATE_RESERVED;
        reserved_aliases()->insert(b);
    }
}

AliasAllocator::~AliasAllocator()
{
}

StateFlowBase::Action AliasAllocator::entry()
{
    cid_frame_sequence_ = 7;
    conflict_detected_ = 0;
    HASSERT(pending_alias()->state == AliasInfo::STATE_EMPTY);
    while (!pending_alias()->alias)
    {
        pending_alias()->alias = seed_;
        next_seed();
        // TODO(balazs.racz): check if the alias is already known about.
    }
    // Registers ourselves as a handler for incoming CAN frames to detect
    // conflicts.
    if_can()->frame_dispatcher()->register_handler(
        &conflictHandler_, pending_alias()->alias, ~0x1FFFF000U);

    // Grabs an outgoing frame buffer.
    return call_immediately(STATE(handle_allocate_for_cid_frame));
}

void AliasAllocator::next_seed()
{
    uint16_t offset;
    offset = if_id_ >> 36;
    offset ^= if_id_ >> 24;
    offset ^= if_id_ >> 12;
    offset ^= if_id_;
    offset <<= 1;
    offset |= 1; // ensures offset is odd.
    // This offset will be guaranteed different for any two node IDs that
    // are within 2048 of each other. Therefore it is guaranteed that they
    // will generate a different alias after a conflict. It is also
    // guaranteed that we will generate every single possible alias before
    // generating a duplicate (the cycle length is always 2^12).

    seed_ += offset;
}

StateFlowBase::Action AliasAllocator::handle_allocate_for_cid_frame()
{
    if (cid_frame_sequence_ >= 4)
    {
        return allocate_and_call(if_can()->frame_write_flow(),
                                 STATE(send_cid_frame));
    }
    else
    {
        // All CID frames are sent, let's wait.
        return sleep_and_call(&timer_, MSEC_TO_NSEC(200), STATE(wait_done));
    }
}

StateFlowBase::Action AliasAllocator::send_cid_frame()
{
    LOG(VERBOSE, "Sending CID frame %d for alias %03x", cid_frame_sequence_,
        pending_alias()->alias);
    auto *b = get_allocation_result(if_can()->frame_write_flow());
    struct can_frame *f = b->data()->mutable_frame();
    if (conflict_detected_)
    {
        b->unref();
        return call_immediately(STATE(handle_alias_conflict));
    }
    CanDefs::control_init(*f, pending_alias()->alias,
                        (if_id_ >> (12 * (cid_frame_sequence_ - 4))) & 0xfff,
                        cid_frame_sequence_);
    b->set_done(n_.reset(this));
    if_can()->frame_write_flow()->send(b);
    --cid_frame_sequence_;
    return wait_and_call(STATE(handle_allocate_for_cid_frame));
}

StateFlowBase::Action AliasAllocator::handle_alias_conflict()
{
    // Marks that we are no longer interested in frames from this alias.
    if_can()->frame_dispatcher()->unregister_handler(
        &conflictHandler_, pending_alias()->alias, ~0x1FFFF000U);

    // Burns up the alias.
    pending_alias()->alias = 0;
    pending_alias()->state = AliasInfo::STATE_EMPTY;
    // Restarts the lookup.
    return call_immediately(STATE(entry));
}

StateFlowBase::Action AliasAllocator::wait_done()
{
    if (conflict_detected_)
    {
        return call_immediately(STATE(handle_alias_conflict));
    }
    // grab a frame buffer for the RID frame.
    return allocate_and_call(if_can()->frame_write_flow(),
                             STATE(send_rid_frame));
}

StateFlowBase::Action AliasAllocator::send_rid_frame()
{
    LOG(VERBOSE, "Sending RID frame for alias %03x", pending_alias()->alias);
    auto *b = get_allocation_result(if_can()->frame_write_flow());
    struct can_frame *f = b->data()->mutable_frame();
    CanDefs::control_init(*f, pending_alias()->alias, CanDefs::RID_FRAME, 0);
    if (conflict_detected_)
    {
        b->unref();
        return call_immediately(STATE(handle_alias_conflict));
    }
    if_can()->frame_write_flow()->send(b);
    // The alias is reserved, put it into the freelist.
    pending_alias()->state = AliasInfo::STATE_RESERVED;
    if_can()->frame_dispatcher()->unregister_handler(
        &conflictHandler_, pending_alias()->alias, ~0x1FFFF000U);
    if_can()->local_aliases()->add(AliasCache::RESERVED_ALIAS_NODE_ID,
                                   pending_alias()->alias);
    reserved_alias_pool_.insert(transfer_message());
    return release_and_exit();
}

void AliasAllocator::ConflictHandler::send(Buffer<CanMessageData> *message,
                                                unsigned priority)
{
    if (parent_->conflict_detected_) {
        message->unref();
        return;
    }
    parent_->conflict_detected_ = 1;
    g_alias_test_conflicts++;
    if (parent_->is_state(static_cast<StateFlowBase::Callback>(&AliasAllocator::wait_done))) {
        /* Wakes up the actual flow to not have to wait all the 200 ms of
         * sleep. This will request the timer callback to be issued
         * immediately, which avoids race condition between the trigger and the
         * regular timeout call. */
        parent_->timer_.trigger();
    }
    message->unref();
}

void AliasAllocator::TEST_finish_pending_allocation() {
    if (is_state(STATE(wait_done))) {
        timer_.trigger();
    }
}

void AliasAllocator::TEST_add_allocated_alias(NodeAlias alias, bool repeat)
{
    Buffer<AliasInfo> *a;
    mainBufferPool->alloc(&a);
    a->data()->reset();
    a->data()->alias = alias;
    a->data()->state = AliasInfo::STATE_RESERVED;
    if (!repeat)
    {
        a->data()->do_not_reallocate();
    }
    if_can()->local_aliases()->add(
        AliasCache::RESERVED_ALIAS_NODE_ID, a->data()->alias);
    reserved_aliases()->insert(a);
}

} // namespace openlcb

/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file SemaphoreNotifiableBlock.hxx
 *
 * An advanced notifiable construct that has a fixed number of slots, and
 * allows pending (including blocking a thread) for the availability of a slot,
 * keeping a BarrierNotifiable for each slot and automatically freeing the slot
 * for the next user when the Barrier goes out of scope.
 *
 * @author Balazs Racz
 * @date 12 July 2016
 */

#ifndef _EXECUTOR_SEMAPHORENOTIFIABLEBLOCK_HXX_
#define _EXECUTOR_SEMAPHORENOTIFIABLEBLOCK_HXX_

/// A block of BarrierNotifiable objects, with a synchronous allocation
/// call. Caller threads can block on allocating a new entry, and then get back
/// a fresh BarrierNotifiable, which, upon being released will automatically be
/// reallocated to a waiting thread, if any.
class SemaphoreNotifiableBlock : private Notifiable, private Atomic
{
public:
    /// constructor. @param num_parallelism tells how many BarrierNotifiables
    /// we should have and hand out to threads requesting them.
    SemaphoreNotifiableBlock(unsigned num_parallelism)
        : count_(num_parallelism)
        , sem_(num_parallelism)
        , barriers_(new BarrierNotifiable[num_parallelism])
    {
    }

    ~SemaphoreNotifiableBlock()
    {
        delete[] barriers_;
    }

    /// Gets a barrier notifiable. May block the current thread if there isn't
    /// one ready. @return a fresh BarrierNotifiable.
    BarrierNotifiable* acquire() {
        sem_.wait();
        AtomicHolder h(this);
        for (unsigned i = 0; i < count_; ++i) {
            if (barriers_[i].is_done()) {
                return barriers_[i].reset(this);
            }
        }
        DIE("SempahoreNotifiableBlock: could not find a free barrier.");
    }

    /// Internal: notifies that a barrier has been returned.
    void notify() override
    {
        sem_.post();
    }

#ifdef __FreeRTOS__
    void notify_from_isr() OVERRIDE
    {
        int woken = 0;
        sem_.post_from_isr(&woken);
    }
#endif

private:
    /// How many barriers did we allocate in total?
    unsigned count_;
    /// Semaphore holding free barriers.
    OSSem sem_;
    /// The raw pointer to the block of barriernotifiables.
    BarrierNotifiable *barriers_;

    DISALLOW_COPY_AND_ASSIGN(SemaphoreNotifiableBlock);
};

#endif // _EXECUTOR_SEMAPHORENOTIFIABLEBLOCK_HXX_

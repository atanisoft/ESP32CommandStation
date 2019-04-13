/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file LimitedPool.hxx
 *
 * Wrapper of an existing Pool that limits the number of buffers that can be
 * allocated at a given time. This can be used on a per-flow basis to avoid
 * infinite sized queues to build up.
 *
 * @author Balazs Racz
 * @date 6 April 2019
 */

#ifndef _UTILS_LIMITEDPOOL_HXX_
#define _UTILS_LIMITEDPOOL_HXX_

#include "utils/Buffer.hxx"

/// Implementation of a Pool interface that takes memory from mainBufferPool but
/// limits the number of buffers allocatable. Later async allocations will be
/// blocked until an earlier buffer gets freed. Freed buffers go back to the
/// main buffer pool, so no memory is captive in this object.
class LimitedPool : public Pool, private Atomic
{
public:
    /// @param entry_size is the byte size of the objects to be
    /// allocated. Usually sizeof(Buffer<YourType>).
    /// @param entry_count max number of buffer that can be allocated via this
    /// pool.
    LimitedPool(unsigned entry_size, unsigned entry_count)
        : itemSize_(entry_size)
        , freeCount_(entry_count)
    {
    }

    /// Number of free items in the pool.
    size_t free_items() override
    {
        return freeCount_;
    }

    /// Number of free items in the pool for a given allocation size.
    /// @param size size of interest
    /// @return number of free items in the pool for a given allocation size
    size_t free_items(size_t size) override
    {
        return size == itemSize_ ? freeCount_ : 0;
    }

protected:
    /// Internal helper funciton used by the Buffer implementation.
    BufferBase *alloc_untyped(size_t size, Executable *flow) override
    {
        HASSERT(size == itemSize_);
        if (!flow)
        {
            DIE("LimitedPool only supports async allocation.");
        }
        BufferBase *b = nullptr;
        {
            AtomicHolder h(this);
            if (freeCount_ > 0)
            {
                --freeCount_;
                b = base_pool()->alloc_untyped(size, nullptr);
                HASSERT(b);
                b->pool_ = this;
            }
        }
        if (b)
        {
            flow->alloc_result(b);
            return b;
        }
        else
        {
            waitingQueue_.insert(flow);
            return nullptr;
        }
    }

    /// Function called when a buffer refcount reaches zero.
    void free(BufferBase *item) override
    {
        HASSERT(item->size() == itemSize_);
        Executable *waiting = NULL;
        {
            AtomicHolder h(this);
            waiting = static_cast<Executable *>(waitingQueue_.next().item);
            if (!waiting)
            {
                ++freeCount_;
            }
        }
        if (waiting)
        {
            waiting->alloc_result(item);
        }
        else
        {
            base_pool()->free(item);
        }
    }

private:
    /// @return the pool from which we should get the actual memory we have.
    Pool *base_pool()
    {
        return mainBufferPool;
    }

    /// How many bytes each entry should be.
    uint16_t itemSize_;
    /// How many entries can still be allocated.
    uint16_t freeCount_;
    /// Async allocators waiting for free buffers.
    Q waitingQueue_;
};

#endif // _UTILS_LIMITEDPOOL_HXX_

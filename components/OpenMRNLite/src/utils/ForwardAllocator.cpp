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
 * \file ForwardAllocator.cxx
 *
 * An arena allocator, which is optimized to not be able to free individual
 * entries, only the entire allocator. Integrated with a buffer pool to reuse
 * large blocks of heap memory without being exposed to heap fragmentation.
 *
 * @author Balazs Racz
 * @date 25 Dec 2019
 */

#include "utils/ForwardAllocator.hxx"

#include "utils/Buffer.hxx"

// static
DynamicPool *ForwardAllocator::kbytePool_ =
    new DynamicPool(Bucket::init(sizeof(BufferType), 0));

static constexpr unsigned MALLOC_OVERHEAD = sizeof(void *);

#ifdef GTEST
// static
void ForwardAllocator::TEST_recreate_pool()
{
    delete kbytePool_;
    kbytePool_ = new DynamicPool(Bucket::init(sizeof(BufferType), 0));
}
#endif

ForwardAllocator::ForwardAllocator()
{
}

ForwardAllocator::~ForwardAllocator()
{
    OSMutexLock h(&lock_);
    while (!allocatedBlocks_.empty())
    {
        allocatedBlocks_.pop_front()->unref();
    }
}

void *ForwardAllocator::allocate(size_t size, size_t align)
{
    HASSERT(align <= alignof(primitive_t));
    if (size >= BLOCK_BYTE_SIZE)
    {
        // Large block. Allocate custom.
        size_t head = sizeof(Buffer<uint64_t>);
        head += size - sizeof(uint64_t);
        HASSERT(head < 65535); // a bit left for buffer header.
        auto *bufp =
            (Buffer<uint64_t> *)mainBufferPool->alloc_untyped(head, nullptr);
        HASSERT(bufp);
        // Links the new buffer into the queue.
        OSMutexLock h(&lock_);
        allocSize_ += head + MALLOC_OVERHEAD;
        if (allocatedBlocks_.empty())
        {
            allocatedBlocks_.SimpleQueue::push_front(bufp);
            offsetInLast_ = BLOCK_BYTE_SIZE + 1;
        }
        else
        {
            auto it = allocatedBlocks_.SimpleQueue::begin();
            ++it;
            allocatedBlocks_.insert(it, bufp);
        }
        return bufp->data();
    }
    OSMutexLock h(&lock_);
    size_t pad = offsetInLast_ % align;
    if (pad)
    {
        pad = (align - pad);
        allocWasted_ += pad;
    }
    offsetInLast_ += pad;
    if (offsetInLast_ + size > BLOCK_BYTE_SIZE)
    {
        allocWasted_ += BLOCK_BYTE_SIZE;
        allocWasted_ -= offsetInLast_;
        // Does not fit. Allocate new block.
        BufferType *new_block;
        kbytePool_->alloc(&new_block);
        allocSize_ += sizeof(BufferType) + MALLOC_OVERHEAD;
        HASSERT(new_block);
        allocatedBlocks_.push_front(new_block);
        offsetInLast_ = 0;
    }
    auto *ret = head_ptr() + offsetInLast_;
    offsetInLast_ += size;
    return ret;
}

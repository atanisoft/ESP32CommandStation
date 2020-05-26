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
 * \file ForwardAllocator.hxx
 *
 * An arena allocator, which is optimized to not be able to free individual
 * entries, only the entire allocator. Integrated with a buffer pool to reuse
 * large blocks of heap memory without being exposed to heap fragmentation.
 *
 * @author Balazs Racz
 * @date 25 Dec 2019
 */

#ifndef _UTILS_FORWARDALLOCATOR_HXX_
#define _UTILS_FORWARDALLOCATOR_HXX_

#include <cstdint>

#include "os/OS.hxx"
#include "utils/Buffer.hxx"
#include "utils/SimpleQueue.hxx"

class DynamicPool;

/// An arena allocator, which is optimized to not be able to free individual
/// entries, only the entire allocator. Integrated with a buffer pool to reuse
/// large blocks of heap memory without being exposed to heap fragmentation.
class ForwardAllocator
{
public:
    /// Allocated blocks will be this size.
    static constexpr size_t BLOCK_BYTE_SIZE = 1024;

    /// Allocates a block of memory. Thread-safe.
    /// @param size the number of bytes to allocate.
    /// @param align alignment of allocated object in bytes (1 to 8).
    void *allocate(size_t size, size_t align);

    ForwardAllocator();
    ~ForwardAllocator();

#ifdef GTEST
    /// Usable only from unittests. Re-creates the pool to be empty.
    static void TEST_recreate_pool();
#endif

private:
    /// A primitive type that has sufficient alignment to what we support.
    typedef uint64_t primitive_t;
    /// An array type with the BLOCK_BYTE_SIZE size.
    typedef primitive_t AlignedPayload[BLOCK_BYTE_SIZE / sizeof(primitive_t)];
    static_assert(sizeof(AlignedPayload) == BLOCK_BYTE_SIZE,
        "aligned payload size mismatch");
    /// Buffer type that we will be allocating from the pool.
    typedef Buffer<AlignedPayload> BufferType;

    /// @return pointer to the beginning of the memory area in the latest
    /// allocated block, cast to uint8_t*, or nullptr if no block has been
    /// allocated yet.
    uint8_t *head_ptr()
    {
        if (allocatedBlocks_.empty())
        {
            return nullptr;
        }
        return (uint8_t *)allocatedBlocks_.front()->data();
    }

    /// Holds all blocks that we allocated (ever). They will all be freed in
    /// the destructor.
    TypedQueue<BufferType> allocatedBlocks_;
    /// Points into the last allocated block to the next emtpy space.
    size_t offsetInLast_ {BLOCK_BYTE_SIZE + 1};

public:
    /// Total number of bytes allocated.
    size_t allocSize_ {0};
    /// Number of bytes lost due to alignment and end-of-block chunks.
    size_t allocWasted_ {0};

private:
    /// Lock that protects the offset in last variable and the queue of blocks.
    OSMutex lock_;
    /// This buffer pool will have one bucket to allocate 1kb objects each and
    /// kep them on a free list. The freelist is shared between different
    /// allocators.
    static DynamicPool *kbytePool_;
};

#endif // _UTILS_FORWARDALLOCATOR_HXX_

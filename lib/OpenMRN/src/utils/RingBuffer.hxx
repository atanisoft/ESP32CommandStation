/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file RingBuffer.hxx
 * This file implements a standard ring buffer.
 *
 * @author Stuart W. Baker
 * @date 21 October 2013
 */

#ifndef _UTILS_RINGBUFFER_HXX_
#define _UTILS_RINGBUFFER_HXX_

#include <new>
#include "utils/macros.h"

/** Implements a vanilla ring buffer.
 */
template <typename T> class RingBuffer
{
public:
    /** Factory method to create a ring buffer instance.
     * @param size size in items for the ring buffer
     * @return the newly cleated RingBuffer object.
     */
    static inline RingBuffer *create(size_t size)
    {
        RingBuffer *ring_buffer =
            (RingBuffer*)malloc(sizeof(RingBuffer) + (size * sizeof(T)));
        /* placement new allows for runtime ring buffer size */
        new (ring_buffer) RingBuffer(size);
        
        return ring_buffer;
    }

    /** Destroy an existing ring buffer instance.
     */
    void destroy()
    {
        free(this);
    }

    /** Insert a number of items to the buffer.
     * @param buf reference to the first item to insert
     * @param items total number of items to insert
     * @return total number of items inserted
     */
    size_t put(const T *buf, size_t items)
    {
        /** @todo (Stuart Baker) significant optimization opportunity */
        size_t inserted = items < (_size - count) ? items : (_size - count);
        
        for (size_t i = 0; i < inserted; ++i)
        {
            data[writeIndex++] = buf[i];
            
            if (writeIndex == _size)
            {
                writeIndex = 0;
            }
        }
        
        count += inserted;
        return inserted;
    }
    
    /** remove a number of items from the buffer.
     * @param buf reference to the data removed
     * @param items total number of items to remove
     * @return total number of items removed
     */
    size_t get(T *buf, size_t items)
    {
        /** @todo (Stuart Baker) significant optimization opportunity */
        size_t removed = items < count ? items : count;
        
        for (size_t i = 0; i < removed; ++i)
        {
            buf[i] = data[readIndex++];
            
            if (readIndex == _size)
            {
                readIndex = 0;
            }
        }
        
        count -= removed;
        return removed;
    }
    
    /** Number of items in the buffer.
     * @return number of items in the buffer
     */
    size_t items()
    {
        return count;
    }
    
    /** Size of buffer in number of items.
     * @return size of buffer in number of items
     */
    size_t size()
    {
        return _size;
    }
    
    /** space left in buffer of buffer in number items.
     * @return space left in buffer in number of items
     */
    size_t space()
    {
        return _size - count;
    }
    
private:
    /** Constructor.
     * @param size size in bytes for the ring buffer
     */
    RingBuffer(size_t size)
        : _size(size),
          count(0),
          readIndex(0),
          writeIndex(0)
    {
    }

    /** Default Constructor.
     */
    RingBuffer();
    
    /** Default destructor.
     */
    ~RingBuffer();

    DISALLOW_COPY_AND_ASSIGN(RingBuffer);
    
    /** size in items of ring buffer */
    size_t _size;
    
    /** total number of items in ring buffer */
    size_t count;
    
    /** read index */
    size_t readIndex;
    
    /** write index */
    size_t writeIndex;
    
    /** ring buffer data */
    T data[];
};

#endif /* _UTILS_RINGBUFFER_HXX_ */

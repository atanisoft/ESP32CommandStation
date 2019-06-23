/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file DeviceBuffer.hxx
 * This file provides a buffer class that is useful in the construction of
 * FreeRTOS device drivers.
 *
 * @author Stuart W. Baker
 * @date 2 March 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_DEVICE_BUFFER_HXX_
#define _FREERTOS_DRIVERS_COMMON_DEVICE_BUFFER_HXX_

#include <new>
#include <cstdint>
#include <unistd.h>
#include <stdlib.h>
#include "utils/macros.h"

#ifndef ARDUINO
#include "Devtab.hxx"
#endif

/** Helper for DeviceBuffer which allows for methods to not be inlined.
 */
class DeviceBufferBase
{
public:
#ifndef ARDUINO
    /** Wait for blocking condition to become true.
     * @param file file to wait on
     * @param read true if this is a read operation, false for write operation
     */
    static void block_until_condition(File *file, bool read);
#endif

    /** Signal the wakeup condition.  This will also wakeup select.
     */
    void signal_condition()
    {
#ifndef ARDUINO
        Device::select_wakeup(&selectInfo);
#endif        
    }

    /** Signal the wakeup condition from an ISR context.  This will also
     * wakeup select.
     */
    void signal_condition_from_isr()
    {
#ifndef ARDUINO
        int woken = 0;
        Device::select_wakeup_from_isr(&selectInfo, &woken);
#endif
    }

    /** flush all the data out of the buffer and reset the buffer.  It is
     * assumed that an interrupt cannot occur which would access the buffer
     * asynchronous to the execution of this method and any thread level
     * mutual exclusion is handled by the caller.
     */
    void flush()
    {
        count = 0;
        readIndex = 0;
        writeIndex = 0;
    }

    /** Return the number of items in the queue.
     * @return number of bytes in the queue
     */
    size_t pending()
    {
        return count;
    }

    /** Return the number of items for which space is available.
     * @return number of items for which space is available
     */
    size_t space()
    {
        return size - count;
    }

    /** Add client to list of clients needing woken.
     */
    void select_insert()
    {
#ifndef ARDUINO
        return Device::select_insert(&selectInfo);
#endif        
    }

    /** Remove a number of items from the buffer by advancing the readIndex.
     * @param items total number of items to remove
     * @return total number of items removed
     */
    size_t consume(size_t items)
    {
        if (items > count)
        {
            items = count;
        }
        size_t consumed = items;
        count -= items;
        if ((readIndex + items) >= size)
        {
            items -= (size - readIndex);
            readIndex = 0;
        }
        readIndex += items;

        return consumed;
    }

    /** Add a number of items to the buffer by advancing the writeIndex.
     * @param items total number of items to add
     * @return total number of items added
     */
    size_t advance(size_t items)
    {
        if (items > space())
        {
            items = space();
        }
        size_t added = items;
        count += items;
        if ((writeIndex + items) >= size)
        {
            items -= (size - writeIndex);
            writeIndex = 0;
        }
        writeIndex += items;

        return added;
    }

protected:
    /** Constructor.
     * @param size size in items for the buffer.
     * @param level minimum amount of space required in buffer to restart
     *        transmitting, unused for receive.
     */
    DeviceBufferBase(size_t size, size_t level)
        : level(level)
        , size(size)
        , count(0)
        , readIndex(0)
        , writeIndex(0)
    {
    }

    /** Destructor.
     */
    ~DeviceBufferBase()
    {
    }

#ifndef ARDUINO
    /** Metadata for select() logic */
    Device::SelectInfo selectInfo;
#endif
    
    /** level of space required in buffer in order to wakeup, 0 if unused */
    uint16_t level;
    
    /** size in items of buffer */
    uint16_t size;
    
    /** total number of items in buffer */
    uint16_t count;
    
    /** read index */
    uint16_t readIndex;
    
    /** write index */
    uint16_t writeIndex;

private:
    DISALLOW_COPY_AND_ASSIGN(DeviceBufferBase);
};

/** Implements a smart buffer specifically designed for character
 * device drivers.  Technically, the private metadata for the size and index
 * counters should more properly be implemented as size_t types.  The choice
 * of uint16_t types is simply to save memory as 2^16 is normally a reasonable
 * maximum size for this type of metadata.
 */
template <typename T> class DeviceBuffer : public DeviceBufferBase
{
public:
    /** Create a DeviceBuffer instance.
     * @param size size in items for the DeviceBuffer
     * @param level minimum amount of space required in buffer to restart
     *        transmitting, unused for receive.
     * @return newly created DeviceBuffer instance
     */
    static DeviceBuffer *create(size_t size, size_t level = 0)
    {
        HASSERT(size <= UINT16_MAX);
        HASSERT(level <= size);
        DeviceBuffer *device_buffer =
            (DeviceBuffer*)malloc(sizeof(DeviceBuffer) + (size * sizeof(T)));
        /* placement new allows for runtime ring buffer size */
        new (device_buffer) DeviceBuffer(size, level);
        
        return device_buffer;
    }

    typedef T member_type;
    
    /** @return the size of each member in bytes. */
    static constexpr unsigned member_size()
    {
        return sizeof(T);
    }

    
    
    /** Destroy an existing DeviceBuffer instance.
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
        size_t last_count = count;
        
        while (items && (count < size))
        {
            data[writeIndex++] = *buf++;
            if (writeIndex == size)
            {
                writeIndex = 0;
            }
            ++count;
            --items;
        }
        
        return count - last_count;
    }

    /** remove a number of items from the buffer.
     * @param buf reference to the data removed
     * @param items total number of items to remove
     * @return total number of items removed
     */
    size_t get(T *buf, size_t items)
    {
        size_t last_count = count;
        
        while (items && (count > 0))
        {
            *buf++ = data[readIndex++];            
            if (readIndex == size)
            {
                readIndex = 0;
            }
            --count;
            --items;
        }
        
        return last_count - count;
    }

    /** Get a reference to the current location in the buffer for read.
     * @param buf location to store resulting reference
     * @return number of items in continuous memory.  May be less than total
     *         number of items in the buffer.
     */
    size_t data_read_pointer(T **buf)
    {
        size_t result = size - readIndex;
        if (count < result)
        {
            result = count;
        }
        *buf = data + readIndex;
        return result;
    }

    /** Get a reference to the current location in the buffer for write.
     * @param buf location to store resulting reference
     * @return amount of space in continuous memory.  May be less than total
     *         amount of space avaiable.
     */
    size_t data_write_pointer(T **buf)
    {
        size_t result = size - writeIndex;
        if (space() < result)
        {
            result = space();
        }
        *buf = data + writeIndex;
        return result;
    }

private:
    /** Constructor.
     * @param size size in items for the buffer.
     * @param level minimum amount of space required in buffer to restart
     *        transmitting, unused for receive.
     */
    DeviceBuffer(size_t size, size_t level)
        : DeviceBufferBase(size, level)
    {
    }

    /** Destructor.
     */
    ~DeviceBuffer()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(DeviceBuffer);

    /** buffer data */
    T data[];
};

#endif /* _FREERTOS_DRIVERS_COMMON_DEVICE_BUFFER_HXX_ */

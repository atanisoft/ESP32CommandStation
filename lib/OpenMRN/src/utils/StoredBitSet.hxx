/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file StoredBitSet.hxx
 *
 * Interface for persistent bit set.
 *
 * @author Balazs Racz
 * @date 5 June 2017
 */

#ifndef _UTILS_STOREDBITSET_HXX_
#define _UTILS_STOREDBITSET_HXX_

#include <stdint.h>
#include <string.h>

#include "utils/macros.h"
#include "utils/Atomic.hxx"

/** Abstract class for representing a set of numbered bits that are stored
 * persistently in some backing store.
 */
class StoredBitSet
{
public:
    /// Sets an individual bit to a specific value. This call is thread-safe.
    /// @param offset is the bit number to set.
    /// @param value is the new value for that bit.
    /// @return *this.
    virtual StoredBitSet &set_bit(unsigned offset, bool value) = 0;

    /// Retrieves an individual bit.
    /// @param offset is the bit number to retrieve.
    /// @return the last set bit value.
    virtual bool get_bit(unsigned offset) = 0;

    /// Sets a block of consecutive bits. This call is thread-safe.
    /// @param offset is the number of the first bit to set.
    /// @param size is the number of bits to set.
    /// @param value contains the data to write, LSB (bit 0) goes to "offset",
    /// bit 1 goes to offset + 1, etc.; bits at and above size are checked to be
    /// zero.
    /// @return *this.
    virtual StoredBitSet &set_multi(
        unsigned offset, unsigned size, unsigned value) = 0;

    /// Returns a block of consecutive bits as an integer value.
    /// @param offset is the number of the first bit to retrieve.
    /// @param size is the number of bits to retrieve.
    /// @return the unsigned value that the last set bits represent, with
    /// LSB-first (bit 0 will be from offset; bit 1 will be from offset + 1,
    /// etc.)
    virtual unsigned get_multi(unsigned offset, unsigned size) = 0;

    /// @return the number of bits in the storage. Valid offsets are
    /// 0..size()-1.
    virtual unsigned size() = 0;

    /// Writes the current values to persistent storage. The caller is
    /// responsible for locking.
    virtual void flush() = 0;

    /// Grabs a lock and writes the current values to persistent storage.
    virtual void lock_and_flush() = 0;
};

class ShadowedStoredBitSet : public StoredBitSet, protected Atomic
{
public:
    StoredBitSet &set_bit(unsigned offset, bool value) override {
        HASSERT(offset < size_);
        auto* shadow_ptr = shadow_ + (offset >> 5);
        auto bit = (UINT32_C(1) << (offset & 31));
        if (value) {
            if (__atomic_fetch_or(shadow_ptr, bit, __ATOMIC_SEQ_CST) & bit) {
                return *this; // nothing to do
            }
        } else {
            if ((__atomic_fetch_and(shadow_ptr, ~bit, __ATOMIC_SEQ_CST) & bit) == 0) {
                return *this; // nothing to do
            }
        }
        cell_offs_t d = offset / granularity_;
        __atomic_fetch_or(dirty_ + (d >> 5), UINT32_C(1) << (d & 31), __ATOMIC_SEQ_CST);
        update_dirty_bounds(d);
        return *this;
    }

    bool get_bit(unsigned offset) override {
        HASSERT(offset < size_);
        return shadow_[offset >> 5] & (UINT32_C(1) << (offset & 31));
    }

    StoredBitSet &set_multi(
        unsigned offset, unsigned size, unsigned value) override {
        HASSERT(size <= 32);
        HASSERT(size > 0);
        HASSERT(offset + size <= size_);
        if (size < 32) {
            HASSERT((value >> size) == 0);
        }
        unsigned sidx = offset >> 5;
        uint32_t smask = UINT32_C(0xFFFFFFFF);
        smask >>= (32 - size); // gets `size' count of one bits
        smask <<= (offset & 31); // moves them to the right place
        uint32_t sval = value;
        sval <<= (offset & 31);
        int rollover = size + (offset & 31) - 32;
        if (rollover > 0) {  // there are leftover bits for a second word
            uint32_t smask2 = (UINT32_C(1) << rollover) - 1;
            uint32_t sval2 = value >> (size - rollover);
            // we need to atomically update two cells.
            uint32_t old1 = shadow_[sidx], new1, old2 = shadow_[sidx + 1], new2;
            do
            {
                new1 = (old1 & ~smask) | sval;
                new2 = (old2 & ~smask2) | sval2;
            } while (!__atomic_compare_exchange_n(shadow_ + sidx, &old1, new1,
                         false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST) ||
                !__atomic_compare_exchange_n(shadow_ + sidx + 1, &old2, new2,
                    false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));
        }
        else
        {
            // just one cell
            uint32_t old1 = shadow_[sidx], new1;
            do
            {
                new1 = (old1 & ~smask) | sval;
            } while (!__atomic_compare_exchange_n(shadow_ + sidx, &old1, new1,
                         false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));
        }
        // Sets dirty bits.
        unsigned d = offset / granularity_;
        update_dirty_bounds(d);
        unsigned de = (offset + size - 1)/granularity_;
        update_dirty_bounds(de);
        if (de > highestDirty_) highestDirty_ = de;
        while (d <= de) {
            __atomic_fetch_or(dirty_ + (d >> 5),(UINT32_C(1)<<(d & 31)), __ATOMIC_SEQ_CST);
            d++;
        }
        return *this;
    }

    unsigned get_multi(unsigned offset, unsigned size) override {
        HASSERT(size <= 32);
        HASSERT(size > 0);
        HASSERT(offset + size <= size_);
        unsigned val = shadow_[offset >> 5] >> (offset & 31);
        int rollover = size + (offset & 31) - 32;
        if (rollover > 0) {
            val |= shadow_[(offset >> 5) + 1] << (size - rollover);
        }
        if (size < 32) {
            val &= (UINT32_C(1) << size) - 1;
        }
        return val;
    }
    
    unsigned size() override {
        return size_;
    }

    void lock_and_flush() override {
        AtomicHolder h(this);
        flush();
    }
    
protected:
    /// @param size is the total number of bits we store.
    /// @param granularity tells how many bits fit into a single cell. The base
    /// class will then keep track of dirty cells.
    ShadowedStoredBitSet(unsigned size, uint8_t granularity)
        : size_(size), granularity_(granularity) {
        unsigned ssize = (size + 31) >> 5;
        shadow_ = new uint32_t[ssize];
        memset(shadow_, 0, ssize * 4);
        unsigned dsize = (num_cells() + 31) >> 5;
        dirty_ = new uint32_t[dsize];
        memset(dirty_, 0, dsize * 4);
        HASSERT(num_cells() < NO_CELL);
    }

    ~ShadowedStoredBitSet() {
        delete[] shadow_;
        delete[] dirty_;
    }

    /// Type indexing cells. Must be an unsigned type.
    typedef uint8_t cell_offs_t;
    /// Type indexing all bits.
    typedef unsigned bit_offs_t;

    static constexpr cell_offs_t NO_CELL = static_cast<cell_offs_t>(-1);

    /// @return the cell number of the next dirty cell, or NO_CELL if nothing
    /// is dirty anymore.
    cell_offs_t next_dirty() {
        if (lowestDirty_ > highestDirty_ || lowestDirty_ >= num_cells()) {
            lowestDirty_ = NO_CELL;
            highestDirty_ = 0;
            return NO_CELL;
        }
        unsigned dirty_idx = lowestDirty_ / 32;
        bool fresh_dirty = false;
        while (dirty_idx < dirty_size_uint32() && dirty_[dirty_idx] == 0) {
            ++dirty_idx;
            fresh_dirty = true;
        }
        if (dirty_idx >= dirty_size_uint32()) {
            lowestDirty_ = NO_CELL;
            highestDirty_ = 0;
            return NO_CELL;
        }
        unsigned bit_ofs = 0;
        if (!fresh_dirty) {
            bit_ofs = lowestDirty_ & 31;
        } else {
        }
        while (((UINT32_C(1)<<bit_ofs) & dirty_[dirty_idx]) == 0) {
            ++bit_ofs;
        }
        lowestDirty_ = (dirty_idx << 5) + bit_ofs;
        return lowestDirty_;
    }

    /// Clears the dirty bit for a given cell.
    /// @param cell is the number of the cell, 0..num_cells()-1.
    void clear_dirty(cell_offs_t cell) {
        HASSERT(cell < num_cells());
        dirty_[cell >> 5] &= ~(UINT32_C(1)<<(cell & 31));
        if (cell == lowestDirty_) {
            ++lowestDirty_;
        }
    }

    /// @return true if the given cell is dirty.
    /// @param cell is the number of the cell, 0..num_cells()-1.
    bool is_dirty(cell_offs_t cell) {
        HASSERT(cell < num_cells());
        return dirty_[cell >>5] & (UINT32_C(1)<<(cell & 31));
    }
    
    /// @return how many cells we have, each with granularity_ bits stored.
    cell_offs_t num_cells() {
        return (size_ + granularity_ - 1) / granularity_;
    }
    
private:
    /// @return how many uint32 are there in the dirty_ array.
    unsigned dirty_size_uint32() {
        return (num_cells() + 31) / 32;
    }

    /// updates lowestDirty_ and highestDirty_ for a given cell.
    /// @param d cell offset to set dirty
    void update_dirty_bounds(cell_offs_t d)
    {
        cell_offs_t old_val = lowestDirty_;
        do
        {
            if (d >= old_val)
                break;
        } while (!__atomic_compare_exchange_n(&lowestDirty_, &old_val, d, false,
                     __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));
        old_val = highestDirty_;
        do
        {
            if (d <= old_val)
                break;
        } while (!__atomic_compare_exchange_n(&highestDirty_, &old_val, d,
                     false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));
    }

    /// Total number of bits.
    const unsigned size_;
    /// How many bits per cell.
    const uint8_t granularity_;
    /// Helper values for iterating over the dirty bits.
    cell_offs_t lowestDirty_{NO_CELL};
    cell_offs_t highestDirty_{0};
    
    /// Data of the actual bits. length is size_ / 32 rounded up.
    uint32_t *shadow_;
    /// Bit set telling whether the individual cells are dirty or not
    /// (i.e. need to be flushed). LSB-first in the individual bits, i.e. bit 1
    /// of entry 2 means that the cell numbered (2*32 + 1) is dirty.
    uint32_t *dirty_;
};

#endif // _UTILS_STOREDBITSET_HXX_

/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file ScheduledQueue.hxx
 *
 * A Queue implementation that has a priority based scheduler on the different
 * bands.
 *
 * @author Balazs Racz
 * @date 30 Dec 2020
 */

#ifndef _UTILS_SCHEDULEDQUEUE_HXX_
#define _UTILS_SCHEDULEDQUEUE_HXX_

#include "os/OS.hxx"
#include "utils/Fixed16.hxx"
#include "utils/Queue.hxx"
#include "utils/logging.h"

/// ScheduledQueue is a queue with multiple priorities, where each priority is
/// a FIFO list. The different priorities are polled according to a weighted
/// stride scheduler instead of in strict numerical priority order.
class ScheduledQueue
{
public:
    /// Constructor.
    /// @param num_bands now many priority bands should there be.
    /// @param strides is an array of the size num_bands. It contains the
    /// stride coefficients (they should all be numbers <= 1. A value of 1
    /// degrades to strict priority order). This array is not used anymore
    /// after the constructor returns. A stride of 0.2 means that 20% of all
    /// tokens at this level will be allocated to this band, and 80% of the
    /// tokens are passed down to lower priorities (bands with higher index).
    ScheduledQueue(unsigned num_bands, const Fixed16 *strides)
        : numBands_(num_bands)
        , bands_(new Band[num_bands])
    {
        for (unsigned i = 0; i < numBands_; i++)
        {
            bands_[i].stride_ = strides[i];
            bands_[i].currentToken_ -= strides[i];
        }
    }

    /// Destructor.
    ~ScheduledQueue()
    {
        delete[] bands_;
    }

    /// Get an item from the queue. The returned item will be according to the
    /// priority scheduler.
    /// @return the member and the priority from which it came.
    Result next()
    {
        OSMutexLock h(lock());
        return next_locked();
    }

    /// Get an item from the queue. The returned item will be according to the
    /// priority scheduler. The caller must acquire the lock() first.
    /// @return the member and the priority from which it came.
    Result next_locked()
    {
        if (!numPending_)
        {
            // Empty queue.
            return Result(0, 0);
        }
        // Execute the priority based scheduling algorithm.
        for (unsigned i = 0; i < numBands_; ++i)
        {
            bands_[i].currentToken_ += bands_[i].stride_;
            if (bands_[i].currentToken_.trunc() >= 1)
            {
                Result ret = bands_[i].queue_.next_locked();
                if (ret.item)
                {
                    ret.index = i;
                    --numPending_;
                    bands_[i].currentToken_ -= 1;
                    return ret;
                }
                else
                {
                    // This queue has a token but is empty. We remove
                    // fractional tokens and keep searching onwards in the
                    // priorities.
                    bands_[i].currentToken_ = 1;
                }
            }
        }
        // Fallen off at the end. We go backwards to find any queue with
        // nonempty members.
        for (int i = numBands_ - 1; i >= 0; --i)
        {
            if (!bands_[i].queue_.empty())
            {
                Result ret = bands_[i].queue_.next_locked();
                bands_[i].currentToken_ = 0;
                ret.index = i;
                --numPending_;
                return ret;
            }
        }
        DIE("Unexpected nonempty queue");
        return Result(0, 0);
    }

    /// The caller must acquire this lock before using any of the _locked()
    /// functions. If the caller needs to do many operations in quick
    /// succession, it might be faster to do them under a single lock,
    /// i.e. acquire lock() first, then call xxx_locked() repeatedly, then
    /// unlock.
    /// @return the lock to use for the _locked() functions.
    OSMutex *lock()
    {
        return &lock_;
    }

    /// Adds an entry to the queue. It will be added to the end of the given
    /// priority band.
    /// @param item the entry to be added to the queue.
    /// @param prio which priority band to add to. 0 = highest priority. Must
    /// be within 0 and numBands_ - 1.
    void insert(QMember *item, unsigned prio)
    {
        OSMutexLock h(lock());
        return insert_locked(item, prio);
    }

    /// Adds an entry to the queue. It will be added to the end of the given
    /// priority band. The caller must hold lock().
    /// @param item the entry to be added to the queue.
    /// @param prio which priority band to add to. 0 = highest priority. Must
    /// be within 0 and numBands_ - 1.
    void insert_locked(QMember *item, unsigned prio)
    {
        HASSERT(prio < numBands_);
        ++numPending_;
        bands_[prio].queue_.insert_locked(item);
    }

    /// Get the number of pending items in the queue.
    /// @param prio in the list to operate on
    /// @return number of pending items in that priority band in the queue
    size_t pending(unsigned prio)
    {
        HASSERT(prio < numBands_);
        return bands_[prio].queue_.pending();
    };

    /// Get the number of pending items in the queue (all bands total)
    /// @return number of pending items
    size_t pending() const
    {
        return numPending_;
    }

    /// @return true if the queue is empty (on all priority bands).
    bool empty() const
    {
        return numPending_ == 0;
    }

    /// @return the number of available priority bands.
    unsigned num_prio() const
    {
        return numBands_;
    }

private:
    /// This structure contains information about one priority band.
    struct Band
    {
        /// Holds the queue for this priority band.
        Q queue_;
        /// How many tokens we add each call.
        Fixed16 stride_ {0};
        /// How many tokens we have right now. If this is > 1 then we will emit
        /// the front of this queue, if it is < 1 then we move on to the next
        /// priority item.
        Fixed16 currentToken_ {1, 0};
    };

    /// Protects insert and next operations.
    OSMutex lock_;

    /// How many priority bands we have.
    unsigned numBands_;

    /// How many queue entries are pending.
    unsigned numPending_ {0};

    /// The actual priority bands.
    Band *bands_;
};

#endif // _UTILS_SCHEDULEDQUEUE_HXX_
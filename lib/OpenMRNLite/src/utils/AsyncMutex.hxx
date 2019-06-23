/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file AsyncMutex.hxx
 *
 * Class for a mutex that can be waited for via the asynchronous Pool API.
 *
 * @author Balazs Racz
 * @date 19 April 2014
 */

#ifndef _UTILS_ASYNCMUTEX_HXX_
#define _UTILS_ASYNCMUTEX_HXX_

#include "utils/Queue.hxx"

/**
 *  Helper class that simulates a (non-reentrant) mutex using the Allocator
 *  queue and a single QueueMember token.
 *
 *  The mutex is defined as unlocked if there is an entry on the allocator
 *  queue. Locking the mutex will take the entry off of the allocator
 *  queue. Any other acquisition attempts will block so long as the allocator's
 *  queue is empty.
 *
 *  Unlocking the mutex will release the token back to the allocator, waking up
 *  the first caller in the queue.
 *
 *  To lock the mutex, use any allocation mechanism (e.g. control
 *  flow::Allocate, or SyncAllocator). To Unlock the mutex, call the Unlock
 *  method.
 */
class AsyncMutex : public QAsync
{
public:
    /// Creates an allocator mutex.
    AsyncMutex()
    {
        Unlock();
    }

    ~AsyncMutex()
    {
    }

    /// Crashes if the the particular value is not the token associated with
    /// this
    /// mutex.
    //
    /// @param token is the value to check.
    void CheckToken(QMember *token)
    {
        HASSERT(token == &token_);
    }

    /// Crashes if the mutex is not locked.
    void AssertLocked()
    {
        HASSERT(empty());
    }

    /// Crashes if the mutex is locked.
    void AssertUnlocked()
    {
        HASSERT(!empty());
    }

    /// Unlocks the mutex. Crashes if the mutex is unlocked.
    void Unlock()
    {
        AssertLocked();
        insert(&token_);
    }

    /** Synchronously locks the mutex. Might block the current thread.
    void Lock()
    {
    @todo(balazs.racz) we don't have a synchronous mechanism to allocate from a QAsync.
        SyncAllocation a(this);
    }
    */

private:
    class Token : public QMember {};
    /// a unique allocation token that can be passed around to signal who owns
    /// the mutex.
    Token token_;
};

#endif // _UTILS_ASYNCMUTEX_HXX_

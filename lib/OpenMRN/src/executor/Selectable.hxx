/** \copyright
 * Copyright (c) 2013-2015, Stuart W Baker and Balazs Racz
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
 * \file Selectable.hxx
 *
 * Base class for delays controlled by a select on an executor.
 *
 * @author Balazs Racz
 * @date 6 Apr 2015
 */

#ifndef _EXECUTOR_SELECTABLE_HXX_
#define _EXECUTOR_SELECTABLE_HXX_

/// Handler structure that ExecutorBase knows about each entry to the select
/// call. See @ref ExecutorBase::select().
class Selectable : public QMember
{
public:
    /// Which operation this file should be selected upon. (i.e. which fd_set
    /// the given fd is coming in).
    enum SelectType
    {
        READ = 1,
        WRITE = 2,
        EXCEPT = 3,
    };

    /// Helper declarations for the certain fields' maximums.
    enum Limits
    {
        /// Largest FD we accept (otherwise crash with error).
        MAX_FD = (1 << 14) - 1,
        /// Largest priority we accept (otherwise we clip).
        MAX_PRIO = (1 << 16) - 1,
    };

    /// Constructor. @param parent is the executable that will be woken up when
    /// this selectable is triggered.
    Selectable(Executable *parent) : selectType_(0), wakeup_(parent)
    {
    }

    /// Re-initialize a Selectable preparing to add it to select().
    ///
    /// @param type whether we waiting for READ, WRITE or EXCEPT.
    /// @param fd file descriptor to wait for.
    /// @param priority what priority should the callback be executed at once
    /// the file triggers.
    void reset(SelectType type, int fd, unsigned priority)
    {
        selectType_ = type;
        HASSERT(fd <= MAX_FD);
        fd_ = fd;
        priority_ = std::min((unsigned)priority, (unsigned)MAX_PRIO);
    }

    /// @return true if this selectable was never used before.
    bool is_empty() {
        return selectType_ == 0;
    }

    /// @return priority upon which the executable should be enqueued upon
    /// trigger.
    unsigned priority()
    {
        return priority_;
    }

    /// @return what we should select this upon, READ, WRITE or EXCEPT.
    SelectType type()
    {
        return static_cast<SelectType>(selectType_);
    }

    /// @return Executable to enqueue upon successful triggering.
    Executable *parent()
    {
        return wakeup_;
    }

    /// @return Filedes to wait for.
    int fd()
    {
        return fd_;
    }

    /** Can be used to override the executable to wake up. Make sure to set it
     * back afterwards. @param e is the new executable to trigger. */
    void set_wakeup(Executable *e)
    {
        wakeup_ = e;
    }

private:
    friend class ExecutorBase;

    /// What to watch the file for. See @ref SelectType
    unsigned selectType_ : 2;
    /// File descriptor to watch.
    unsigned fd_ : 14;
    /// When the select condition is met, the Executable will be scheduled at
    /// this priority.
    unsigned priority_ : 16;
    /// This executable will be scheduled on the executor when the select
    /// condition is met.
    Executable *wakeup_;
};

#endif // _EXECUTOR_SELECTABLE_HXX_

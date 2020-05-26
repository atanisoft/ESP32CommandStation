/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file AutoSyncFileFlow.hxx
 *
 * Automatic sync to disk for a file handle at regulard intervals
 *
 * @author Mike Dunston
 * @date 3 Aug 2019
 */

#ifndef _UTILS_AUTOSYNCFILEFLOW_H_
#define _UTILS_AUTOSYNCFILEFLOW_H_

#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "utils/StringPrintf.hxx"

/// Simple state flow to configure automatic calls to fsync on a single file
/// handle at regular intervals.
class AutoSyncFileFlow : public StateFlowBase
{
public:
    /// Constructor
    ///
    /// @param service is the @ref Service to hook into for periodic callbacks.
    /// @param sync_fd is the file handle to sync.
    /// @param interval is the interval at which to sync the file handle. Default
    /// is once per second.
    AutoSyncFileFlow(Service *service
                   , int sync_fd
                   , uint64_t interval=SEC_TO_NSEC(1))
                   : StateFlowBase(service)
                   , fd_(sync_fd)
                   , interval_(interval)
                   , name_(StringPrintf("AutoSyncFileFlow(%d)", fd_))
    {
      HASSERT(fd_ >= 0);
      HASSERT(interval_ > 0);
      start_flow(STATE(sleep_and_call_sync));
    }

    /// Requests the discontinuation of automatic calls to fsync.
    ///
    /// @param n is the @ref Notifiable to call when this flow exits.
    void shutdown(Notifiable *n)
    {
        shutdown_ = true;
        std::swap(shutdownDone_, n);
        if (n)
        {
            n->notify();
        }
        timer_.ensure_triggered();
    }

private:
    const int fd_;
    const uint64_t interval_;
    const std::string name_;
    StateFlowTimer timer_{this};
    bool shutdown_{false};
    Notifiable *shutdownDone_{nullptr};

    Action sleep_and_call_sync()
    {
        return sleep_and_call(&timer_, interval_, STATE(sync));
    }

    Action sync()
    {
        if (shutdown_)
        {
            AutoNotify n(shutdownDone_);
            return exit();
        }
        ERRNOCHECK(name_.c_str(), fsync(fd_));
        return call_immediately(STATE(sleep_and_call_sync));
    }
};

#endif // _UTILS_AUTOSYNCFILEFLOW_H_

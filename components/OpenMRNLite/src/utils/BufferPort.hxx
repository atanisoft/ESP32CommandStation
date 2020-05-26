/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file BufferPort.hxx
 *
 * Wrapper for a string-valued Hub port. Uses a time delay to buffer string
 * output up to a certain size before sending off to a target port.
 *
 * @author Balazs Racz
 * @date 20 Jun 2016
 */

#ifndef _UTILS_BUFFERPORT_HXX_
#define _UTILS_BUFFERPORT_HXX_

#include "utils/LimitedPool.hxx"

/// A wrapper class around a string-based Hub Port that buffers the outgoing
/// bytes for a specified delay timer before sending the data off. This helps
/// accumulate more data per TCP packet and increase transmission efficiency.
///
// Added by default on GridConnect bridges.
class BufferPort : public HubPort
{
public:
    /// Constructor.
    ///
    /// @param service specifies which thread to operate on. Typically the same
    /// as the calling Hub's executor.
    /// @param downstream where to send the (buffered) data onwards.
    /// @param buffer_bytes how many bytes to buffer up max.
    /// @param delay_nsec how many nanoseconds long we should buffer the output
    /// data max.
    BufferPort(Service *service, HubPortInterface *downstream,
        unsigned buffer_bytes, long long delay_nsec)
        : HubPort(service)
        , downstream_(downstream)
        , delayNsec_(delay_nsec)
        , sendBuf_(new char[buffer_bytes])
        , bufSize_(buffer_bytes)
        , bufEnd_(0)
        , timerPending_(0)
    {
        HASSERT(sendBuf_);
    }

    ~BufferPort()
    {
        delete [] sendBuf_;
    }

    bool shutdown() {
        flush_buffer();
        if (timerPending_) {
            return false;
        }
        if (!is_waiting()) {
            return false;
        }
        return true;
    }

private:
    Action entry() override
    {
        if (!tgtBuf_)
        {
            return allocate_and_call(downstream_, STATE(buf_alloc_done),
                config_gridconnect_bridge_max_outgoing_packets() <= 1
                    ? nullptr
                    : &outputPool_);
        }
        if (msg().size() < (bufSize_ - bufEnd_))
        {
            // Fits into the buffer.
            memcpy(sendBuf_ + bufEnd_, msg().data(), msg().size());
            bufEnd_ += msg().size();
            if (!timerPending_)
            {
                timerPending_ = 1;
                bufferTimer_.start(delayNsec_);
            }
            return release_and_exit();
        }
        else
        {
            flush_buffer();
        }

        if (msg().size() >= bufSize_)
        {
            // Cannot buffer: send off directly.
            downstream_->send(transfer_message(), priority());
            return exit();
        }
        else
        {
            // After flushing the buffers this will fit.
            return again();
        }
    }

    /// State when the allocation of output buffer completed.
    Action buf_alloc_done()
    {
        tgtBuf_ = get_allocation_result(downstream_);
        tgtBuf_->data()->skipMember_ = message()->data()->skipMember_;
        return call_immediately(STATE(entry));
    }

    /// Sends off any data we may have accumulated in the buffer to the
    /// downstream consumer.
    void flush_buffer()
    {
        if (!bufEnd_) return; // nothing to do
        auto *b = tgtBuf_;
        tgtBuf_ = nullptr;
        b->data()->assign(sendBuf_, bufEnd_);
        bufEnd_ = 0;
        if (message())
        {
            b->set_done(message()->new_child());
        }
        downstream_->send(b);
    }

    /// Callback from the timer.
    void timeout()
    {
        timerPending_ = 0;
        flush_buffer();
    }

    /// @return the current message that we are processing.
    const string &msg()
    {
        return *message()->data();
    }

    /// Timer that triggers the parent flow when expiring. Used to flush the
    /// accumulated gridconnect bytes to the downstream HUB.
    class BufferTimer : public ::Timer
    {
    public:
        /// Constructor. @param parent what to call when expiring.
        BufferTimer(BufferPort *parent)
            : Timer(parent->service()->executor()->active_timers())
            , parent_(parent)
        {
        }

        long long timeout() override
        {
            parent_->timeout();
            return NONE;
        }

    private:
        BufferPort *parent_; ///< what to notify upon timeout.
    } bufferTimer_{this}; ///< timer instance.

    /// Pool implementation that limits the number of buffers allocatable to
    /// the configuration option.
    LimitedPool outputPool_ {sizeof(*tgtBuf_),
        (unsigned)config_gridconnect_bridge_max_outgoing_packets()};
    /// Caches one output buffer to fill in the buffer flush method.
    Buffer<HubData> *tgtBuf_{nullptr};
    /// Where to send output data to.
    HubPortInterface* downstream_;
    /// How long maximum we should buffer the input data.
    long long delayNsec_;
    /// Temporarily stores outgoing data.
    char *sendBuf_;
    /// How many bytes are there in the send buffer.
    unsigned bufSize_;
    /// Offset in sendBuf_ of the first unused byte.
    unsigned bufEnd_ : 24;
    /// 1 if the timer is running and there will be a timer callback coming in
    /// the future.
    unsigned timerPending_ : 1;
};

#endif // _UTILS_BUFFERPORT_HXX_

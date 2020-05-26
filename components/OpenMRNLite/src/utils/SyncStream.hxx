/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file SyncStream.hxx
 * Utility classes for processing a stream of bytes
 *
 * @author Balazs Racz
 * @date 17 May 2017
 */

#ifndef _UTILS_SYNCSTREAM_HXX_
#define _UTILS_SYNCSTREAM_HXX_

#include <memory>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "utils/macros.h"
#include "utils/logging.h"

class SyncStream
{
public:
    virtual ~SyncStream()
    {
    }

    /** Main entry point to the data consumption.
     *
     * @param data is the pointer to a block of data to consume.
     * @param len is the number of bytes to consume.
     * @return 0 if the stream is completed/EOF (not consuming data anymore);
     * negative value if there is an error; or the number of bytes consumed
     * from the stream. */
    virtual ssize_t write(const void *data, size_t len) = 0;

    /** Called once after all data has been written to close the stream and
     * release resources. Return 0 on success, <0 on failure.
     *
     * @param status is an error code seen by wrapping streams. Default 0 (OK),
     * if negative, streams might want to roll back their changes. */
    virtual int finalize(int status)
    {
        return 0;
    }

    /** Repeatedly writes until all data has been consumed or an error
     * occurs. Returns a short write only when an EOF occured. */
    ssize_t write_all(const void *data, size_t len)
    {
        auto *d = to_8(data);
        size_t written = 0;
        while (len > 0)
        {
            auto ret = write(d, len);
            if (ret < 0)
            {
                return ret;
            }
            if (ret == 0)
            {
                return written;
            }
            written += ret;
            d += ret;
            len -= ret;
        }
        return written;
    }

protected:
    /// Converts a void pointer to an equivalent byte pointer.
    static const uint8_t *to_8(const void *d)
    {
        return static_cast<const uint8_t *>(d);
    }

    /// Converts a void pointer to an equivalent byte pointer.
    static uint8_t *to_8(void *d)
    {
        return static_cast<uint8_t *>(d);
    }
};

/**
 * Stream implementation that takes a fixed number of bytes, filling in a
 * header structure, and then returns EOF.
 */
class HeaderStream : public SyncStream
{
public:
    HeaderStream(void *header, size_t header_size)
        : data_(to_8(header))
        , remaining_(header_size)
    {
    }

    ssize_t write(const void *data, size_t len) override
    {
        if (remaining_ == 0)
        {
            return 0;
        }
        if (len > remaining_)
        {
            len = remaining_;
        }
        memcpy(data_, data, len);
        remaining_ -= len;
        data_ += len;
        return len;
    }

private:
    /// Pointer where we need to save the incoming bytes.
    uint8_t *data_;
    /// How many bytes we still need to save.
    size_t remaining_;
};

/** Helper class for defining streams that forward data to another stream
 * internally. */
class WrappedStream : public SyncStream
{
public:
    WrappedStream(SyncStream *delegate)
        : delegate_(delegate)
    {
    }

    /// Overrides the target where to send the incoming data onwards. Frees
    /// (and finalizes) the previous delegate.
    /// @param delegate is the wrapped stream. Takes ownership of the pointer.
    void set_delegate(SyncStream *delegate)
    {
        if (delegate_)
        {
            // TODO: discards error value.
            delegate_->finalize(-1);
        }
        delegate_.reset(delegate);
    }

    int finalize(int status) override
    {
        int ret = 0;
        if (delegate_)
        {
            ret = delegate_->finalize(status);
            delegate_.reset();
        }
        return ret;
    }

protected:
    /// Where to write the data to.
    std::unique_ptr<SyncStream> delegate_;
};

/** Stream wrapper that limits the number of bytes sent to the child stream,
 * and reports EOF after the given length. */
class MaxLengthStream : public WrappedStream
{
public:
    /// @param length is the number of bytes after which to report error.
    /// @param delegate is the wrapped stream. Takes ownership of the pointer.
    MaxLengthStream(size_t length, SyncStream *delegate)
        : WrappedStream(delegate)
        , remaining_(length)
    {
    }

    ~MaxLengthStream() {
        LOG(INFO, "deleting maxlengthstream remaining=%d", (int) remaining_);
    }
    
    ssize_t write(const void *data, size_t len) override
    {
        if (remaining_ == 0)
        {
            return 0;
        }
        if (len > remaining_)
        {
            len = remaining_;
        }
        ssize_t ret = delegate_->write(data, len);
        if (ret <= 0)
        {
            return ret;
        }
        remaining_ -= ret;
        return ret;
    }

private:
    /// How many bytes we still have to write.
    size_t remaining_;
};

/** Stream wrapper that contains a small internal buffer to ensure that all
 * writes are at least a certain minimum size long. The delegate has to
 * guarantee that it will always accept a min_size length write. */
class MinWriteStream : public WrappedStream
{
public:
    MinWriteStream(
        unsigned min_write_length, uint8_t fill_byte, SyncStream *delegate)
        : WrappedStream(delegate)
        , buffer_(nullptr)
        , bufLength_(0)
        , minWriteLength_(min_write_length)
        , fillByte_(fill_byte)
        , needsFill_(1)
    {
    }

    MinWriteStream(
        unsigned min_write_length, SyncStream *delegate)
        : WrappedStream(delegate)
        , buffer_(nullptr)
        , bufLength_(0)
        , minWriteLength_(min_write_length)
        , needsFill_(0)
    {
    }
    
    ~MinWriteStream()
    {
        LOG(INFO, "deleting minwritestream l=%d", (int) minWriteLength_);
        delete[] buffer_;
    }

    ssize_t write(const void *data, size_t len) override
    {
        if (len == 0)
            return 0; // not sure what to do here
        if (bufLength_)
        {
            // There is some data in the buffer. Try to complete and flush it.
            HASSERT(buffer_);
            size_t cp = len;
            if (cp + bufLength_ > minWriteLength_)
            {
                cp = minWriteLength_ - bufLength_;
            }
            memcpy(buffer_ + bufLength_, data, cp);
            bufLength_ += cp;
            if (bufLength_ >= minWriteLength_)
            {
                // Flush
                auto ret = delegate_->write_all(buffer_, bufLength_);
                if (ret <= 0)
                    return ret;
                HASSERT(ret == (int)bufLength_);
                bufLength_ = 0;
            }
            return cp;
        }
        if (len < minWriteLength_)
        {
            // Too small write. Must copy stuff to the buffer.
            if (!buffer_)
            {
                buffer_ = (uint8_t *)new uint32_t[minWriteLength_ / 4];
            }
            memcpy(buffer_, data, len);
            bufLength_ = len;
            return len;
        }
        auto ret = delegate_->write(data, len);
        return ret;
    }

    int finalize(int status) override
    {
        if (bufLength_)
        {
            memset(
                buffer_ + bufLength_, fillByte_, minWriteLength_ - bufLength_);
            auto ret = delegate_->write(buffer_, minWriteLength_);
            if (ret < 0)
                return ret;
            if (ret == 0)
                return -1; // dropped data.
        }
        return delegate_->finalize(status);
    }

private:
    uint8_t *buffer_;
    /// Number of used bytes in the buffer.
    unsigned bufLength_;
    /// Total length of the buffer. All writes to the downstream object will be
    /// at least this long.
    unsigned minWriteLength_;
    /// What byte to append to the stream at finalize time when we still have
    /// bytes to send onwards.
    uint8_t fillByte_;
    /// Whether to do fill (1: yes, 0: no).
    uint8_t needsFill_;
};

/** Simple stream implementation that appends all data to a given
 * std::string. */
class StringAppendStream : public SyncStream {
public:
    /// @param output is the string to write the data to. Does NOT take
    /// ownership of the pointer.
    StringAppendStream(std::string* output)
        : output_(output) {}

    void set_output(std::string* output) {
        output_ = output;
    }
    
    ssize_t write(const void *data, size_t len) override {
        output_->append((const char*)data, len);
        return len;
    }

private:
    std::string* output_;
};

/** Stream implementation that allows running a state machine of different
 * streams, typically alternating a header stream and some payload streams. */
class DelegateStream : public SyncStream
{
public:
    ssize_t write(const void *data, size_t len) override
    {
        if (len == 0)
            return 0;
        while (true)
        {
            if (!delegate_)
            {
                return 0;
            }
            ssize_t ret = delegate_->write(data, len);
            if (ret != 0)
            {
                // Both positive (data actually written) as well as negative
                // (error) will be returned directly.
                return ret;
            }
            on_eof();
        }
    }

protected:
    /// This function will be called when the delegate returns EOF. Usually
    /// used to process something and then set up a new delegate (or clear
    /// delegate which will start returning EOF).
    virtual void on_eof()
    {
        delegate_->finalize(0);
        delegate_.reset();
    }

    /// Stream implementation to delegate the logic to. Must be instantiated by
    /// the implementation of DelegateStream. If null, then the stream will
    /// return EOF to the caller.
    std::unique_ptr<SyncStream> delegate_;
};

#endif // _UTILS_SYNCSTREAM_HXX_

/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file SimpleLog.hxx
 *
 * A very simple logging mechanism of driver events that is capable of logging
 * a few entries of an 8-byte enum value, in a gdb-friendly way.
 *
 * @author Balazs Racz
 * @date 14 September 2014
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SIMPLELOG_HXX_
#define _FREERTOS_DRIVERS_COMMON_SIMPLELOG_HXX_

/// A very simple logging mechanism of driver events that is capable of logging
/// a few entries of an 8-bit enum value, in a gdb-friendly way.
///
/// C is typically uint64_t.
template <typename C> class SimpleLog
{
public:
    SimpleLog()
        : log_(0)
    {
    }

    /// Append a byte worth of data to the end of the log buffer. Rotates out
    /// some old data.
    void log(uint8_t value)
    {
        log_ <<= 8;
        log_ |= value;
    }

private:
    /// The raw log buffer.
    C log_;
};

/// Actual class that keeps 8 log entries of one byte each.
typedef SimpleLog<uint64_t> LogBuffer;


/// Alternative for hundreds of entries.
template<class T, int N> class LogRing {
public:
    void add(T data) {
        data_[next_] = data;
        last_ = data_ + next_;
        if (next_) {
            --next_;
        } else {
            next_ = N-1;
        }
    }
    
private:
    T data_[N];
    unsigned next_{N};
    T* last_{data_};
};

#endif // _FREERTOS_DRIVERS_COMMON_SIMPLELOG_HXX_

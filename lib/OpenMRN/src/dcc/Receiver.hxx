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
 * \file Receiver.hxx
 *
 * State flow for DCC packet reception and decoding.
 *
 * @author Balazs Racz
 * @date 30 November 2014
 */

#ifndef _DCC_RECEIVER_HXX_
#define _DCC_RECEIVER_HXX_

#include <fcntl.h>
#include <unistd.h>

#include "executor/StateFlow.hxx"

namespace dcc
{

/// State machine for decoding a DCC packet flow. Supports both DCC and
/// Marklin-Motorola packets.
class DccDecoder
{
public:
    DccDecoder()
    {
        timings_[DCC_ONE].set(52, 64);
        timings_[DCC_ZERO].set(95, 9900);
        timings_[MM_PREAMBLE].set(1000, -1);
        timings_[MM_SHORT].set(20, 32);
        timings_[MM_LONG].set(200, 216);
    }

    /// Internal states of the decoding state machine.
    enum State
    {
        UNKNOWN,             // 0
        DCC_PREAMBLE,        // 1
        DCC_END_OF_PREAMBLE, // 2
        DCC_DATA,            // 3
        DCC_DATA_ONE,        // 4
        DCC_DATA_ZERO,       // 5
        DCC_MAYBE_CUTOUT,    // 6
        DCC_CUTOUT,          // 7
        DCC_PACKET_FINISHED, // 8
        MM_DATA,
        MM_ZERO,
        MM_ONE,
        MM_PACKET_FINISHED,
    };

    /// @return the current decoding state.
    State state()
    {
        return parseState_;
    }

    /// Call this function for each time the polarity of the signal changes.
    /// @param value is the number of clock cycles since the last polarity
    /// change.
    void process_data(uint32_t value)
    {
        switch (parseState_)
        {
            case DCC_PACKET_FINISHED:
            case MM_PACKET_FINISHED:
            case UNKNOWN:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseCount_ = 0;
                    parseState_ = DCC_PREAMBLE;
                    return;
                }
                if (timings_[MM_PREAMBLE].match(value))
                {
                    parseCount_ = 1 << 2;
                    ofs_ = 0;
                    data_[ofs_] = 0;
                    parseState_ = MM_DATA;
                }
                break;
            }
            case DCC_PREAMBLE:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseCount_++;
                    return;
                }
                if (timings_[DCC_ZERO].match(value) && (parseCount_ >= 16))
                {
                    parseState_ = DCC_END_OF_PREAMBLE;
                    return;
                }
                break;
            }
            case DCC_END_OF_PREAMBLE:
            {
                if (timings_[DCC_ZERO].match(value))
                {
                    parseState_ = DCC_DATA;
                    parseCount_ = 1 << 7;
                    ofs_ = 0;
                    data_[ofs_] = 0;
                    return;
                }
                break;
            }
            case DCC_DATA:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseState_ = DCC_DATA_ONE;
                    return;
                }
                if (timings_[DCC_ZERO].match(value))
                {
                    parseState_ = DCC_DATA_ZERO;
                    return;
                }
                break;
            }
            case DCC_DATA_ONE:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    if (parseCount_)
                    {
                        data_[ofs_] |= parseCount_;
                        parseCount_ >>= 1;
                        parseState_ = DCC_DATA;
                        return;
                    }
                    else
                    {
                        // end of packet 1 bit.
                        parseState_ = DCC_MAYBE_CUTOUT;
                        return;
                    }
                    return;
                }
                break;
            }
            case DCC_DATA_ZERO:
            {
                if (timings_[DCC_ZERO].match(value))
                {
                    if (parseCount_)
                    {
                        // zero bit into data_.
                        parseCount_ >>= 1;
                    }
                    else
                    {
                        // end of byte zero bit. Packet is not finished yet.
                        ofs_++;
                        HASSERT(ofs_ < sizeof(data_));
                        data_[ofs_] = 0;
                        parseCount_ = 1 << 7;
                    }
                    parseState_ = DCC_DATA;
                    return;
                }
                break;
            }
            case DCC_MAYBE_CUTOUT:
            {
                if (value < timings_[DCC_ZERO].min_value)
                {
                    parseState_ = DCC_CUTOUT;
                    return;
                }
                parseState_ = DCC_PACKET_FINISHED;
                return;
            }
            case DCC_CUTOUT:
            {
                parseState_ = DCC_PACKET_FINISHED;
                return;
            }
            case MM_DATA:
            {
                if (timings_[MM_LONG].match(value))
                {
                    parseState_ = MM_ZERO;
                    return;
                }
                if (timings_[MM_SHORT].match(value))
                {
                    parseState_ = MM_ONE;
                    return;
                }
                break;
            }
            case MM_ZERO:
            {
                if (timings_[MM_SHORT].match(value))
                {
                    // data_[ofs_] |= 0;
                    parseCount_ >>= 1;
                    if (!parseCount_)
                    {
                        if (ofs_ == 2)
                        {
                            parseState_ = MM_PACKET_FINISHED;
                            return;
                        }
                        else
                        {
                            ofs_++;
                            parseCount_ = 1 << 7;
                            data_[ofs_] = 0;
                        }
                    }
                    parseState_ = MM_DATA;
                    return;
                }
                break;
            }
            case MM_ONE:
            {
                if (timings_[MM_LONG].match(value))
                {
                    data_[ofs_] |= parseCount_;
                    parseCount_ >>= 1;
                    if (!parseCount_)
                    {
                        if (ofs_ == 2)
                        {
                            parseState_ = MM_PACKET_FINISHED;
                            return;
                        }
                        else
                        {
                            ofs_++;
                            parseCount_ = 1 << 7;
                            data_[ofs_] = 0;
                        }
                    }
                    parseState_ = MM_DATA;
                    return;
                }
                break;
            }
        }
        parseState_ = UNKNOWN;
        return;
    }

    /// Returns true if we are close to the DCC cutout. This situation is
    /// recognized by having seen the first half of the end-of-packet one bit.
    bool before_dcc_cutout() {
        return (!parseCount_) &&           // end of byte
            (parseState_ == DCC_DATA_ONE); // one bit comes
    }

    /// Returns the number of payload bytes in the current packet.
    uint8_t packet_length()
    {
        return ofs_ + 1;
    }

    /// Returns the current packet payload buffer. The buffer gets invalidated
    /// at the next call to process_data.
    const uint8_t *packet_data()
    {
        return data_;
    }

private:
    uint32_t parseCount_ = 0;
    State parseState_ = UNKNOWN;
    // Payload of current packet.
    uint8_t data_[6];
    uint8_t ofs_; // offset inside data_;

    /// Represents the timing of a half-wave of the digital track signal.
    struct Timing
    {
        void set(int min_usec, int max_usec)
        {
            if (min_usec < 0)
            {
                min_value = 0;
            }
            else
            {
                min_value = usec_to_clock(min_usec);
            }
            if (max_usec < 0)
            {
                max_usec = UINT_MAX;
            }
            else
            {
                max_value = usec_to_clock(max_usec);
            }
        }

        bool match(uint32_t value_clocks) const
        {
            return min_value <= value_clocks && value_clocks <= max_value;
        }

        static uint32_t usec_to_clock(int usec)
        {
            return (configCPU_CLOCK_HZ / 1000000) * usec;
        }

        uint32_t min_value;
        uint32_t max_value;
    };

    /// Indexes the timing array.
    enum TimingInfo
    {
        DCC_ONE = 0,
        DCC_ZERO,
        MM_PREAMBLE,
        MM_SHORT,
        MM_LONG,
        MAX_TIMINGS
    };
    /// The various timings by the standards.
    Timing timings_[MAX_TIMINGS];
};

/// User-space DCC decoding flow. This flow receives a sequence of numbers from
/// the DCC driver, where each number means a specific number of microseconds
/// for which the signal was of the same polarity (e.g. for dcc packet it would
/// be something like 56, 56, 56, 56, ..., 56, 56, 105, 105, 56, 56, ...). Then
/// calls for each packet the virtual function dcc_packet_finished() or
/// mm_packet_finished().
///
/// This flow is a pretty expensive way to decode DCC data.
class DccDecodeFlow : public StateFlowBase
{
public:
    DccDecodeFlow(Service *s, const char *dev)
        : StateFlowBase(s)
    {
        fd_ = ::open(dev, O_RDONLY | O_NONBLOCK);
        start_flow(STATE(register_and_sleep));
    }

private:
    Action register_and_sleep()
    {
        ::ioctl(fd_, CAN_IOC_READ_ACTIVE, this);
        return wait_and_call(STATE(data_arrived));
    }

    Action data_arrived()
    {
        while (true)
        {
            uint32_t value;
            int ret = ::read(fd_, &value, sizeof(value));
            if (ret != 4)
            {
                return call_immediately(STATE(register_and_sleep));
            }
            MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0xff);
            debug_data(value);
            decoder_.process_data(value);
            if (decoder_.state() == DccDecoder::DCC_PACKET_FINISHED)
            {
                dcc_packet_finished(
                    decoder_.packet_data(), decoder_.packet_length());
            }
            else if (decoder_.state() == DccDecoder::MM_PACKET_FINISHED)
            {
                mm_packet_finished(
                    decoder_.packet_data(), decoder_.packet_length());
            }

            static uint8_t x = 0;
            // MAP_GPIOPinWrite(LED_GREEN, 0);
            x = ~x;
        }
    }

    virtual void dcc_packet_finished(const uint8_t* payload, size_t len) = 0;
    virtual void mm_packet_finished(const uint8_t* payload, size_t len) = 0;
    virtual void debug_data(uint32_t value)
    {
    }

    int fd_;
    uint32_t lastValue_ = 0;

protected:
    DccDecoder decoder_;
};

} // namespace dcc

#endif // _DCC_RECEIVER_HXX_

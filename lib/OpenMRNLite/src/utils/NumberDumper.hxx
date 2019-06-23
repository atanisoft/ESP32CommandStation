/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file NumberDumper.hxx
 * Efficient implementation of dumping a lot of numbers to a log.
 *
 * @author Balazs Racz
 * @date 14 May 2018
 */

#ifndef _UTILS_NUMBERDUMPER_HXX_
#define _UTILS_NUMBERDUMPER_HXX_

#include "executor/StateFlow.hxx"
#include "utils/Singleton.hxx"
#include "utils/logging.h"

class NumberDumper : public Singleton<NumberDumper>
{
public:
    NumberDumper(Service *s)
        : displayFlow_(s)
    {
    }

    void add(uint16_t data)
    {
        if (!nextChunk_)
        {
            nextChunk_ = displayFlow_.alloc();
            nextOfs_ = 0;
        }
        nextChunk_->data()->data[nextOfs_] = data;
        if (++nextOfs_ >= BUFSIZE)
        {
            displayFlow_.send(nextChunk_);
            nextChunk_ = nullptr;
        }
    }

    static constexpr uint16_t EOLN = 0xffffu;

private:
    static constexpr unsigned BUFSIZE = 128;

    struct Chunk
    {
        uint16_t data[BUFSIZE];
    };

    Buffer<Chunk> *nextChunk_ = nullptr;
    unsigned nextOfs_ = 0;

    class Displayer : public StateFlow<Buffer<Chunk>, QList<1>>
    {
    public:
        Displayer(Service *s)
            : StateFlow<Buffer<Chunk>, QList<1>>(s)
        {
            endp_ = output;
            thisLineLimit_ = endp_ + MIN_LINE;
        }

        Action entry() override
        {
            nextOfs_ = 0;
            return call_immediately(STATE(render));
        }

        Action render()
        {
            while (true)
            {
                bool send_off = false;
                if (nextOfs_ >= BUFSIZE)
                {
                    return release_and_exit();
                }
                auto d = message()->data()->data[nextOfs_];
                if (d == EOLN)
                {
                    if (!line_prefer_end())
                    {
                        *endp_++ = '|';
                    }
                    else if (fits_next_line())
                    {
                        *endp_++ = '\n';
                        thisLineLimit_ = endp_ + MIN_LINE;
                    }
                    else
                    {
                        send_off = true;
                    }
                }
                else if (!has_one_number())
                {
                    send_off = true;
                }

                if (send_off)
                {
                    GLOBAL_LOG_OUTPUT(output, endp_ - output);
                    endp_ = output;
                    thisLineLimit_ = endp_ + MIN_LINE;
                }

                if (d != EOLN)
                {
                    endp_ = unsigned_integer_to_buffer(d, endp_);
                    *endp_ = ' ';
                    endp_++;
                }
                if (++nextOfs_ % 30 == 0)
                    return yield();
            }
        }

    private:
        /// @return true if we still have a numbers' worth of buffer.
        bool has_one_number()
        {
            return endp_ < (output_limit() - 12);
        }

        /// @return true if this line end should be a physical line end
        bool line_prefer_end()
        {
            return endp_ >= thisLineLimit_;
        }

        /// @return true if we can fit the next line within the buffer.
        bool fits_next_line()
        {
            return endp_ + (MIN_LINE * 3) <= output_limit();
        }

        /// @return pointer to the end of the physical buffer.
        char *output_limit()
        {
            return output + sizeof(output);
        }

        /// Pointer in the buffer to the terminating null.
        char *endp_;
        /// Pointer in the buffer where the current line should end the latest.
        char *thisLineLimit_;
        /// Index into the input number array where to continue rendering.
        uint16_t nextOfs_;
        /// Output buffer where we render the numbers.
        char output[320];
        /// Desired length of a line.
        static constexpr uint8_t MIN_LINE = 60;
    };

    Displayer displayFlow_;
};

#endif // _UTILS_NUMBERDUMPER_HXX_

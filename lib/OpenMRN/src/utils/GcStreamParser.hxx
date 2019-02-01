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
 * \file GcStreamParser.hxx
 *
 * Finds gridconnect packets in a stream of characters.
 *
 * @author Balazs Racz
 * @date 26 May 2016
 */

#ifndef _UTILS_GCSTREAMPARSER_HXX_
#define _UTILS_GCSTREAMPARSER_HXX_

#include <string>

/**
   Parses a sequence of characters; finds GridConnect protocol packet
   boundaries in the sequence of packets. Contains an internal buffer holding
   the partial (or last found) gridconnect packet.

   This class is not thread-safe, but thread-compatible.
 */
class GcStreamParser
{
public:
    GcStreamParser()
        : offset_(-1)
    {
    }

    /** Adds the next character from the source stream. @return true if the
     * internal buffer contains a complete frame. @param c next character. */
    bool consume_byte(char c);

    /** Parses the current contents of the frame buffer to a can_frame
     * struct. Should be called if and inly if the previous consume_char call
     * returned true.
     *
     * @param output_frame is an output argument, non-NULL, into this we will
     * be writing the binary frame.
     * @return true on success, false if there was a parse error. In this case
     * the frame is set to an error frame. */
    bool parse_frame_to_output(struct can_frame *output_frame);

    /** @param payload fills with the current contents of the frame buffer. */
    void frame_buffer(std::string *payload);

private:
    /// Collects data from a partial GC packet.
    char cbuf_[32];
    /// offset of next byte in cbuf to write.
    int offset_;
};

#endif // _UTILS_GCSTREAMPARSER_HXX_

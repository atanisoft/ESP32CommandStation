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

#include <string>

#include "utils/GcStreamParser.hxx"
#include "utils/gc_format.h"

bool GcStreamParser::consume_byte(char c)
{
    if (c == ':')
    {
        // Frame is starting here.
        offset_ = 0;
        return false;
    }
    if (c == ';')
    {
        if (offset_ < 0)
        {
            return false;
        }
        // Frame ends here.
        cbuf_[offset_] = 0;
        offset_ = -1;
        return true;
    }
    if (offset_ >= static_cast<int>(sizeof(cbuf_) - 1))
    {
        // We overran the buffer, so this can't be a valid frame.
        // Reset and look for sync byte again.
        offset_ = -1;
        return false;
    }
    if (offset_ >= 0)
    {
        cbuf_[offset_++] = c;
    }
    else
    {
        // Drop byte to the floor -- we're not in the middle of a
        // packet.
    }
    return false;
}

void GcStreamParser::frame_buffer(std::string* payload) {
    if (offset_ >= 0) {
        payload->assign(cbuf_, offset_);
    } else {
        payload->assign(cbuf_);
    }
}

bool GcStreamParser::parse_frame_to_output(struct can_frame* output_frame) {
    int ret = gc_format_parse(cbuf_, output_frame);
    return (ret == 0);
}

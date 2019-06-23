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
 * \file StreamDefs.hxx
 * Constants and helper function useful for a stream implementation.
 *
 * @author Balazs Racz
 * @date 14 December 2014
 */

#include "openlcb/If.hxx"

namespace openlcb
{

/// Static constants and helper functions for the OpenLCB streaming protocol.
struct StreamDefs
{
    static const uint16_t MAX_PAYLOAD = 0xffff;

    enum Flags
    {
        FLAG_CARRIES_ID = 0x01,
        FLAG_REJECT_OUT_OF_ORDER = 0x02,
        FLAG_PERMANENT_ERROR = 0x40,
        FLAG_ACCEPT = 0x80,
    };

    enum AdditionalFlags
    {
        REJECT_INFO_LOGGED = 0x01,
        REJECT_PERMANENT_INVALID_REQUEST = 0x20,
        REJECT_PERMANENT_SRC_NOT_PERMITTED = 0x40,
        REJECT_PERMANENT_STREAMS_NOT_ACCEPTED = 0x80,
        REJECT_TEMPORARY_BUFFER_FULL = 0x20,
        REJECT_TEMPORARY_OUT_OF_ORDER = 0x40,
    };

    static Payload create_initiate_request(uint16_t max_buffer_size,
                                           bool has_ident,
                                           uint8_t src_stream_id)
    {
        Payload p(5, 0);
        p[0] = max_buffer_size >> 8;
        p[1] = max_buffer_size & 0xff;
        p[2] = has_ident ? FLAG_CARRIES_ID : 0;
        p[3] = 0;
        p[4] = src_stream_id;
        return p;
    }

    static Payload create_close_request(uint8_t src_stream_id, uint8_t dst_stream_id)
    {
        Payload p(2, 0);
        p[0] = src_stream_id;
        p[1] = dst_stream_id;
        return p;
    }
};

} // namespace openlcb

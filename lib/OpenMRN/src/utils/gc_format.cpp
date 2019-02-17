/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file gc_format.cxx
 * Implementations of GridConnect packet parser and formatter routines.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

//#define LOGLEVEL VERBOSE

#include <stdint.h>
#include "utils/logging.h"
#include "utils/gc_format.h"
#include "can_frame.h"

extern "C" {

/** Build an ASCII character representation of a nibble value (uppercase hex).
 * @param nibble to convert
 * @return converted value
 */
static char nibble_to_ascii(int nibble)
{
    nibble &= 0xf;

    if (nibble < 10)
    {
        return ('0' + nibble);
    }
    
    return ('A' + (nibble - 10));
}

/** Tries to parse a hex character to a nibble. Understands both upper and
    lowercase hex.
    @param c is the character to convert.
    @return a converted value, or -1 if an invalid character was encountered.
*/
static int ascii_to_nibble(const char c)
{
    if ('0' <= c && '9' >= c)
    {
        return c - '0';
    }
    else if ('A' <= c && 'F' >= c)
    {
        return c - 'A' + 10;
    }
    else if ('a' <= c && 'f' >= c)
    {
        return c - 'a' + 10;
    }
    return -1;
}


int gc_format_parse(const char* buf, struct can_frame* can_frame)
{
    CLR_CAN_FRAME_ERR(*can_frame);
    if (*buf == 'X')
    {
        SET_CAN_FRAME_EFF(*can_frame);
    }
    else if (*buf == 'S') 
    {
        CLR_CAN_FRAME_EFF(*can_frame);
    } else
    {
        // Unknown packet type.
        SET_CAN_FRAME_ERR(*can_frame);
        return -1;
    }
    buf++;
    uint32_t id = 0;
    while (1)
    {
        int nibble = ascii_to_nibble(*buf);
        if (nibble >= 0)
        {
            id <<= 4;
            id |= nibble;
            ++buf;
        }
        else if (*buf == 'N')
        {
            // end of ID, frame is coming.
            CLR_CAN_FRAME_RTR(*can_frame);
            ++buf;
            break;
        }
        else if (*buf == 'R')
        {
            // end of ID, remote frame is coming.
            SET_CAN_FRAME_RTR(*can_frame);
            ++buf;
            break;
        }
        else
        {
            // This character should not happen here.
            SET_CAN_FRAME_ERR(*can_frame);
            return -1;
        }
    } // while parsing ID
    if (IS_CAN_FRAME_EFF(*can_frame))
    {
        SET_CAN_FRAME_ID_EFF(*can_frame, id);
    }
    else
    { 
        SET_CAN_FRAME_ID(*can_frame, id);
    }
    int index = 0;
    while (*buf)
    {
        int nh = ascii_to_nibble(*buf++);
        int nl = ascii_to_nibble(*buf++);
        if (nh < 0 || nl < 0)
        {
            SET_CAN_FRAME_ERR(*can_frame);
            return -1;
        }
        can_frame->data[index++] = (nh << 4) | nl;
    } // while parsing data
    can_frame->can_dlc = index;
    CLR_CAN_FRAME_ERR(*can_frame);
    return 0;
}

/// Helper function for appending to a buffer ONCE.
///
/// @param dst buffer to append data to
/// @param value one character to append to.
///
static void output_single(char*& dst, char value)
{
    *dst++ = value;
}

/// Helper function for appending to a buffer TWICE. Used in the implementation
/// of the double-byte gridconnect output protocol.
///
/// @param dst buffer to append data to
/// @param value one character to append to.
///
static void output_double(char*& dst, char value)
{
    *dst++ = value;
    *dst++ = value;
}

/** Formats a can frame in the GridConnect protocol.

    If requested, it can create the double protocol with leading !!, trailing ;;
    and all the characters doubled.

    If the input frame is an error frame, then does not output anything and
    returns the input pointer.

    @param can_frame is the input frame.

    @param buf is the output buffer. The caller must ensure this is big enough
    to hold the resulting frame (28 or 56 bytes).

    @param double_format if non-zero, the doubling format will be generated.

    @return the pointer to the buffer character after the formatted can frame.
*/
char* gc_format_generate(const struct can_frame* can_frame, char* buf, int double_format)
{
    if (IS_CAN_FRAME_ERR(*can_frame))
    {
        LOG(VERBOSE, "GC generate: incoming frame ERR.");
        return buf;
    }
    void (*output)(char*& dst, char value);
    if (double_format)
    {
        output = output_double;
        output(buf, '!');
    }
    else
    {
        output = output_single;
        output(buf, ':');
    }
    uint32_t id;
    int offset;
    if (IS_CAN_FRAME_EFF(*can_frame))
    {
        id = GET_CAN_FRAME_ID_EFF(*can_frame);
        output(buf, 'X');
        offset = 28;
    }
    else
    {
        id = GET_CAN_FRAME_ID(*can_frame);
        output(buf, 'S');
        offset = 8;
    }
    for (;offset >= 0; offset -= 4)
    {
        output(buf, nibble_to_ascii((id >> offset) & 0xf));
    }
    /* handle remote or normal */
    if (IS_CAN_FRAME_RTR(*can_frame))
    {
        output(buf, 'R');
    }
    else
    {
        output(buf, 'N');
    }
    for (offset = 0; offset < can_frame->can_dlc; ++offset)
    {
        output(buf, nibble_to_ascii(can_frame->data[offset] >> 4));
        output(buf, nibble_to_ascii(can_frame->data[offset] & 0xf));
    }
    output(buf, ';');
    if (config_gc_generate_newlines()) {
        output(buf, '\n');
    }
    return buf;
}

}

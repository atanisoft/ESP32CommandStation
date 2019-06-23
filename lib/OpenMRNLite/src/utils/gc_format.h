/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file gc_format.h
 * Headers for conversion to gridconnect format.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

#ifndef _UTILS_GC_FORMAT_H_
#define _UTILS_GC_FORMAT_H_

#include "utils/constants.hxx"

#ifdef __cplusplus
extern "C" {
#endif

struct can_frame;

/// Whether gridconnect format should create newline characters at the end of
/// packets.
DECLARE_CONST(gc_generate_newlines);

/** Parses a GridConnect packet.
    
    @param buf points to a character buffer that contains the packet. The
    leading ":" must be already removed, the tailing ';' must be replaced by a
    \0 char.

    @param can_frame is the CAN frame that will be filled based on the source
    packet.

    @return 0 in case of success, -1 if there was a packet format error (in
    this case the frame is set to an error frame).
*/
int gc_format_parse(const char* buf, struct can_frame* can_frame);

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
char* gc_format_generate(const struct can_frame* can_frame, char* buf, int double_format);

#ifdef __cplusplus
}
#endif


#endif // _UTILS_GC_FORMAT_H_

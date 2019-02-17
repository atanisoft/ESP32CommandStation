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
 * \file dcc/Defs.hxx
 *
 * Definitions for DCC concepts.
 *
 * @author Balazs Racz
 * @date 27 Feb 2016
 */

#ifndef _DCC_DEFS_HXX_
#define _DCC_DEFS_HXX_

namespace dcc {

/// Which address type this legacy train node uses. These address types
/// translate to mutually independent packets on the track.
enum class TrainAddressType : uint8_t
{
    /// DCC packets with short address (1..127)
    DCC_SHORT_ADDRESS = 1,
    /// DCC packets with long address (128..~10000)
    DCC_LONG_ADDRESS,
    /// Marklin-motorola packets. Addresses 1..80 are supported.
    MM,
    /// Unsupported address type (e.g. a protocol we don't have an
    /// implementation for).
    UNSUPPORTED = 255,
    /// Unspecified address type (default / match any).
    UNSPECIFIED = 254,
};

}  // namespace dcc

#endif

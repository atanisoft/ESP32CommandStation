/** \copyright
 * Copyright (c) 2019, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file FirmwareUpgradeDefs.hxx
 *
 * Definitions from the Firmware Upgrade Standard
 *
 * @author Balazs Racz
 * @date 14 Dec 2019
 */

#ifndef _OPENLCB_FIRMWAREUPGRADEDEFS_HXX_
#define _OPENLCB_FIRMWAREUPGRADEDEFS_HXX_

namespace openlcb
{

/// Contains definitions from the Firmware Upgrade Standard.
struct FirmwareUpgradeDefs
{
    enum
    {
        ERROR_WRITE_CHECKSUM_FAILED = 0x2088,
        ERROR_INCOMPATIBLE_FIRMWARE = 0x1088,
        ERROR_CORRUPTED_DATA = 0x1089,
    };

    enum
    {
        SPACE_FIRMWARE = 0xEF,
    };
};

}

#endif // _OPENLCB_FIRMWAREUPGRADEDEFS_HXX_

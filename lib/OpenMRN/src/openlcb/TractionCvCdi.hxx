/** \copyright
 * Copyright (c) 2014-2016, Balazs Racz
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
 * \file TractionCvCdi.hxx
 *
 * CDI entry defining the commandstation traindb entry.
 *
 * @author Balazs Racz
 * @date 20 Mar 2016
 */

#ifndef _OPENLCB_TRACTIONCVCDI_HXX_
#define _OPENLCB_TRACTIONCVCDI_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace openlcb {

/// CDI group that can be used with the TractionCvSpace class to enable CV
/// access via MemoryConfig protocol.
CDI_GROUP(TractionShortCvSpace, Segment(openlcb::MemoryConfigDefs::SPACE_DCC_CV), Offset(0x7F000000), Name("CV access"), Description("Individual CVs can be read and modified for Railcom-enabled locomotives using POM commands. Write the CV number variable first, then write or read the CV value variable."));
/// Which CV number to read/write.
CDI_GROUP_ENTRY(number, Uint32ConfigEntry, Name("CV number"));
/// Value of the given CV.
CDI_GROUP_ENTRY(value, Uint8ConfigEntry, Name("CV value"));
/// Verify value
CDI_GROUP_ENTRY(verify_value, Uint8ConfigEntry, Name("Verify CV value"),
    Description("Write this value first to send out programming track verify "
                "commands"));
CDI_GROUP_ENTRY(
    verify_result, Uint8ConfigEntry, Name("Verify CV result"),
    Description(
        "Refresh this line to verify CV value. Return 0=no ack, 1=ack."));

CDI_GROUP_END();

static_assert(TractionShortCvSpace::size() == 7,
    "Traction CV space's size not as expected.");

}  // namespace openlcb

#endif // _OPENLCB_TRACTIONCVCDI_HXX_

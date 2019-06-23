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
 * \file RoutingLogic.cxx
 *
 * Dat structure for gateways and routers of OpenLCB, keeping track of the
 * necessary information to make routing decisions.
 *
 * @author Balazs Racz
 * @date 23 May 2016
 */

#include "openlcb/RoutingLogic.hxx"

namespace openlcb {

/** Decodes an event range, encoded according to the Event Transport protocol
 * specification.
 *
 * @param event is a pointer to the variable holding the event range. This
 * value will be modified to hold only the base value of the event, without the
 * mask bits.
 *
 * @returns the number of mask bits that were there, in the range of 1..64.
 */
uint8_t event_range_to_bit_count(EventId* event) {
    EventId e = *event;
    EventId mask = 1;
    uint8_t ret = 0;
    EventId value = e & mask;
    while (ret < 64 && ((e & mask) == value)) {
        mask <<= 1;
        value <<= 1;
        ++ret;
    }
    *event &= ~(mask - 1);
    return ret;
}

} // namespace openlcb

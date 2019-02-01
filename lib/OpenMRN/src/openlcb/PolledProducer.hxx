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
 * \file PolledProducer.hxx
 * A complete class that acts as a producer for a bit event from a polled input
 * with debouncing.
 *
 * @author Balazs Racz
 * @date 13 Jul 2014
 */

#ifndef _OPENLCB_POLLEDPRODUCER_HXX_
#define _OPENLCB_POLLEDPRODUCER_HXX_

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/RefreshLoop.hxx"

namespace openlcb {

/// Producer class for GPIO bits. Polls the hardware state at a given
/// frequency, applies a debouncing algorithm, and when the output of the
/// devbouncing algorithm changes, requests the Producer class to update (i.e.,
/// generate the event report message to the OpenLCB bus).
template <class Debouncer, class BaseBit>
class PolledProducer : public BaseBit, public Polling
{
public:
    template <typename... Fields>
    PolledProducer(const typename Debouncer::Options &debounce_args,
                   Fields... bit_args)
        : BaseBit(bit_args...)
        , debouncer_(debounce_args)
        , producer_(this)
    {
        debouncer_.initialize(BaseBit::get_current_state() == EventState::VALID);
    }

    EventState get_current_state() OVERRIDE
    {
        return debouncer_.current_state() ? EventState::VALID : EventState::INVALID;
    }

    void set_state(bool new_value) OVERRIDE
    {
        debouncer_.override(new_value);
    }

    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (debouncer_.update_state(BaseBit::get_current_state() == EventState::VALID))
        {
            producer_.SendEventReport(helper, done);
        }
        else
        {
            done->notify();
        }
    }

private:
    Debouncer debouncer_;
    BitEventPC producer_;
};

} // namespace openlcb

#endif // _OPENLCB_POLLEDPRODUCER_HXX_

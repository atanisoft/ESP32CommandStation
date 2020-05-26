/** @copyright
 * Copyright (c) 2018, Stuart W. Baker
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
 * @file BroadcastTimeClient.cxx
 *
 * Implementation of a Broadcast Time Protocol client.
 *
 * @author Stuart W. Baker
 * @date 14 October 2018
 */

#include "openlcb/BroadcastTimeClient.hxx"

namespace openlcb
{

//
// BroadcastTimeClient::handle_updates()
//
void BroadcastTimeClient::handle_updates(EventReport *event, bool report)
{
    BroadcastTimeDefs::EventType type;
    type = BroadcastTimeDefs::get_event_type(event->event);

    switch (type)
    {
        case BroadcastTimeDefs::REPORT_TIME:
        {
            int min = BroadcastTimeDefs::event_to_min(event->event);
            int hour = BroadcastTimeDefs::event_to_hour(event->event);
            if (min >= 0 && hour >= 0)
            {
                tm_.tm_sec = 0;
                tm_.tm_min = min;
                tm_.tm_hour = hour;
                immediateUpdate_ = report;
                break;
            }
            // invalid event data, bail
            return;
        }
        case BroadcastTimeDefs::REPORT_DATE:
        {
            int day = BroadcastTimeDefs::event_to_day(event->event);
            int month = BroadcastTimeDefs::event_to_month(event->event);
            if (day >= 0 && month >= 0)
            {
                tm_.tm_mday = day;
                tm_.tm_mon = month - 1;
                if (report)
                {
                    rolloverPendingDate_ = false;
                }
                break;
            }
            // invalid event data, bail
            return;
        }
        case BroadcastTimeDefs::REPORT_YEAR:
        {
            tm_.tm_year = BroadcastTimeDefs::event_to_year(event->event) - 1900;
            if (report)
            {
                rolloverPendingYear_ = false;
            }
            break;
        }
        case BroadcastTimeDefs::REPORT_RATE:
        {
            rateRequested_ = BroadcastTimeDefs::event_to_rate(event->event);
            break;
        }
        case BroadcastTimeDefs::DATE_ROLLOVER:
            if (report)
            {
                rolloverPendingDate_ = true;
                rolloverPendingYear_ = true;
                rolloverPending_ = true;
            }
            break;
        case BroadcastTimeDefs::STOP:
            start_stop_logic(false);
            // no further processing required
            return;
        case BroadcastTimeDefs::START:
            start_stop_logic(true);
            // no further processing required
            return;
        default:
            // uninteresting event type
            return;
    }

    if (!report)
    {
        // An immediate update is expected in the future
        immediatePending_ = true;
    }

    wakeup();
}

} // namespace openlcb

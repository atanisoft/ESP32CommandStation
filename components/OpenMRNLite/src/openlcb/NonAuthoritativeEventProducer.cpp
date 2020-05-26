/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file NonAuthoritativeEventProducer.cxx
 *
 * Defines implementations of Event Producers that are uathoratative, meaning
 * that they do not represent the actual state of an event, but can still
 * query that state (presumably from a consumer) and produce a new state.
 *
 * @author Stuart Baker
 * @date 17 June 2017
 */

#include "openlcb/NonAuthoritativeEventProducer.hxx"
#include "openlcb/EventService.hxx"

namespace openlcb
{

//
// BitRangeNonauthoritativeEventP::send_query_consumer()
//
void BitRangeNonAuthoritativeEventP::send_query_consumer(unsigned bit,
                                                        WriteHelper *writer,
                                                        BarrierNotifiable *done)
{
    HASSERT(bit < size_);
    uint64_t event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) :
                                          eventBaseOn_ + bit;
    writer->WriteAsync(node_, Defs::MTI_CONSUMER_IDENTIFY,
                       WriteHelper::global(),
                       eventid_to_buffer(event), done);
}

//
// BitRangeNonauthoritativeEventP::handle_event_report()
//
void BitRangeNonAuthoritativeEventP::handle_event_report(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    done->notify();
    if (!stateCallback_)
    {
        // there is nobody to notify
        return;
    }

    switch (entry.user_arg)
    {
        default:
            // uninteresting event range, should never get here
            HASSERT(0);
        case EVENT_BASE:
            if (event->event >= eventBase_ &&
                event->event < (eventBase_ + (size_ * 2)))
            {
                bool value = (event->event % 2) == (eventBase_ % 2);
                stateCallback_((event->event - eventBase_) / 2, value);
            }
            break;
        case EVENT_BASE_ON:
            if (event->event >= eventBaseOn_ &&
                event->event < (eventBaseOn_ + size_))
            {
                stateCallback_((event->event - eventBaseOn_), true);
            }
            break;
        case EVENT_BASE_OFF:
            if (event->event >= eventBaseOff_ &&
                event->event < (eventBaseOff_ + size_))
            {
                stateCallback_((event->event - eventBaseOff_), false);
            }
            break;
    }
}

//
// BitRangeNonauthoritativeEventP::handle_consumer_identified()
//
void BitRangeNonAuthoritativeEventP::handle_consumer_identified(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    done->notify();
    if (!stateCallback_)
    {
        // there is nobody to notify
        return;
    }

    bool value;
    if (event->state == EventState::VALID)
    {
        value = true;
    }
    else if (event->state == EventState::INVALID)
    {
        value = false;
    }
    else
    {
        return; // nothing to learn from this message.
    }

    switch (entry.user_arg)
    {
        default:
            // uninteresting event range, should never get here
            HASSERT(0);
        case EVENT_BASE:
            if (event->event >= eventBase_ &&
                event->event < (eventBase_ + (size_ * 2)))
            {
                if ((event->event % 2) != (eventBase_ % 2))
                {
                    value = !value;
                }
                stateCallback_((event->event - eventBase_) / 2, value);
            }
            break;
        case EVENT_BASE_ON:
            if (event->event >= eventBaseOn_ &&
                event->event < (eventBaseOn_ + size_))
            {
                stateCallback_((event->event - eventBase_), value);
            }
            break;
        case EVENT_BASE_OFF:
            if (event->event >= eventBaseOff_ &&
                event->event < (eventBaseOff_ + size_))
            {
                stateCallback_((event->event - eventBaseOff_), !value);
            }
            break;
    }
}

//
// BitRangeNonauthoritativeEventP::handle_identify_producer()
//
void BitRangeNonAuthoritativeEventP::handle_identify_global(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    if (event->dst_node && event->dst_node != node_)
    {
        done->notify();
    }
    else
    {
        uint64_t range;
        switch (entry.user_arg)
        {
            default:
                // uninteresting event range, should never get here
                HASSERT(0);
            case EVENT_BASE:
                range = EncodeRange(eventBase_, size_ * 2);
                break;
            case EVENT_BASE_ON:
                range = EncodeRange(eventBaseOn_, size_);
                break;
            case EVENT_BASE_OFF:
                range = EncodeRange(eventBaseOff_, size_);
                break;
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(range), done);
    }
}

//
// BitRangeNonauthoritativeEventP::handle_identify_producer()
//
void BitRangeNonAuthoritativeEventP::handle_identify_producer(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    bool valid = false;
    switch (entry.user_arg)
    {
        default:
            // uninteresting event range, should never get here
            HASSERT(0);
        case EVENT_BASE:
            if (event->event >= eventBase_ &&
                event->event < (eventBase_ + (size_ * 2)))
            {
                valid = true;
            }
            break;
        case EVENT_BASE_ON:
            if (event->event >= eventBaseOn_ &&
                event->event < (eventBaseOn_ + size_))
            {
                valid = true;
            }
            break;
        case EVENT_BASE_OFF:
            if (event->event >= eventBaseOff_ &&
                event->event < (eventBaseOff_ + size_))
            {
                valid = true;
            }
            break;
    }

    if (valid)
    {
        event->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
            eventid_to_buffer(event->event), done);
    }
    else
    {
        done->notify();
    }
}

//
// BitRangeNonauthoritativeEventP::set()
//
void BitRangeNonAuthoritativeEventP::set(unsigned bit, bool new_value,
                                         WriteHelper *writer,
                                         BarrierNotifiable *done)
{
    HASSERT(bit < size_);

    uint64_t event;
    if (new_value)
    {
        event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) :
                                     eventBaseOn_ + bit;
    }
    else
    {
        event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) + 1 :
                                     eventBaseOff_ + bit;
    }

    writer->WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
                       eventid_to_buffer(event), done);
}

} // namespace openlcb

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
 * @file BroadcastTimeDefs.hxx
 *
 * Static definitions for implementations of the OpenLCB Broadcast Time
 * Protocol.
 *
 * @author Stuart W. Baker
 * @date 14 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMEDEFS_HXX_
#define _OPENLCB_BROADCASTTIMEDEFS_HXX_

#include <time.h>

#include "openlcb/Defs.hxx"

namespace openlcb
{

/// Static constants and helper functions for Broadcast Time Protocol.
struct BroadcastTimeDefs
{
    /// Unique identifier for the Default Fast Clock
    static constexpr NodeID DEFAULT_FAST_CLOCK_ID = 0x010100000100ULL;

    /// Unique identifier for the Default Real-Time Clock
    static constexpr NodeID DEFAULT_REALTIME_CLOCK_ID = 0x010100000101ULL;

    /// Unique identifier for Alternate Clock 1
    static constexpr NodeID ALTERNATE_CLOCK_1_ID = 0x010100000102ULL;

    /// Unique identifier for Alternate Clock 2
    static constexpr NodeID ALTERNATE_CLOCK_2_ID = 0x010100000103ULL;

    /// Type of event
    enum EventType
    {
        REPORT_TIME = 0, ///< report time event
        REPORT_DATE,     ///< report date event
        REPORT_YEAR,     ///< report year event
        REPORT_RATE,     ///< report rate event
        SET_TIME,        ///< set time event
        SET_DATE,        ///< set date event
        SET_YEAR,        ///< set year event
        SET_RATE,        ///< set rate event
        QUERY,           ///< query event
        STOP,            ///< stop clock event
        START,           ///< start clock event
        DATE_ROLLOVER,   ///< date rollover event
        UNDEFINED,       ///< undefined event
    };

    enum
    {
        EVENT_ID_MASK      = 0xFFFFFFFFFFFF0000, ///< Unique ID mask
        EVENT_SUFFIX_MASK  = 0x000000000000FFFF, ///< suffix mask
        EVENT_TYPE_MASK    = 0x000000000000F000, ///< type mask
        EVENT_HOURS_MASK   = 0x0000000000001F00, ///< hours mask
        EVENT_MINUTES_MASK = 0x00000000000000FF, ///< minutes mask
        EVENT_MONTH_MASK   = 0x0000000000000F00, ///< month mask
        EVENT_DAY_MASK     = 0x00000000000000FF, ///< day mask
        EVENT_YEAR_MASK    = 0x0000000000000FFF, ///< rate mask
        EVENT_RATE_MASK    = 0x0000000000000FFF, ///< rate mask

        EVENT_HOURS_SHIFT   = 8, ///< hours mask
        EVENT_MINUTES_SHIFT = 0, ///< minutes mask
        EVENT_MONTH_SHIFT   = 8, ///< month mask
        EVENT_DAY_SHIFT     = 0, ///< day mask
        EVENT_YEAR_SHIFT    = 0, ///< year mask
        EVENT_RATE_SHIFT    = 0, ///< rate mask

        TIME_EVENT_BASE_SUFFIX     = 0x0000, ///< time event base suffix
        DATE_EVENT_BASE_SUFFIX     = 0x2000, ///< date event base suffix
        YEAR_EVENT_BASE_SUFFIX     = 0x3000, ///< year event base suffix
        RATE_EVENT_BASE_SUFFIX     = 0x4000, ///< rate event base suffix
        QUERY_EVENT_SUFFIX         = 0xF000, ///< query event suffix value
        STOP_EVENT_SUFFIX          = 0xF001, ///< stop clock event suffix value
        START_EVENT_SUFFIX         = 0xF002, ///< start clock event suffix value
        DATE_ROLLOVER_EVENT_SUFFIX = 0xF003, ///< rollover the date suffix value

        EVENT_SET_SUFFIX_MASK = 0x8000, ///< suffix max for setting a property
    };

    enum
    {
        SUNDAY = 0, ///< Day of the week value Sunday
        MONDAY, ///< Day of the week value Monday
        TUESDAY, ///< Day of the week value Tuesday
        WEDNESDAY, ///< Day of the week value Wednesday
        THURSDAY, ///< Day of the week value Thursday
        FRIDAY, ///< Day of the week value Friday
        SATURDAY, ///< Day of the week value Saturday
    };

    /// Get the EventTuype from the event suffix number.
    /// @param suffix 16-bit event suffix
    /// @return the EventType
    static EventType get_event_type(uint16_t suffix)
    {
        switch (suffix & EVENT_TYPE_MASK)
        {
            case 0x0000:
            case 0x1000:
                if (valid_time(suffix))
                {
                    return REPORT_TIME;
                }
                break;
            case 0x2000:
                if (valid_date(suffix))
                {
                    return REPORT_DATE;
                }
                break;
            case 0x3000:
                return REPORT_YEAR;
            case 0x4000:
                return REPORT_RATE;
            case 0x8000:
            case 0x9000:
                if (valid_time(suffix))
                {
                    return SET_TIME;
                }
                break;
            case 0xA000:
                if (valid_date(suffix))
                {
                    return SET_DATE;
                }
                break;
            case 0xB000:
                return SET_YEAR;
            case 0xC000:
                return SET_RATE;
            case 0xF000:
                switch (suffix & 0xFFF)
                {
                    case 0x000:
                        return QUERY;
                    case 0x001:
                        return STOP;
                    case 0x002:
                        return START;
                    case 0x003:
                        return DATE_ROLLOVER;
                    default:
                        break;
                }                        
            default:
                break;
        }
        return UNDEFINED;
    }

    /// Validate that this is a supported time event. Assume that the four most
    /// significant bits have been seperately validated.
    /// @return true of valid, else false
    static bool valid_time(uint16_t suffix)
    {
        return (((suffix & EVENT_HOURS_MASK) >> EVENT_HOURS_SHIFT) <= 23 &&
                ((suffix & EVENT_MINUTES_MASK) >> EVENT_MINUTES_SHIFT) <= 59);
    }

    /// Validate that this is a supported date event. Assume that the four most
    /// significant bits have been seperately validated.
    /// @return true of valid, else false
    static bool valid_date(uint16_t suffix)
    {
        return (((suffix & EVENT_MONTH_MASK) >> EVENT_MONTH_SHIFT) >= 1 &&
                ((suffix & EVENT_MONTH_MASK) >> EVENT_MONTH_SHIFT) <= 12 &&
                ((suffix & EVENT_DAY_MASK) >> EVENT_DAY_SHIFT) >= 1 &&
                ((suffix & EVENT_DAY_MASK) >> EVENT_DAY_SHIFT) <= 31);
    }

    /// Get the minutes from the event.  To save logic, the event is assumed to
    /// be of type REPORT_TIME.
    /// @return -1 on error, else minute
    static int event_to_min(uint64_t event)
    {
        unsigned min = (event & EVENT_MINUTES_MASK) >> EVENT_MINUTES_SHIFT;
        if (min <= 59)
        {
            return min;
        }
        return -1;
    }
    
    /// Get the hour from the event.  To save logic, the event is assumed to
    /// be of type REPORT_TIME.
    /// @return -1 on error, else hour
    static int event_to_hour(uint64_t event)
    {
        unsigned hour = (event & EVENT_HOURS_MASK) >> EVENT_HOURS_SHIFT;
        if (hour <= 23)
        {
            return hour;
        }
        return -1;
    }
    
    /// Get the day from the event.  To save logic, the event is assumed to
    /// be of type REPORT_DATE.
    /// @return -1 on error, else day
    static int event_to_day(uint64_t event)
    {
        unsigned day = (event & EVENT_DAY_MASK) >> EVENT_DAY_SHIFT;
        if (day >= 1 && day <= 31)
        {
            return day;
        }
        return -1;
    }
    
    /// Get the month from the event.  To save logic, the event is assumed to
    /// be of type REPORT_DATE.
    /// @return -1 on error, else month (January = 1)
    static int event_to_month(uint64_t event)
    {
        unsigned month = (event & EVENT_MONTH_MASK) >> EVENT_MONTH_SHIFT;
        if (month >= 1 && month <= 12)
        {
            return month;
        }
        return -1;
    }
    
    /// Get the year from the event.  To save logic, the event is assumed to
    /// be of type REPORT_YEAR.
    /// @return years past 0AD
    static int event_to_year(uint64_t event)
    {
        return (event & EVENT_YEAR_MASK) >> EVENT_YEAR_SHIFT;
    }
    
    /// Get the rate from the event.  To save logic, the event is assumed to
    /// be of type REPORT_RATE.
    /// @return signed 12-bit rate value.
    static int16_t event_to_rate(uint64_t event)
    {
        union Rate
        {
            uint16_t rate_;
            int16_t srate_;
        };

        Rate rate;
        
        rate.rate_ = (event & EVENT_RATE_MASK) >> EVENT_RATE_SHIFT;
        if (rate.rate_ & 0x0800)
        {
            // sign extend negative value
            rate.rate_ |= 0xF000;
        }

        return rate.srate_;
    }

    /// Build an event from hours and minutes.
    /// @param event_base base event ID of the event pool
    /// @param hours hours (0 to 23)
    /// @param minutes minutes (0 to 59)
    /// @return resulting event ID
    static uint64_t time_to_event(uint64_t event_base, int hours, int minutes)
    {
        HASSERT(minutes >= 0 && minutes <= 59);
        HASSERT(hours >= 0 && hours <= 23);

        return event_base + TIME_EVENT_BASE_SUFFIX +
               (hours << EVENT_HOURS_SHIFT) + (minutes << EVENT_MINUTES_SHIFT);
    }

    /// Build an event from month and day.
    /// @param event_base base event ID of the event pool
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    /// @return resulting event ID
    static uint64_t date_to_event(uint64_t event_base, int month, int day)
    {
        HASSERT(month >= 1 && month <= 12);
        HASSERT(day >= 1 && day <= 31);

        return event_base + DATE_EVENT_BASE_SUFFIX +
               (month << EVENT_MONTH_SHIFT) + (day << EVENT_DAY_SHIFT);
    }

    /// Build an event from year.
    /// @param event_base base event ID of the event pool
    /// @param year (0AD to 4095AD)
    /// @return resulting event ID
    static uint64_t year_to_event(uint64_t event_base, int year)
    {
        HASSERT(year >= 0 && year <= 4095);

        return event_base + YEAR_EVENT_BASE_SUFFIX + (year << EVENT_YEAR_SHIFT);
    }

    /// Build an event from rate.
    /// @param event_base base event ID of the event pool
    /// @param year (0AD to 4095AD)
    /// @return resulting event ID
    static uint64_t rate_to_event(uint64_t event_base, int16_t rate)
    {
        HASSERT(rate >= -2048 && rate < 2048);

        union Rate
        {
            uint16_t rate_;
            int16_t srate_;
        };

        Rate r;
        r.srate_ = rate;

        return event_base + RATE_EVENT_BASE_SUFFIX +
               ((r.rate_ & EVENT_RATE_MASK) << EVENT_RATE_SHIFT);
    }
};

}  // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMEDEFS_HXX_

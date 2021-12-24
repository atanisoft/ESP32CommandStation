/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef FASTCLOCK_MEMORY_CONFIG_SPACE_HXX_
#define FASTCLOCK_MEMORY_CONFIG_SPACE_HXX_

#include <FastClockConfigurationGroup.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <StringUtils.hxx>
#include <FastClockConfigurationGroup.hxx>

#include "NvsManagerStruct.hxx"
#include "sdkconfig.h"

namespace esp32cs
{
    /// Node configuration holder
    static constexpr FastClockConfiguration FastClockConfigHolder(CONFIG_OLCB_FASTCLOCK_MEMORY_SPACE_OFFSET);

    /// Virtual memory space that allows reconfiguration of the persistent node
    /// identifier.
    class FastClockMemoryConfigSpace : public openlcb::VirtualMemorySpace
    {
    public:
        /// Constructor.
        ///
        /// @param stack is the @ref SimpleStackBase that this memory space
        /// should be registered with.
        /// @param node_id is the current node identifier.
        FastClockMemoryConfigSpace(openlcb::SimpleStackBase *stack,
            node_config_t *config) : config_(config)
        {
            #define REGISTER_NUMBER(field, type, name) \
                LOG(ALWAYS, "[FastClockMemCfg:%02x] register_numeric(%s): %d", SPACE, name, field.offset()); \
                register_numeric(field, numeric_reader<type>(field.offset()), \
                                numeric_writer<type>(field.offset()))
            #define REGISTER_STRING(field, name) \
                LOG(ALWAYS, "[FastClockMemCfg:%02x] register_string(%s): %d", SPACE, name, field.offset()); \
                register_string(field, string_reader(field.offset()), \
                                string_writer(field.offset()))

            REGISTER_NUMBER(FastClockConfigHolder.enabled(), uint8_t, "enabled");
            REGISTER_STRING(FastClockConfigHolder.id(), "id");
            REGISTER_NUMBER(FastClockConfigHolder.year(), uint16_t, "year");
            REGISTER_NUMBER(FastClockConfigHolder.month(), uint8_t, "month");
            REGISTER_NUMBER(FastClockConfigHolder.day(), uint8_t, "day");
            REGISTER_NUMBER(FastClockConfigHolder.hour(), uint8_t, "hour");
            REGISTER_NUMBER(FastClockConfigHolder.minute(), uint8_t, "minute");
            REGISTER_NUMBER(FastClockConfigHolder.rate(), int16_t, "rate");
            stack->memory_config_handler()->registry()->insert(
                stack->node(), SPACE, this);
            #undef REGISTER_NUMBER
            #undef REGISTER_STRING
            LOG(INFO, "[FastClockMemCfg:%02x] registered", SPACE);
        }

        /// @return true if the configuration has been changed via this virtual
        /// memory space, false otherwise.
        bool updated()
        {
            return updated_;
        }
    private:
        node_config_t *config_;
        bool updated_{false};
        static constexpr uint8_t SPACE = CONFIG_OLCB_FASTCLOCK_MEMORY_SPACE_ID;
        template <typename T>
        typename std::function<T(unsigned repeat, BarrierNotifiable *done)>
        numeric_reader(int offset)
        {
            return [this, offset](unsigned repeat, BarrierNotifiable *done)
            {
                LOG(INFO, "[FastClockMemCfg:%02x-RD] offs: %d", SPACE, offset);
                T value = (T)0;
                switch(offset)
                {
                    case FastClockConfigHolder.enabled().offset():
                        value = config_->fastclock_enabled;
                        break;
                    case FastClockConfigHolder.year().offset():
                        value = config_->fastclock_year + 1900;
                        break;
                    case FastClockConfigHolder.month().offset():
                        value = config_->fastclock_month;
                        break;
                    case FastClockConfigHolder.day().offset():
                        value = config_->fastclock_day;
                        break;
                    case FastClockConfigHolder.hour().offset():
                        value = config_->fastclock_hour;
                        break;
                    case FastClockConfigHolder.minute().offset():
                        value = config_->fastclock_minute;
                        break;
                    case FastClockConfigHolder.rate().offset():
                        value = config_->fastclock_rate;
                        break;
                    default:
                        LOG_ERROR("[FastClockMemCfg:%02x-RD] request for unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
                return value;
            };
        }
        template <typename T>
        std::function<void(unsigned repeat, T value, BarrierNotifiable *done)>
        numeric_writer(int offset)
        {
            return [this, offset](unsigned repeat, T value, BarrierNotifiable *done)
            {
                LOG(INFO, "[FastClockMemCfg:%02x-WR] offs: %d", SPACE, offset);
                switch(offset)
                {
                    case FastClockConfigHolder.enabled().offset():
                        config_->fastclock_enabled = value;
                        break;
                    case FastClockConfigHolder.year().offset():
                        config_->fastclock_year = value - 1900;
                        break;
                    case FastClockConfigHolder.month().offset():
                        config_->fastclock_month = value;
                        break;
                    case FastClockConfigHolder.day().offset():
                        config_->fastclock_day = value;
                        break;
                    case FastClockConfigHolder.hour().offset():
                        config_->fastclock_hour = value;
                        break;
                    case FastClockConfigHolder.minute().offset():
                        config_->fastclock_minute = value;
                        break;
                    case FastClockConfigHolder.rate().offset():
                        config_->fastclock_rate = value;
                        break;
                    default:
                        LOG_ERROR("[FastClockMemCfg:%02x-WR] request for unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
            };
        }
        std::function<void(unsigned repeat, string *value, BarrierNotifiable *done)>
        string_reader(int offset)
        {
            return [this, offset](unsigned repeat, string *value, BarrierNotifiable *done)
            {
                LOG(INFO, "[FastClockMemCfg:%02x-RDSTR] offs: %d", SPACE, offset);
                switch (offset)
                {
                    case FastClockConfigHolder.id().offset():
                        *value = node_id_to_string(config_->fastclock_id);
                        break;
                    default:
                        LOG_ERROR("[FastClockMemCfg:%02x-RDSTR] request for unrecognized offset:%d", SPACE, offset);
                        *value = "";
                }
                done->notify();
            };
        }
        std::function<void(unsigned repeat, string value, BarrierNotifiable *done)>
        string_writer(int offset)
        {
            return [this, offset](unsigned repeat, string value, BarrierNotifiable *done)
            {
                // strip off nulls (if found)
                value.erase(
                    std::remove(value.begin(), value.end(), '\0'), value.end());
                LOG(INFO, "[FastClockMemCfg:%02x-WRSTR] offs: %d, value:%s",
                    SPACE, offset, value.c_str());
                switch(offset)
                {
                    case FastClockConfigHolder.id().offset():
                        config_->fastclock_id = string_to_uint64(value);
                        break;
                    default:
                        LOG_ERROR("[FastClockMemCfg:%02x-WDSTR] request for unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
            };
        }
    };
} // namespace esp32cs

#endif // FASTCLOCK_MEMORY_CONFIG_SPACE_HXX_

/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#ifndef REALTIME_CLOCK_MEMORY_CONFIG_SPACE_HXX_
#define REALTIME_CLOCK_MEMORY_CONFIG_SPACE_HXX_

#include <RealTimeClockConfigurationGroup.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/StringUtils.hxx>

#include "NvsManagerStruct.hxx"
#include "sdkconfig.h"
#include "AbstractVirtualMemorySpace.hxx"

namespace esp32cs
{
    /// Node configuration holder
    static constexpr RealTimeClockConfiguration RealTimeClockConfigHolder(CONFIG_OLCB_REALTIMECLOCK_MEMORY_SPACE_OFFSET);

    /// Virtual memory space that allows reconfiguration of the persistent node
    /// identifier.
    class RealTimeClockMemoryConfigSpace : public openlcb::VirtualMemorySpace,
                                           public AbstractVirtualMemorySpace
    {
    public:
        /// Constructor.
        ///
        /// @param stack is the @ref SimpleStackBase that this memory space
        /// should be registered with.
        /// @param node_id is the current node identifier.
        RealTimeClockMemoryConfigSpace(openlcb::SimpleStackBase *stack,
            node_config_t *config) : config_(config)
        {
            #define REGISTER_NUMBER(field, type, name)                         \
                LOG(VERBOSE,                                                   \
                    "[RealTimeClockMemCfg:%02x] register_numeric(%s): %d",     \
                    SPACE, name, field.offset());                              \
                register_numeric(field, numeric_reader<type>(field.offset()),  \
                                numeric_writer<type>(field.offset()))
            #define REGISTER_STRING(field, name)                               \
                LOG(VERBOSE,                                                   \
                    "[RealTimeClockMemCfg:%02x] register_string(%s): %d",      \
                    SPACE, name, field.offset());                              \
                register_string(field, string_reader(field.offset()),          \
                                string_writer(field.offset()))

            REGISTER_NUMBER(RealTimeClockConfigHolder.enabled(), uint8_t,
                            "enabled");
            REGISTER_STRING(RealTimeClockConfigHolder.id(), "id");
            stack->memory_config_handler()->registry()->insert(
                stack->node(), SPACE, this);
            #undef REGISTER_NUMBER
            #undef REGISTER_STRING
            LOG(INFO, "[RealTimeClockMemCfg:%02x] registered", SPACE);

        }

    private:
        node_config_t *config_;
        static constexpr uint8_t SPACE =
            CONFIG_OLCB_REALTIMECLOCK_MEMORY_SPACE_ID;
        template <typename T>
        typename std::function<T(unsigned repeat, BarrierNotifiable *done)>
        numeric_reader(int offset)
        {
            return [this, offset](unsigned repeat, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                LOG(VERBOSE, "[RealTimeClockMemCfg:%02x-RD] offs: %d", SPACE,
                    offset);
                T value = (T)0;
                switch(offset)
                {
                    case RealTimeClockConfigHolder.enabled().offset():
                        value = config_->fastclock_enabled;
                        break;
                    default:
                        LOG_ERROR("[RealTimeClockMemCfg:%02x-RD] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
                return value;
            };
        }
        template <typename T>
        std::function<void(unsigned repeat, T value, BarrierNotifiable *done)>
        numeric_writer(int offset)
        {
            return [this, offset](unsigned repeat, T value, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                LOG(VERBOSE, "[RealTimeClockMemCfg:%02x-WR] offs: %d", SPACE,
                    offset);
                switch(offset)
                {
                    case RealTimeClockConfigHolder.enabled().offset():
                        config_->fastclock_enabled = value;
                        set_modified(true);
                        break;
                    default:
                        LOG_ERROR("[RealTimeClockMemCfg:%02x-WR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
            };
        }
        std::function<void(unsigned repeat, string *value, BarrierNotifiable *done)>
        string_reader(int offset)
        {
            return [this, offset](unsigned repeat, string *value, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                LOG(VERBOSE, "[RealTimeClockMemCfg:%02x-RDSTR] offs: %d",
                    SPACE, offset);
                switch (offset)
                {
                    case RealTimeClockConfigHolder.id().offset():
                        *value = utils::node_id_to_string(config_->fastclock_id);
                        break;
                    default:
                        LOG_ERROR("[RealTimeClockMemCfg:%02x-WRSTR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                        *value = "";
                }
            };
        }
        std::function<void(unsigned repeat, string value, BarrierNotifiable *done)>
        string_writer(int offset)
        {
            return [this, offset](unsigned repeat, string value, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                // strip off nulls (if found)
                value.erase(
                    std::remove(value.begin(), value.end(), '\0'), value.end());
                LOG(VERBOSE,
                    "[RealTimeClockMemCfg:%02x-WRSTR] offs: %d, value:%s",
                    SPACE, offset, value.c_str());
                switch(offset)
                {
                    case RealTimeClockConfigHolder.id().offset():
                        config_->fastclock_id = utils::string_to_uint64(value);
                        set_modified(true);
                        break;
                    default:
                        LOG_ERROR("[RealTimeClockMemCfg:%02x-WSTR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
            };
        }
    };
} // namespace esp32cs

#endif // REALTIME_CLOCK_MEMORY_CONFIG_SPACE_HXX_

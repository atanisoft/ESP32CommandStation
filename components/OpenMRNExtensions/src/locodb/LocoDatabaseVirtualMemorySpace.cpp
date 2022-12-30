/*
 * SPDX-FileCopyrightText: 2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabase.hxx"
#include "locodb/LocoDatabaseVirtualMemorySpace.hxx"

#include <executor/Notifiable.hxx>
#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/constants.hxx>

namespace locodb
{

#ifndef LOCODB_VMS_LOGGING
#ifdef CONFIG_LOCODB_VMS_LOGGING
#define LOCODB_VMS_LOGGING CONFIG_LOCODB_VMS_LOGGING
#else
#define LOCODB_VMS_LOGGING VERBOSE
#endif // CONFIG_LOCODB_VMS_LOGGING
#endif // LOCODB_VMS_LOGGING

using openlcb::SimpleStackBase;
using openlcb::VirtualMemorySpace;

/// Static configuration holder used in this virtual memory space.
static constexpr TrainDatabaseSegment cfg_holder(TrainDatabaseSegment::group_opts().offset());

LocoDatabaseVirtualMemorySpace::LocoDatabaseVirtualMemorySpace(SimpleStackBase *stack)
{
    LOG(LOCODB_VMS_LOGGING, "[TrainDbVirtualMemorySpace:%02x] Registering",
        TrainDatabaseSegment::group_opts().segment());

    auto functions = cfg_holder.functions().all_functions();

    register_numeric(cfg_holder.index(),
                     numeric_reader<uint32_t>("index", cfg_holder.index().offset()),
                     numeric_writer<uint32_t>("index", cfg_holder.index().offset()));
    register_numeric(cfg_holder.max_index(),
                     numeric_reader<uint32_t>("max_index", cfg_holder.max_index().offset()),
                     numeric_writer<uint32_t>("max_index", cfg_holder.max_index().offset()));
    register_numeric(cfg_holder.address(),
                     numeric_reader<uint16_t>("address", cfg_holder.address().offset()),
                     numeric_writer<uint16_t>("address", cfg_holder.address().offset()));
    register_numeric(cfg_holder.mode(),
                     numeric_reader<uint8_t>("mode", cfg_holder.mode().offset()),
                     numeric_writer<uint8_t>("mode", cfg_holder.mode().offset()));
    register_numeric(functions.entry<0>().icon(),
                     numeric_reader<uint8_t>("fn_icon", functions.entry<0>().icon().offset()),
                     numeric_writer<uint8_t>("fn_icon", functions.entry<0>().icon().offset()));
    register_numeric(functions.entry<0>().is_momentary(),
                     numeric_reader<uint8_t>("fn_momentary", functions.entry<0>().is_momentary().offset()),
                     numeric_writer<uint8_t>("fn_momentary", functions.entry<0>().is_momentary().offset()));
    register_repeat(functions);
    register_string(cfg_holder.name(),
                    string_reader("name", cfg_holder.name().offset()),
                    string_writer("name", cfg_holder.name().offset()));
    register_string(cfg_holder.description(),
                    string_reader("description", cfg_holder.description().offset()),
                    string_writer("description", cfg_holder.description().offset()));
    stack->memory_config_handler()->registry()->insert(
        stack->node(), TrainDatabaseSegment::group_opts().segment(), this);
}

template <typename T>
typename std::function<T(unsigned, BarrierNotifiable *)>
LocoDatabaseVirtualMemorySpace::numeric_reader(const char *name, int offset)
{
    LOG(LOCODB_VMS_LOGGING,
        "[TrainDbVirtualMemorySpace:%02x] Registering READ offs %d for %s",
        TrainDatabaseSegment::group_opts().segment(), offset, name);
    return [this, offset](unsigned repeat, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        switch (offset)
        {
            case cfg_holder.index().offset():
                return (T)index_;
            case cfg_holder.max_index().offset():
                return (T)Singleton<LocoDatabase>::instance()->size();
            case cfg_holder.address().offset():
                if (entry_)
                {
                    return (T)entry_->get_legacy_address();
                }
                break;
            case cfg_holder.mode().offset():
                if (entry_)
                {
                    return (T)entry_->get_legacy_drive_mode();
                }
                break;
            case cfg_holder.functions().all_functions().entry<0>().icon().offset():
                if (entry_)
                {
                    return (T)entry_->get_function_def(repeat + 1);
                }
                break;
            case cfg_holder.functions().all_functions().entry<0>().is_momentary().offset():
                if (entry_)
                {
                    return (T)entry_->is_function_momentary(repeat + 1);
                }
                break;
        }
        return (T)0;
    };
}

template <typename T>
std::function<void(unsigned, T, BarrierNotifiable *)>
LocoDatabaseVirtualMemorySpace::numeric_writer(const char *name, int offset)
{
    LOG(LOCODB_VMS_LOGGING,
        "[TrainDbVirtualMemorySpace:%02x] Registering WRITE offs %d for %s",
        TrainDatabaseSegment::group_opts().segment(), offset, name);

    return [this, offset](unsigned repeat, T value, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        switch(offset)
        {
        case cfg_holder.index().offset():
            index_ = value;
            entry_ = Singleton<LocoDatabase>::instance()->get_entry(index_);
            break;
        case cfg_holder.address().offset():
            if (entry_)
            {
                entry_->set_legacy_address(value);
            }
            break;
        case cfg_holder.mode().offset():
            if (entry_)
            {
                entry_->set_legacy_drive_mode(static_cast<DriveMode>(value));
            }
            break;
        case cfg_holder.functions().all_functions().entry<0>().icon().offset():
            if (entry_)
            {
                entry_->set_function_def(repeat + 1, static_cast<Function>(value));
            }
            break;
        case cfg_holder.functions().all_functions().entry<0>().is_momentary().offset():
            if (entry_)
            {
                uint8_t label = entry_->get_function_def(repeat + 1);
                if (label != Function::UNKNOWN &&
                    label != Function::UNINITIALIZED &&
                    label != Function::MOMENTARY)
                {
                    if (value)
                    {
                        label |= Function::MOMENTARY;
                    }
                    else
                    {
                        label &= ~Function::MOMENTARY;
                    }
                    entry_->set_function_def(repeat + 1, static_cast<Function>(label));
                }
            }
            break;
        }
    };
}

std::function<void(unsigned, string *, BarrierNotifiable *)>
LocoDatabaseVirtualMemorySpace::string_reader(const char *name, int offset)
{
    LOG(LOCODB_VMS_LOGGING,
        "[TrainDbVirtualMemorySpace:%02x] Registering READ offs %d for %s",
        TrainDatabaseSegment::group_opts().segment(), offset, name);

    return [this, offset](unsigned repeat, string *value, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        switch (offset)
        {
        case cfg_holder.name().offset():
            if (entry_)
            {
                *value = entry_->get_train_name();
            }
            else
            {
                *value = "unavailable";
            }
            break;
        case cfg_holder.description().offset():
            if (entry_)
            {
                *value = entry_->get_train_description();
            }
            else
            {
                *value = "unavailable";
            }
            break;
        }
    };
}

std::function<void(unsigned, string, BarrierNotifiable *)>
LocoDatabaseVirtualMemorySpace::string_writer(const char *name, int offset)
{
    LOG(LOCODB_VMS_LOGGING,
        "[TrainDbVirtualMemorySpace:%02x] Registering WRITE offs %d for %s",
        TrainDatabaseSegment::group_opts().segment(), offset, name);

    return [this, offset](unsigned repeat, string value, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        // strip off nulls (if found)
        value.erase(
            std::remove(value.begin(), value.end(), '\0'), value.end());
        switch(offset)
        {
            case cfg_holder.name().offset():
                if (entry_)
                {
                    entry_->set_train_name(value);
                }
                break;
            case cfg_holder.description().offset():
                if (entry_)
                {
                    entry_->set_train_description(value);
                }
                break;
        }
  };
}

} // namespace locodb

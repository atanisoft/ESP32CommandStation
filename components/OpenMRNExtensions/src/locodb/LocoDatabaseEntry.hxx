/*
 * SPDX-FileCopyrightText: 2014 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "locodb/Defs.hxx"
#include <openlcb/Defs.hxx>

#ifndef _LOCODB_LOCODATABASEENTRY_HXX_
#define _LOCODB_LOCODATABASEENTRY_HXX_

#include <utils/Destructable.hxx>
#include <utils/StringPrintf.hxx>
#include <vector>

#ifndef LOCODB_LOG_LEVEL
#define LOCODB_LOG_LEVEL VERBOSE
#endif

namespace locodb
{

/// This class represents a single entry in the @ref LocoDatabase.
class LocoDatabaseEntry : public Destructable
{
public:
    /// Retrieves the NMRAnet NodeID for the virtual node that represents a
    /// particular train known to the database.
    virtual openlcb::NodeID get_traction_node() = 0;

    /// Returns the largest valid function ID for this train, or -1 if the train
    /// has no functions.
    virtual int get_max_fn() = 0;

    /// If non-negative, represents a file offset in the openlcb CONFIG_FILENAME
    /// file where this train has its data stored.
    virtual ssize_t file_offset()
    {
        return -1;
    }

    /// Returns an internal identifier that uniquely defines where this traindb
    /// entry was allocated from.
    virtual std::string identifier()
    {
        dcc::TrainAddressType addrType =
            drive_mode_to_address_type(mode_, address_);
        if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS ||
            addrType == dcc::TrainAddressType::DCC_LONG_ADDRESS)
        {
            string prefix = "long_address";
            if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS)
            {
                prefix = "short_address";
            }
            if ((mode_ & DCC_SS_MASK) == 1)
            {
                return StringPrintf("dcc_14/%s/%d", prefix.c_str(), address_);
            }
            else if ((mode_ & DCC_SS_MASK) == 2)
            {
                return StringPrintf("dcc_28/%s/%d", prefix.c_str(), address_);
            }
            else
            {
                return StringPrintf("dcc_128/%s/%d", prefix.c_str(), address_);
            }
        }
        else if (addrType == dcc::TrainAddressType::MM)
        {
            return StringPrintf("marklin/%d", address_);
        }
        return StringPrintf("unknown/%d", address_);
    }

    /// Retrieves the name of the train.
    std::string get_train_name()
    {
        return name_;
    }

    /// Sets the name of the train.
    void set_train_name(const std::string &name)
    {
        if (name_.compare(name))
        {
            if (name.length() > MAX_TRAIN_NAME_LEN)
            {
                LOG(WARNING,
                    "[Train:%d] Truncating name: %s -> %s", address_,
                    name.c_str(), name.substr(0, MAX_TRAIN_NAME_LEN).c_str());
                name_ = std::move(name.substr(0, MAX_TRAIN_NAME_LEN).c_str());
            }
            else
            {
                name_ = std::move(name);
            }
            LOG(LOCODB_LOG_LEVEL, "[Train:%d] Setting name:%s", address_,
                name_.c_str());
            modified_ = true;
        }
    }

    /// Retrieves the description of the train.
    std::string get_train_description()
    {
        return description_;
    }

    /// Sets the description of the train.
    void set_train_description(const std::string &description)
    {
        if (description_.compare(description))
        {
            if (description.length() > MAX_TRAIN_DESC_LEN)
            {
                LOG(WARNING,
                    "[Train:%d] Truncating description: %s -> %s", address_,
                    description.c_str(),
                    description.substr(0, MAX_TRAIN_DESC_LEN).c_str());
                description_ =
                    std::move(description.substr(0, MAX_TRAIN_DESC_LEN));
            }
            else
            {
                description_ = std::move(description);
            }

            LOG(LOCODB_LOG_LEVEL, "[Train:%d] Setting description:%s",
                address_, description.c_str());
            modified_ = true;
        }
    }

    /// Retrieves the legacy address of the train.
    uint16_t get_legacy_address()
    {
        return address_;
    }

    /// Sets the legacy address of the train.
    void set_legacy_address(const uint16_t address)
    {
        if (address_ != address)
        {
            LOG(LOCODB_LOG_LEVEL, "[Train:%d] Updating address to:%d",
                address_, address_);
            address_ = address;
            modified_ = true;
        }
    }

    /// Retrieves the traction drive mode of the train.
    DriveMode get_legacy_drive_mode()
    {
        return mode_;
    }

    /// Sets the traction drive mode of the train.
    void set_legacy_drive_mode(const DriveMode mode)
    {
        if (mode_ != mode)
        {
            LOG(LOCODB_LOG_LEVEL, "[Train:%d] Updating drive mode to:%d",
                address_, mode);
            mode_ = mode;
            modified_ = true;
        }
    }

    /// Retrieves the definition assigned to a given function, or
    /// NONEXISTANT if the function does not exist.
    Function get_function_def(size_t id)
    {
        // if the function id is larger than our max list reject it
        if (id > functions_.size())
        {
            return Function::NONEXISTANT;
        }
        // return the mapping for the function
        return functions_[id];
    }

    /// Sets the definition assigned to a given function.
    void set_function_def(const size_t id, const Function type)
    {
        if (functions_[id] != type)
        {
            if (type != Function::NONEXISTANT)
            {
                const char* label = function_to_string(type);
                LOG(LOCODB_LOG_LEVEL,
                    "[Train:%d] Configuring fn:%d as %s (%d)", address_, id,
                    label, type);
            }
            else
            {
                LOG(LOCODB_LOG_LEVEL,
                    "[Train:%d] Marking fn:%d as unavailable", address_, id);
            }
            functions_[id] = type;
            modified_ = true;
        }
    }

    /// Returns true if the provided function id maps to a valid function
    /// definition. A valid function definition is one that exists and has been
    /// defined.
    bool is_function_valid(const size_t fn_id)
    {
        Function type = get_function_def(fn_id);
        return type != Function::NONEXISTANT &&
               type != Function::UNINITIALIZED;
    }

    /// Returns true if the provided function id is defined as momentary.
    bool is_function_momentary(const size_t fn_id)
    {
        return (get_function_def(fn_id) & Function::MOMENTARY);
    }

    /// Notifies that we are going to read all functions. Sometimes a
    /// re-initialization is helpful at this point.
    virtual void start_read_functions()
    {
    }

    /// Sets this locomotive automatic idle state.
    ///
    /// When enabled this locomotive may be serviced by the command station
    /// update loop.
    void set_automatic_idle(bool idle)
    {
        if (idle_ != idle)
        {
            LOG(LOCODB_LOG_LEVEL,
                "[Train:%d] Setting automatic idle: %s", address_,
                idle ? "On" : "Off");
            idle_ = idle;
            modified_ = true;
        }
    }

    /// Returns true if this locomotive has automatic idle enabled.
    bool is_automatic_idle()
    {
        return idle_;
    }

    /// Returns true if this locomotive has been modified and may need to be
    /// persisted.
    virtual bool needs_persist()
    {
        return modified_;
    }

protected:
    /// Maximum length of the train name, limit is determined by SNIP field
    /// length with one space for null terminator.
    static constexpr uint8_t MAX_TRAIN_NAME_LEN = 62;

    /// Maximum length of the train description, limit is determined by SNIP
    /// field length with one space for null terminator.
    static constexpr uint8_t MAX_TRAIN_DESC_LEN = 63;

    /// Constructor.
    ///
    /// @param name is the name to use for the locomotive in the OpenLCB node
    /// and SNIP responses.
    /// @param description is the description to use for the locomotive in the
    /// OpenLCB node and SNIP responses.
    /// @param address is the legacy address for the locomotive.
    /// @param mode is the legacy @ref DriveMode (or protocol) for the
    /// locomotive.
    /// @param idle is used to enable automatic idle of the locomotive upon
    /// startup. Any locomotive that has this enabled will be created and
    /// may be serviced by the command station update loop.
    LocoDatabaseEntry(std::string name, std::string description,
                      uint16_t address, DriveMode mode, bool idle = false)
        : name_(name), description_(description), address_(address),
        mode_(mode), idle_(idle)
    {
        
    }

    /// Resets the modified flag.
    ///
    /// This is intended for usage by the @ref LocoDatabase implementation as
    /// part of the persistence mechanism.
    void set_modified(bool value)
    {
        modified_ = value;
    }

    /// Name of this locomotive.
    /// Used as part of the OpenLCB Node and SNIP response.
    std::string name_;

    /// Description of this locomotive.
    /// Used as part of the OpenLCB Node and SNIP response.
    std::string description_;

    /// Legacy address to use for this locomotive.
    uint16_t address_;

    /// Legacy drive mode (protocol) to use for this locomotive.
    DriveMode mode_{DriveMode::DEFAULT};

    /// Automatic idle flag for this locomotive.
    bool idle_{false};
    
    /// Tracking if this locomotive has been modified.
    bool modified_{false};

    /// Collection of functions defined for this locomotive.
    std::vector<Function> functions_;
};

} // namespace locodb

#endif // _LOCODB_LOCODATABASEENTRY_HXX_
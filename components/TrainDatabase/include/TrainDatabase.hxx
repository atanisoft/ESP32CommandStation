/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#ifndef TRAINDB_IMPL_HXX_
#define TRAINDB_IMPL_HXX_

#include "sdkconfig.h"

#include <algorithm>
#include <AutoPersistCallbackFlow.h>
#include <mutex>
#include <openlcb/Defs.hxx>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <openlcb/TractionTrain.hxx>
#include <os/OS.hxx>
#include <locodb/LocoDatabase.hxx>
#include <utils/Uninitialized.hxx>
#include <vector>

#ifndef CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS
#define CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS false
#endif

namespace openlcb
{
  class SimpleStackBase;
}

namespace esp32cs
{
  class Esp32TrainDatabase;
  
  class Esp32TrainDbEntry : public locodb::LocoDatabaseEntry
  {
  public:
    Esp32TrainDbEntry(Esp32TrainDatabase *db, uint16_t address,
                      locodb::DriveMode mode,
                      std::vector<locodb::Function> functions,
                      std::string name = "unknown",
                      std::string description = "unknown",
                      bool auto_idle = CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS,
                      bool modified = true);

    openlcb::NodeID get_traction_node() override;

    ssize_t file_offset() override;

    std::string to_json(bool readable = true);

    int get_max_fn() override
    {
      return maxFn_;
    }

    void start_read_functions() override
    {
      // calculate the max function based on the first occurrence of
      // FN_NONEXISTANT and if not found default to the size of the functions
      // definition vector.
      auto e = std::find_if(functions_.begin(), functions_.end()
                          , [](const uint8_t &e)
      {
        return e == locodb::Function::NONEXISTANT;
      });

      if (e != functions_.end())
      {
        maxFn_ = std::distance(functions_.begin(), e);
      }
      else
      {
        maxFn_ =  functions_.size();
      }
    }

  private:
    Esp32TrainDatabase *db_;
    int maxFn_;
    bool persistable_{true};

    friend class Esp32TrainDatabase;
    void reset_persist_flag()
    {
      set_modified(false);
    }

    bool is_persistable()
    {
      return persistable_;
    }

    void set_persistable_flag(bool value)
    {
      persistable_ = value;
    }
  };

  class Esp32TrainDatabase : public locodb::LocoDatabase
  {
  public:
    Esp32TrainDatabase(openlcb::SimpleStackBase *stack, Service *service);

    void stop()
    {
      persistFlow_->stop();
    }

    // number of known trains
    size_t size() override
    {
      return trains_.size();
    }

    int get_index(size_t address);

    bool is_valid_train(size_t train_id) override
    {
      return get_entry(train_id) != nullptr;
    }

    bool is_valid_train(openlcb::NodeID train_id) override;

    ssize_t get_entry_offset(openlcb::NodeID train_id) override
    {
      auto entry = get_entry(train_id);
      if (entry)
      {
        return entry->file_offset();
      }
      return -1;
    }

    std::shared_ptr<locodb::LocoDatabaseEntry> create_or_update(
      uint16_t address, std::string name = "unknown",
      string desciption = "unknown",
      locodb::DriveMode mode = locodb::DriveMode::DCC_128,
      bool idle = false);

    void remove_entry(size_t train_id);
    void remove_entry(uint16_t address, locodb::DriveMode mode);

    std::shared_ptr<locodb::LocoDatabaseEntry> get_entry(const std::string &name) override;

    std::shared_ptr<locodb::LocoDatabaseEntry> get_entry(size_t train_id) override;

    std::shared_ptr<locodb::LocoDatabaseEntry> get_entry(
      openlcb::NodeID traction_node_id, unsigned hint = 0) override;

    std::shared_ptr<locodb::LocoDatabaseEntry> get_entry(uint16_t address)
    {
      return get_entry(0, address);
    }

    size_t create_entry(uint16_t address, locodb::DriveMode mode) override;
    void set_train_name(uint16_t address, std::string name);
    void set_train_description(uint16_t address, std::string description);
    void set_train_auto_idle(uint16_t address, bool idle);
    void set_train_function_label(uint16_t address, uint8_t fn_id, locodb::Function fndef);
    void set_train_drive_mode(uint16_t address, locodb::DriveMode mode);

    std::string to_json();
    std::string to_json(uint16_t address, bool readable = true);

    void persist();

  private:
    std::string to_json_locked(uint16_t address, bool readable = true);
    openlcb::SimpleStackBase *stack_;
    bool entryDeleted_{false};
    OSMutex mux_;
    std::vector<std::shared_ptr<Esp32TrainDbEntry>> trains_;
    uninitialized<AutoPersistFlow> persistFlow_;
  };

} // namespace esp32cs

#endif // TRAINDB_IMPL_HXX_

/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

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

#ifndef _ESP32_TRAIN_DB_H_
#define _ESP32_TRAIN_DB_H_

#include <mutex>
#include <vector>

#include <openlcb/Defs.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <openlcb/TractionTrain.hxx>
#include <os/OS.hxx>

#include <TrainDb.hxx>

#include <AutoPersistCallbackFlow.h>

#include "sdkconfig.h"

#ifndef CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS
#define CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS false
#endif

namespace openlcb
{
  class SimpleStackBase;
}

namespace esp32cs
{
  using namespace commandstation;

  struct Esp32PersistentTrainData
  {
    uint16_t address;
    std::string name;
    bool automatic_idle;
    bool show_on_limited_throttles;
    uint8_t mode;
    std::vector<uint8_t> functions;
    Esp32PersistentTrainData()
    {
    }
    Esp32PersistentTrainData(uint16_t address, std::string name="unknown"
                           , DccMode mode=DccMode::DCC_128)
    {
      this->address = address;
      this->name = name;
      this->mode = mode;
      this->automatic_idle = CONFIG_ROSTER_AUTO_IDLE_NEW_LOCOS;
      this->show_on_limited_throttles = false;
      // set some defaults
      if (this->mode & DccMode::DCC_ANY)
      {
        this->functions.push_back(Symbols::LIGHT);
        this->functions.push_back(Symbols::BELL);
        this->functions.push_back(Symbols::HORN);
        while (this->functions.size() < DCC_MAX_FN)
        {
          this->functions.push_back(Symbols::FN_UNKNOWN);
        }
      }
      else if (this->mode & DccMode::MARKLIN_ANY)
      {
        this->functions.push_back(Symbols::LIGHT);
        this->functions.push_back(Symbols::FN_UNKNOWN);
        this->functions.push_back(Symbols::FN_UNKNOWN);
        this->functions.push_back(Symbols::ABV);
        if (this->mode & DccMode::MARKLIN_TWOADDR)
        {
          this->functions.push_back(Symbols::FN_UNKNOWN);
          this->functions.push_back(Symbols::FN_UNKNOWN);
          this->functions.push_back(Symbols::FN_UNKNOWN);
          this->functions.push_back(Symbols::FN_UNKNOWN);
        }
      }
    }
  };

  class Esp32TrainDbEntry : public commandstation::TrainDbEntry
  {
  public:
    Esp32TrainDbEntry(Esp32PersistentTrainData, bool persist=true);

    std::string identifier() override;

    openlcb::NodeID get_traction_node() override;

    std::string get_train_name() override
    {
      return data_.name;
    }

    void set_train_name(std::string name)
    {
      data_.name = std::move(name);
      dirty_ = true;
    }

    int get_legacy_address() override
    {
      return data_.address;
    }

    void set_legacy_address(int address)
    {
      data_.address = address;
      dirty_ = true;
    }

    DccMode get_legacy_drive_mode() override
    {
      return (DccMode)data_.mode;
    }

    void set_legacy_drive_mode(DccMode mode)
    {
      data_.mode = mode;
      dirty_ = true;
    }

    unsigned get_function_label(unsigned fn_id) override;

    void set_function_label(unsigned fn_id, Symbols label)
    {
      data_.functions[fn_id] = label;
      dirty_ = true;
      recalcuate_max_fn();
    }

    int get_max_fn() override
    {
      return maxFn_;
    }

    int file_offset() override;

    void start_read_functions() override
    {
    }

    Esp32PersistentTrainData get_data()
    {
      return data_;
    }

    void set_auto_idle(bool idle)
    {
      data_.automatic_idle = idle;
      dirty_ = true;
    }

    void set_show_on_limited_throttles(bool show)
    {
      data_.show_on_limited_throttles = show;
      dirty_ = true;
    }

    bool is_dirty()
    {
      return dirty_;
    }

    void reset_dirty(bool dirty=false)
    {
      dirty_ = dirty;
    }

    bool is_persisted()
    {
      return persist_;
    }

    bool is_auto_idle()
    {
      return data_.automatic_idle;
    }

    bool is_show_on_limited_throttles()
    {
      return data_.show_on_limited_throttles;
    }

  private:
    void recalcuate_max_fn();
    Esp32PersistentTrainData data_;
    uint8_t maxFn_;
    bool dirty_;
    bool persist_;
  };

  class Esp32TrainDatabase : public commandstation::TrainDb
                           , public Singleton<Esp32TrainDatabase>
  {
  public:
    Esp32TrainDatabase(openlcb::SimpleStackBase *stack);

    void stop()
    {
      persistFlow_->stop();
    }

    // number of known trains
    size_t size() override
    {
      return knownTrains_.size();
    }

    int get_index(unsigned address);

    bool is_train_id_known(unsigned train_id) override
    {
      return get_entry(train_id) != nullptr;
    }

    bool is_train_id_known(openlcb::NodeID train_id) override;

    std::shared_ptr<commandstation::TrainDbEntry> create_if_not_found(
      unsigned address, std::string name="unknown"
    , DccMode mode=DccMode::DCC_128);

    void delete_entry(unsigned address);

    std::shared_ptr<commandstation::TrainDbEntry> get_entry(unsigned train_id) override;

    std::shared_ptr<commandstation::TrainDbEntry> find_entry(
      openlcb::NodeID traction_node_id, unsigned hint = 0) override;

    unsigned add_dynamic_entry(uint16_t address, DccMode mode) override;

    std::set<uint16_t> get_default_train_addresses(uint16_t limit);

    void set_train_name(unsigned address, std::string name);
    void set_train_auto_idle(unsigned address, bool idle);
    void set_train_show_on_limited_throttle(unsigned address, bool show);
    void set_train_function_label(unsigned address, uint8_t fn_id, Symbols label);
    void set_train_drive_mode(unsigned address, DccMode mode);

    std::string get_all_entries_as_json();
    std::string get_entry_as_json(unsigned address);

    openlcb::MemorySpace *get_train_cdi()
    {
      return trainCdiFile_.get();
    }

    openlcb::MemorySpace *get_temp_train_cdi()
    {
      return tempTrainCdiFile_.get();
    }

    void persist();

  private:
    std::string get_entry_as_json_locked(unsigned address);
    openlcb::SimpleStackBase *stack_;
    bool entryDeleted_{false};
    std::mutex knownTrainsLock_;
    std::vector<std::shared_ptr<Esp32TrainDbEntry>> knownTrains_;
    std::unique_ptr<openlcb::MemorySpace> trainCdiFile_;
    std::unique_ptr<openlcb::MemorySpace> tempTrainCdiFile_;
    uninitialized<AutoPersistFlow> persistFlow_;
  };

} // namespace esp32cs

#endif // _ESP32_TRAIN_DB_H_

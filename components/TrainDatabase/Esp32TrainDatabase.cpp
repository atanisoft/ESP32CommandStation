/*
 * SPDX-FileCopyrightText: 2019-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "TrainDatabase.hxx"

#include <algorithm>
#include <cJSON.h>
#include <locomgr/LocoManager.hxx>
#include <openlcb/SimpleStack.hxx>
#include <utils/FileUtils.hxx>
#include <utils/StringUtils.hxx>
#include <utils/logging.h>

namespace esp32cs
{

using locodb::DriveMode;
using locodb::Function;
using locodb::LocoDatabaseEntry;
using openlcb::TractionDefs;

static constexpr const char * TRAIN_DB_JSON_FILE = "/fs/trains.json";

Esp32TrainDatabase::Esp32TrainDatabase(openlcb::SimpleStackBase *stack,
                                       Service *service)
{
  struct stat statbuf;
  if (!stat(TRAIN_DB_JSON_FILE, &statbuf))
  {
    LOG(INFO, "[TrainDB] Loading %s...", TRAIN_DB_JSON_FILE);
    auto roster = read_file_to_string(TRAIN_DB_JSON_FILE);
    cJSON *root = cJSON_ParseWithLength(roster.c_str(), roster.length());
    if (cJSON_IsArray(root))
    {
      cJSON *entry;
      cJSON_ArrayForEach(entry, root)
      {
        cJSON *mode = cJSON_GetObjectItem(entry, "mode");
        cJSON *functions = cJSON_GetObjectItem(entry, "fn");
        uint16_t address = cJSON_GetObjectItem(entry, "addr")->valueint;
        std::string name = cJSON_GetObjectItem(entry, "name")->valuestring;
        std::string desc = cJSON_GetObjectItem(entry, "desc")->valuestring;
        DriveMode drive_mode =
          static_cast<DriveMode>(cJSON_GetObjectItem(mode, "type")->valueint);
        bool idle = cJSON_IsTrue(cJSON_GetObjectItem(entry, "idle"));
        std::vector<Function> fns;
        if (cJSON_IsArray(functions))
        { 
          cJSON *function;
          cJSON_ArrayForEach(function, functions)
          {
            uint8_t id = cJSON_GetObjectItem(function, "id")->valueint;
            Function type =
              static_cast<Function>(
                cJSON_GetObjectItem(function, "type")->valueint);
            LOG(CONFIG_ROSTER_LOG_LEVEL,
                "[TrainDB:%d] function: %d -> %d", address, id, type);
            fns[id] = type;
          }
        }
        auto train =
          std::make_shared<Esp32TrainDbEntry>(this, address, drive_mode,
                                              fns, name, desc, idle,
                                              false /* modified */);
        LOG(CONFIG_ROSTER_LOG_LEVEL,
            "[TrainDB-%zu] Registering %s, name:%s, desc:%s, idle:%s",
            trains_.size(), train->identifier().c_str(),
            train->get_train_name().c_str(),
            train->get_train_description().c_str(),
            train->is_automatic_idle() ? "On" : "Off");
        trains_.emplace_back(train);
      }
    }
    else
    {
      LOG_ERROR("[TrainDB] Persistent storage is corrupt and will not be loaded!");
    }
    cJSON_Delete(root);
  }
  else
  {
    LOG(WARNING, "[TrainDB] %s does not exist, skipping loading.",
        TRAIN_DB_JSON_FILE);
  }

  LOG(INFO, "[TrainDB] Found %d persistent roster entries.", trains_.size());

  persistFlow_.emplace(service,
                       SEC_TO_NSEC(CONFIG_ROSTER_PERSISTENCE_INTERVAL_SEC),
                       std::bind(&Esp32TrainDatabase::persist, this));
}

#define FIND_TRAIN(id)                                    \
  std::find_if(trains_.begin(), trains_.end(),            \
    [id](const auto &train)                               \
    {                                                     \
      return train->get_legacy_address() == id;           \
    })

// Case insensitive matching by name
#define FIND_TRAIN_BY_NAME(name)                                \
  std::find_if(trains_.begin(), trains_.end(),                  \
    [name](const auto &train)                                   \
    {                                                           \
      auto entry_name = train->get_train_name();                \
      if (entry_name.size() != name.size())                     \
      {                                                         \
        return false;                                           \
      }                                                         \
      return std::equal(entry_name.begin(), entry_name.end(),   \
                        name.begin(), name.end(),               \
                        [](unsigned char a, unsigned char b) {  \
                          return tolower(a) == tolower(b);      \
                        });                                     \
    })

#define FIND_TRAIN_HINT(id, id2)                          \
  std::find_if(trains_.begin(), trains_.end(),            \
    [id,id2](const auto &train)                           \
    {                                                     \
      return train->get_traction_node() == id ||          \
             train->get_legacy_address() == id2;          \
    })

std::shared_ptr<LocoDatabaseEntry> Esp32TrainDatabase::create_or_update(
  uint16_t address, string name, string description, DriveMode mode, bool idle)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] Searching for roster entry for address: %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
#if CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
    LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Found existing entry:%s.",
        (*entry)->identifier().c_str());
#endif // CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
    (*entry)->set_train_name(name);
    (*entry)->set_train_description(description);
    (*entry)->set_legacy_drive_mode(mode);
    (*entry)->set_automatic_idle(idle);
    return *entry;
  }
  auto index = trains_.size();

  std::vector<Function> functions;
  trains_.emplace_back(
    new Esp32TrainDbEntry(this, address, mode, functions, name, description, idle));
#if CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] No entry was found, created new entry:%s.",
      trains_[index]->identifier().c_str());
#endif // CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  return trains_[index];
}

int Esp32TrainDatabase::get_index(size_t address)
{
  auto ent = FIND_TRAIN(address);
  if (ent != trains_.end())
  {
    return std::distance(trains_.begin(), ent);
  }

  return -1;
}

bool Esp32TrainDatabase::is_valid_train(openlcb::NodeID train_id)
{
#if CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] searching for train with id: %s",
      utils::node_id_to_string(train_id).c_str());
#endif // CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  dcc::TrainAddressType type;
  uint32_t addr =  0;
  // verify that it is a valid train node id
  if (TractionDefs::legacy_address_from_train_node_id(train_id, &type, &addr))
  {
    // only search with the address and discard the drive type (for now)
    auto ent = FIND_TRAIN(addr);
    return ent != trains_.end();
  }
  return false;
}

void Esp32TrainDatabase::remove_entry(size_t train_id)
{
  uint16_t addr = 0;
  DriveMode mode = DriveMode::DEFAULT;
  {
    OSMutexLock lock(&mux_);
    if (train_id < trains_.size())
    {
      addr = trains_[train_id]->get_legacy_address();
      mode = trains_[train_id]->get_legacy_drive_mode();
    }
    else
    {
      return;
    }
  }
  remove_entry(addr, mode);
}

void Esp32TrainDatabase::remove_entry(uint16_t address, DriveMode mode)
{
  OSMutexLock lock(&mux_);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    LOG(CONFIG_ROSTER_LOG_LEVEL,
        "[TrainDB] Removing persistent entry for address %u", address);
    trains_.erase(entry);

    entryDeleted_ = true;

    // Send the request to cleanup the train node to the train node manager via
    // it's executor. This may occur in the background after this method exits.
    Singleton<locomgr::LocoManager>::instance()->train_service()->executor()->add(
      new CallbackExecutable([mode, address]()
      {
        Singleton<locomgr::LocoManager>::instance()->delete_train(mode, address);
      }
    ));
  }
}

std::shared_ptr<LocoDatabaseEntry> Esp32TrainDatabase::get_entry(size_t train_id)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] get_entry(%u) : %zu", train_id,
      trains_.size());
  if (train_id < trains_.size())
  {
    return trains_[train_id];
  }
  // check if the train_id is a locomotive address that we know of
  auto entry = FIND_TRAIN(train_id);
  if (entry != trains_.end())
  {
    return *entry;
  }
  return nullptr;
}

std::shared_ptr<LocoDatabaseEntry> Esp32TrainDatabase::get_entry(const string &name)
{
  OSMutexLock lock(&mux_);
  auto entry = FIND_TRAIN_BY_NAME(name);
  if (entry != trains_.end())
  {
    return *entry;
  }
  return nullptr;
}

std::shared_ptr<LocoDatabaseEntry> Esp32TrainDatabase::get_entry(
  openlcb::NodeID node_id, unsigned hint)
{
  OSMutexLock lock(&mux_);
#if CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] Searching for Train Node:%s, Hint:%u",
      utils::node_id_to_string(node_id).c_str(), hint);
#endif // CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
  auto entry = FIND_TRAIN_HINT(node_id, hint);
  if (entry != trains_.end())
  {
#if CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
    LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Found existing entry: %s.",
        (*entry)->identifier().c_str());
#endif // CONFIG_ROSTER_LOG_LEVEL >= VERBOSE
    return *entry;
  }
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] No entry found!");
  return nullptr;
}

// The caller of this method expects a zero based index into the vector.
size_t Esp32TrainDatabase::create_entry(uint16_t address, DriveMode mode)
{
  OSMutexLock lock(&mux_);
  size_t index = 0;
  std::string name = std::to_string(address);

  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Searching for loco %d", address);

  // prevent duplicate entries in the roster
  auto ent = FIND_TRAIN(address);
  if (ent != trains_.end())
  {
    index = std::distance(trains_.begin(), ent);
    LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Found existing entry (%zu)",
        index);
  }
  else
  {
    // track the index for the new train entry
    index = trains_.size();
    std::vector<Function> functions;

#ifdef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    LOG(CONFIG_ROSTER_LOG_LEVEL,
        "[TrainDB] Creating persistent roster entry for locomotive %d.",
        address);
#else
    LOG(INFO, "[TrainDB] Adding temporary roster entry for locomotive %d.",
        address);
#endif
    trains_.emplace_back(
      new Esp32TrainDbEntry(this, address, mode, functions, name, name));
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
  trains_.back()->set_persistable_flag(false);
#endif
  }
  return index;
}

void Esp32TrainDatabase::set_train_name(uint16_t address, std::string name)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Searching for train with address %d",
      address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    (*entry)->set_train_name(name);
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    (*entry)->set_persistable_flag(false);
#endif
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set the name!", address);
  }
}

void Esp32TrainDatabase::set_train_description(uint16_t address, std::string description)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    (*entry)->set_train_description(description);
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    (*entry)->set_persistable_flag(false);
#endif
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set the name!", address);
  }
}

void Esp32TrainDatabase::set_train_auto_idle(uint16_t address, bool idle)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    (*entry)->set_automatic_idle(idle);
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    (*entry)->set_persistable_flag(false);
#endif
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set idle state!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_function_label(uint16_t address, uint8_t fn_id, Function fndef)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Searching for train with address %u",
      address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    (*entry)->set_function_def(fn_id, fndef);
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    (*entry)->set_persistable_flag(false);
#endif
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set function definition!",
              address);
  }
}

void Esp32TrainDatabase::set_train_drive_mode(uint16_t address, DriveMode mode)
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] Searching for train with address %u",
      address);
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    (*entry)->set_legacy_drive_mode(mode);
#ifndef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    (*entry)->set_persistable_flag(false);
#endif
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set drive mode!"
            , address);
  }
}

string Esp32TrainDatabase::to_json()
{
  OSMutexLock lock(&mux_);
  string res = "[";
  for (auto entry : trains_)
  {
    if (res.length() > 1)
    {
      res += ",";
    }
    res += entry->to_json();
  }
  res += "]";
  return res;
}

string Esp32TrainDatabase::to_json(uint16_t address, bool readable)
{
  OSMutexLock lock(&mux_);
  return to_json_locked(address, readable);
}

string Esp32TrainDatabase::to_json_locked(uint16_t address, bool readable)
{
  std::string serialized = "{}";
  auto entry = FIND_TRAIN(address);
  if (entry != trains_.end())
  {
    serialized = (*entry)->to_json(readable);
  }
  return serialized;
}

void Esp32TrainDatabase::persist()
{
  OSMutexLock lock(&mux_);
  LOG(CONFIG_ROSTER_LOG_LEVEL,
      "[TrainDB] Checking if roster needs to be persisted...");
  auto ent = std::find_if(trains_.begin(), trains_.end(),
    [](const auto &train)
    {
      return train->needs_persist();
    });
  if (ent != trains_.end() || entryDeleted_)
  {
    LOG(CONFIG_ROSTER_LOG_LEVEL,
        "[TrainDB] At least one entry requires persistence.");
    std::string serialized = "[";
    size_t count = 0;
    for (auto entry : trains_)
    {
      if (entry->is_persistable())
      {
        if (count)
        {
          serialized += ",";
        }
        serialized += entry->to_json(false);
        count++;
      }
      entry->reset_persist_flag();
    }
    serialized += "]";
    write_string_to_file(TRAIN_DB_JSON_FILE, serialized);
    LOG(INFO, "[TrainDB] Persisted %zu roster entries.", count);
    entryDeleted_ = false;
  }
  else
  {
    LOG(CONFIG_ROSTER_LOG_LEVEL, "[TrainDB] No entries require persistence");
  }
}

} // namespace esp32cs

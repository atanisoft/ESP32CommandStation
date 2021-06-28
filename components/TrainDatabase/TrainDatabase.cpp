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

#include "TrainDatabase.h"

#include <AllTrainNodes.hxx>
#include <CDIXMLGenerator.hxx>
#include <cJSON.h>
#include <TrainDbCdi.hxx>
#include <openlcb/SimpleStack.hxx>
#include <StringUtils.hxx>
#include <utils/FileUtils.hxx>

namespace esp32cs
{

using commandstation::DccMode;
using commandstation::AllTrainNodes;
using openlcb::TractionDefs;

// This should really be defined inside TractionDefs.hxx and used by the call
// to TractionDefs::train_node_id_from_legacy().
static constexpr uint64_t const OLCB_NODE_ID_USER = 0x050101010000ULL;

Esp32TrainDbEntry::Esp32TrainDbEntry(Esp32PersistentTrainData data,
                                     Esp32TrainDatabase *db, bool persist)
  : data_(data), db_(db), dirty_(true), persist_(persist)
{
  recalcuate_max_fn();
  LOG(VERBOSE, "[Loco:%s] Locomotive '%s' created", identifier().c_str()
    , data_.name.c_str());
}

string Esp32TrainDbEntry::identifier()
{
  dcc::TrainAddressType addrType =
    dcc_mode_to_address_type((DccMode)data_.mode, data_.address);
  if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS ||
      addrType == dcc::TrainAddressType::DCC_LONG_ADDRESS)
  {
    string prefix = "long_address";
    if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS)
    {
      prefix = "short_address";
    }
    if ((data_.mode & DCC_SS_MASK) == 1)
    {
      return StringPrintf("dcc_14/%s/%d", prefix.c_str(), data_.address);
    }
    else if ((data_.mode & DCC_SS_MASK) == 2)
    {
      return StringPrintf("dcc_28/%s/%d", prefix.c_str(), data_.address);
    }
    else
    {
      return StringPrintf("dcc_128/%s/%d", prefix.c_str(), data_.address);
    }
  }
  return StringPrintf("unknown/%d", data_.address);
}

openlcb::NodeID Esp32TrainDbEntry::get_traction_node()
{
  if (data_.mode == DCCMODE_OLCBUSER)
  {
    return OLCB_NODE_ID_USER | static_cast<openlcb::NodeID>(data_.address);
  }
  else
  {
    return openlcb::TractionDefs::train_node_id_from_legacy(
        dcc_mode_to_address_type((DccMode)data_.mode, data_.address), data_.address);
  }
}

unsigned Esp32TrainDbEntry::get_function_label(unsigned fn_id)
{
  // if the function id is larger than our max list reject it
  if (fn_id > maxFn_)
  {
    return FN_NONEXISTANT;
  }
  // return the mapping for the function
  return data_.functions[fn_id];
}

int Esp32TrainDbEntry::file_offset()
{
  if (persist_)
  {
    return db_->get_index(data_.address);
  }
  // non-persistent entries should not have an offset
  return -1;
}

void Esp32TrainDbEntry::recalcuate_max_fn()
{
  // recalculate the maxFn_ based on the first occurrence of FN_NONEXISTANT
  // and if not found default to the size of the functions labels vector.
  auto e = std::find_if(data_.functions.begin(), data_.functions.end()
                      , [](const uint8_t &e)
  {
    return e == FN_NONEXISTANT;
  });

  if (e != data_.functions.end())
  {
    maxFn_ = std::distance(data_.functions.begin(), e);
  }
  else
  {
    maxFn_ = data_.functions.size();
  }
}

std::string Esp32TrainDbEntry::to_json(bool readable)
{
  string json = R"!^!({"addr":)!^!";
  json += integer_to_string(data_.address);
  json += R"!^!(,"name":")!^!";
  json += data_.name;
  json += R"!^!(","desc":")!^!";
  json += data_.description;
  json += R"!^!(","idle":)!^!";
  if (data_.automatic_idle)
  {
    json += R"!^!(true,)!^!";
  }
  else
  {
    json += R"!^!(false,)!^!";
  }
  json += R"!^!("mode":{"type":)!^!";
  json += integer_to_string(data_.mode);
  if (readable)
  {
    switch (data_.mode)
    {
      case DCCMODE_OLCBUSER:
        json += R"!^!(,"name":"DCC-OlcbUser")!^!";
        break;
      case DCC_DEFAULT:
        json += R"!^!(,"name":"DCC (auto speed step)")!^!";
        break;
      case DCC_14:
        json += R"!^!(,"name":"DCC (14 speed step)")!^!";
        break;
      case DCC_14_LONG_ADDRESS:
        json += R"!^!(,"name":"DCC (14 speed step, long address)")!^!";
        break;
      case DCC_28:
        json += R"!^!(,"name":"DCC (28 speed step)")!^!";
        break;
      case DCC_28_LONG_ADDRESS:
        json += R"!^!(,"name":"DCC (28 speed step, long address)")!^!";
        break;
      case DCC_128:
        json += R"!^!(,"name":"DCC (128 speed step)")!^!";
        break;
      case DCC_128_LONG_ADDRESS:
        json += R"!^!(,"name":"DCC (128 speed step, long address)")!^!";
        break;
      case DCCMODE_DEFAULT:
      default:
        json += R"!^!(,"name":"DCC (default)")!^!";
        break;
    }
  }
  json += R"!^!(},"fn":[)!^!";
  for (size_t idx = 0; idx < DCC_MAX_FN; idx++)
  {
    if (idx > 0)
    {
      json += ",";
    }
    json += R"!^!({"id":)!^!";
    json += integer_to_string(idx);
    json += R"!^!(,"type":)!^!";
    json += integer_to_string(data_.functions[idx]);
    if (readable)
    {
      switch (data_.functions[idx])
      {
        case FN_NONEXISTANT:
          json += R"!^!(,"name":"N/A")!^!";
          break;
        case LIGHT:
          json += R"!^!(,"name":"Light")!^!";
          break;
        case HORN:
          json += R"!^!(,"name":"Horn")!^!";
          break;
        case BELL:
          json += R"!^!(,"name":"Bell")!^!";
          break;
        case WHISTLE:
          json += R"!^!(,"name":"Whistle")!^!";
          break;
        case SHUNT:
          json += R"!^!(,"name":"Shunting mode")!^!";
          break;
        case MOMENTUM:
          json += R"!^!(,"name":"Momentum")!^!";
          break;
        case MUTE:
          json += R"!^!(,"name":"Mute")!^!";
          break;
        case GENERIC:
          json += R"!^!(,"name":"Function")!^!";
          break;
        case COUPLER:
          json += R"!^!(,"name":"Coupler")!^!";
          break;
        case FN_UNKNOWN:
        default:
          json += R"!^!(,"name":"Unknown")!^!";
          break;
      }
    }
    json += R"!^!(})!^!";
  }
  json += R"!^!(]})!^!";
  json.shrink_to_fit();
  return json;
}

static constexpr const char * TRAIN_DB_JSON_FILE = "/fs/trains.json";
static constexpr const char * PERSISTED_TRAIN_CDI = "/fs/train.xml";
static constexpr const char * TEMP_TRAIN_CDI = "/fs/tmptrain.xml";

Esp32TrainDatabase::Esp32TrainDatabase(openlcb::SimpleStackBase *stack,
                                       Service *service)
{
  LOG(INFO, "[TrainDB] Refreshing train CDI files...");
  commandstation::TrainConfigDef train_cfg(0);
  commandstation::TrainTmpConfigDef temp_train_cfg(0);
  CDIXMLGenerator::create_config_descriptor_xml(train_cfg, PERSISTED_TRAIN_CDI, nullptr);
  CDIXMLGenerator::create_config_descriptor_xml(temp_train_cfg, TEMP_TRAIN_CDI, nullptr);
  trainCdiFile_.emplace(PERSISTED_TRAIN_CDI);
  tempTrainCdiFile_.emplace(TEMP_TRAIN_CDI);
  struct stat statbuf;
  if (!stat(TRAIN_DB_JSON_FILE, &statbuf))
  {
    LOG(INFO, "[TrainDB] Loading database...");
    auto roster = read_file_to_string(TRAIN_DB_JSON_FILE);
    cJSON *root = cJSON_ParseWithLength(roster.c_str(), roster.length());
    if (!cJSON_IsArray(root))
    {
      LOG_ERROR("[TrainDB] database is corrupt, no trains loaded!");
      unlink(TRAIN_DB_JSON_FILE);
    }
    else
    {
      cJSON *entry;
      cJSON_ArrayForEach(entry, root)
      {
        if (cJSON_HasObjectItem(entry, "name") &&
            cJSON_HasObjectItem(entry, "desc") &&
            cJSON_HasObjectItem(entry, "addr") && 
            cJSON_HasObjectItem(entry, "idle") &&
            cJSON_HasObjectItem(entry, "mode"))
        {
          cJSON *mode = cJSON_GetObjectItem(entry, "mode");
          Esp32PersistentTrainData data(
            cJSON_GetObjectItem(entry, "addr")->valueint,
            cJSON_GetObjectItem(entry, "name")->valuestring,
            cJSON_GetObjectItem(entry, "desc")->valuestring,
            (DccMode)cJSON_GetObjectItem(mode, "type")->valueint,
            cJSON_IsTrue(cJSON_GetObjectItem(entry, "idle")));
          if (cJSON_HasObjectItem(entry, "fn") &&
              cJSON_IsArray(cJSON_GetObjectItem(entry, "fn")))
          { 
            cJSON *function;
            cJSON_ArrayForEach(function, cJSON_GetObjectItem(entry, "fn"))
            {
              if (cJSON_HasObjectItem(function, "id") &&
                  cJSON_HasObjectItem(function, "type"))
              {
                uint8_t id = cJSON_GetObjectItem(function, "id")->valueint;
                uint8_t type = cJSON_GetObjectItem(function, "type")->valueint;
                LOG(VERBOSE, "[Train: %d] function: %d -> %d", data.address,
                    id, type);
                data.functions[id] = type;
              }
              else
              {
                LOG_ERROR("[TrainDB] Loco %d (%s) has corrupt function "
                          "entries which will be ignored.",
                          data.address, data.name.c_str());
              }
            }
          }
          auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
          [data](const auto &train)
          {
            return train->get_legacy_address() == data.address;
          });
          if (ent == knownTrains_.end())
          {
            LOG(INFO, "[TrainDB] Registering %u - %s (auto-idle: %s)",
                data.address, data.name.c_str(),
                data.automatic_idle ? "On" : "Off");
            auto train = new Esp32TrainDbEntry(data, this);
            train->reset_dirty();
            if (train->is_auto_idle())
            {
              uint16_t address = train->get_legacy_address();
              stack->executor()->add(new CallbackExecutable([address]()
              {
                auto trainMgr = Singleton<AllTrainNodes>::instance();
                trainMgr->allocate_node(DccMode::DCC_128, address);
              }));
            }
            knownTrains_.emplace_back(train);
          }
          else
          {
            LOG_ERROR("[TrainDB] Duplicate roster entry detected for loco addr %u."
                    , data.address);
          }
        }
      }
    }
  }

  LOG(INFO, "[TrainDB] Found %d persistent roster entries."
    , knownTrains_.size());

  persistFlow_.emplace(service,
                       SEC_TO_NSEC(CONFIG_ROSTER_PERSISTENCE_INTERVAL_SEC),
                       std::bind(&Esp32TrainDatabase::persist, this));
}

#define FIND_TRAIN(id)                                    \
  std::find_if(knownTrains_.begin(), knownTrains_.end(),  \
    [id](const auto &train)                               \
    {                                                     \
      return train->get_legacy_address() == id;           \
    })

#define FIND_TRAIN_HINT(id, id2)                          \
  std::find_if(knownTrains_.begin(), knownTrains_.end(),  \
    [id,id2](const auto &train)                           \
    {                                                     \
      return train->get_traction_node() == id ||          \
             train->get_legacy_address() == id2;          \
    })

std::shared_ptr<TrainDbEntry> Esp32TrainDatabase::create_or_update(
  unsigned address, string name, string description, DccMode mode, bool idle)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for roster entry for address: %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Found existing entry:%s.", (*entry)->identifier().c_str());
    (*entry)->set_train_name(name);
    (*entry)->set_train_description(description);
    (*entry)->set_legacy_drive_mode(mode);
    (*entry)->set_auto_idle(idle);
    return *entry;
  }
  auto index = knownTrains_.size();
  knownTrains_.emplace_back(
    new Esp32TrainDbEntry(
      Esp32PersistentTrainData(address, name, description, mode, idle), this));
  LOG(INFO, "[TrainDB] No entry was found, created new entry:%s.",
      knownTrains_[index]->identifier().c_str());
  return knownTrains_[index];
}

int Esp32TrainDatabase::get_index(unsigned address)
{
  auto ent = FIND_TRAIN(address);
  if (ent != knownTrains_.end())
  {
    return std::distance(knownTrains_.begin(), ent);
  }

  return -1;
}

bool Esp32TrainDatabase::is_train_id_known(openlcb::NodeID train_id)
{
  LOG(INFO, "[TrainDB] searching for train with id: %s",
      esp32cs::node_id_to_string(train_id).c_str());
  dcc::TrainAddressType type;
  uint32_t addr =  0;
  // verify that it is a valid train node id
  if (TractionDefs::legacy_address_from_train_node_id(train_id, &type, &addr))
  {
    // only search with the address and discard the drive type (for now)
    auto ent = FIND_TRAIN(addr);
    if (ent != knownTrains_.end())
    {
      LOG(VERBOSE, "[TrainDB] %s", (*ent)->identifier().c_str());
    }
    return ent != knownTrains_.end();
  }
  return false;
}

void Esp32TrainDatabase::delete_entry(unsigned address)
{
  OSMutexLock lock(&mux_);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Removing persistent entry for address %u", address);
    knownTrains_.erase(entry);
    entryDeleted_ = true;
  }
}

std::shared_ptr<TrainDbEntry> Esp32TrainDatabase::get_entry(unsigned train_id)
{
  OSMutexLock lock(&mux_);
  LOG(VERBOSE, "[TrainDB] get_entry(%u) : %zu", train_id, knownTrains_.size());
  if (train_id < knownTrains_.size())
  {
    return knownTrains_[train_id];
  }
  // check if the train_id is a locomotive address that we know of
  auto entry = FIND_TRAIN(train_id);
  if (entry != knownTrains_.end())
  {
    return *entry;
  }
  return nullptr;
}

std::shared_ptr<TrainDbEntry> Esp32TrainDatabase::find_entry(openlcb::NodeID node_id
                                                           , unsigned hint)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for Train Node:%s, Hint:%u",
      esp32cs::node_id_to_string(node_id).c_str(), hint);
  auto entry = FIND_TRAIN_HINT(node_id, hint);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Found existing entry: %s."
      , (*entry)->identifier().c_str());
    return *entry;
  }
  LOG(INFO, "[TrainDB] No entry found!");
  return nullptr;
}

// The caller of this method expects a zero based index into the vector. This
// may be changed in the future to use the loco address instead.
unsigned Esp32TrainDatabase::add_dynamic_entry(uint16_t address, DccMode mode)
{
  OSMutexLock lock(&mux_);
  size_t index = 0;

  LOG(INFO, "[TrainDB] Searching for loco %d", address);

  // prevent duplicate entries in the roster
  auto ent = FIND_TRAIN(address);
  if (ent != knownTrains_.end())
  {
    index = std::distance(knownTrains_.begin(), ent);
    LOG(INFO, "[TrainDB] Found existing entry (%zu)", index);
  }
  else
  {
    // track the index for the new train entry
    index = knownTrains_.size();

#ifdef CONFIG_ROSTER_AUTO_CREATE_ENTRIES
    LOG(INFO
      , "[TrainDB] Creating persistent roster entry for locomotive %u."
      , address);

    // create the new entry, it will default to being marked dirty so it will
    // automatically persist.
    knownTrains_.emplace_back(
      new Esp32TrainDbEntry(
        Esp32PersistentTrainData(address, std::to_string(address),
                                 std::to_string(address), mode), this));
#else
    LOG(INFO
      , "[TrainDB] Adding temporary roster entry for locomotive %u."
      , address);
    // create the new entry and do not mark it as dirty so it doesn't
    // automatically persist. If the locomotive is later edited via the web UI
    // it will be marked as dirty and persisted at that point.
    knownTrains_.emplace_back(
      new Esp32TrainDbEntry(
        Esp32PersistentTrainData(address, std::to_string(address),
                                 std::to_string(address), mode), this, false));
#endif
  }
  return index;
}

void Esp32TrainDatabase::set_train_name(unsigned address, std::string name)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Setting train(%u) name: %s", address, name.c_str());
    (*entry)->set_train_name(name);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set the name!", address);
  }
}

void Esp32TrainDatabase::set_train_description(unsigned address, std::string description)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Setting train(%u) description: %s", address, description.c_str());
    (*entry)->set_train_description(description);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set the name!", address);
  }
}

void Esp32TrainDatabase::set_train_auto_idle(unsigned address, bool idle)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Setting auto-idle: %s", idle ? "On" : "Off");
    (*entry)->set_auto_idle(idle);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set idle state!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_function_label(unsigned address, uint8_t fn_id, Symbols label)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    (*entry)->set_function_label(fn_id, label);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set function label!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_drive_mode(unsigned address, commandstation::DccMode mode)
{
  OSMutexLock lock(&mux_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    (*entry)->set_legacy_drive_mode(mode);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set drive mode!"
            , address);
  }
}

string Esp32TrainDatabase::get_all_entries_as_json()
{
  OSMutexLock lock(&mux_);
  string res = "[";
  for (auto entry : knownTrains_)
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

string Esp32TrainDatabase::get_entry_as_json(unsigned address, bool readable)
{
  OSMutexLock lock(&mux_);
  return get_entry_as_json_locked(address, readable);
}

string Esp32TrainDatabase::get_entry_as_json_locked(unsigned address, bool readable)
{
  std::string serialized = "{}";
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    serialized = (*entry)->to_json(readable);
  }
  return serialized;
}

void Esp32TrainDatabase::persist()
{
  OSMutexLock lock(&mux_);
  LOG(VERBOSE, "[TrainDB] Checking if roster needs to be persisted...");
  auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [](const auto &train)
    {
      return train->is_dirty() && train->is_persisted();
    });
  if (ent != knownTrains_.end() || entryDeleted_)
  {
    LOG(VERBOSE, "[TrainDB] At least one entry requires persistence.");
    std::string serialized = "[";
    size_t count = 0;
    for (auto entry : knownTrains_)
    {
      if (entry->is_persisted())
      {
        if (count)
        {
          serialized += ",";
        }
        serialized += entry->to_json(false);
        count++;
      }
      entry->reset_dirty();
    }
    serialized += "]";
    write_string_to_file(TRAIN_DB_JSON_FILE, serialized);
    LOG(INFO, "[TrainDB] Persisted %zu entries.", count);
    entryDeleted_ = false;
  }
  else
  {
    LOG(VERBOSE, "[TrainDB] No entries require persistence");
  }
}

} // namespace esp32cs

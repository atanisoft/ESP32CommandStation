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
#include <json.hpp>
#include <TrainDbCdi.hxx>
#include <openlcb/SimpleStack.hxx>
#include <StringUtils.hxx>
#include <utils/FileUtils.hxx>

namespace commandstation
{
// JSON serialization mappings for the commandstation::DccMode enum
NLOHMANN_JSON_SERIALIZE_ENUM(DccMode,
{
  { DCCMODE_DEFAULT,     "DCC (default)" },
  { DCCMODE_OLCBUSER,    "DCC-OlcbUser" },
  { MARKLIN_DEFAULT,     "Marklin" },
  { MARKLIN_OLD,         "Marklin (v1)" },
  { MARKLIN_NEW,         "Marklin (v2, f0-f4)" },
  { MARKLIN_TWOADDR,     "Marklin (v2, f0-f8)" },
  { MFX,                 "Marklin (MFX)" },
  { DCC_DEFAULT,         "DCC (auto speed step)"},
  { DCC_14,              "DCC (14 speed step)"},
  { DCC_28,              "DCC (28 speed step)"},
  { DCC_128,             "DCC (128 speed step)"},
  { DCC_14_LONG_ADDRESS, "DCC (14 speed step, long address)"},
  { DCC_28_LONG_ADDRESS, "DCC (28 speed step, long address)"},
  { DCC_128_LONG_ADDRESS,"DCC (128 speed step, long address)"},
});

// JSON serialization mappings for the commandstation::Symbols enum
NLOHMANN_JSON_SERIALIZE_ENUM(Symbols,
{
  { FN_NONEXISTANT,   "N/A" },
  { LIGHT,            "Light" },
  { BEAMER,           "Beamer" },
  { BELL,             "Bell" },
  { HORN,             "Horn" },
  { SHUNT,            "Shunting mode" },
  { PANTO,            "Pantograph" },
  { SMOKE,            "Smoke" },
  { ABV,              "Momentum On/Off" },
  { WHISTLE,          "Whistle" },
  { SOUND,            "Sound" },
  { FNT11,            "Generic Function" },
  { SPEECH,           "Announce" },
  { ENGINE,           "Engine" },
  { LIGHT1,           "Light1" },
  { LIGHT2,           "Light2" },
  { TELEX,            "Coupler" },
  { FN_UNKNOWN,       "Unknown" },
  { MOMENTARY,        "momentary" },
  { FNP,              "fnp" },
  { SOUNDP,           "soundp" },
  { FN_UNINITIALIZED, "uninit" },
})
}

namespace esp32cs
{

using nlohmann::json;
using commandstation::DccMode;
using commandstation::AllTrainNodes;
using openlcb::TractionDefs;

constexpr const char * JSON_NAME_NODE = "name";
constexpr const char * JSON_ID_NODE = "id";
constexpr const char * JSON_TYPE_NODE = "type";
constexpr const char * JSON_ADDRESS_NODE = "addr";
constexpr const char * JSON_IDLE_ON_STARTUP_NODE = "idle";
constexpr const char * JSON_DEFAULT_ON_THROTTLE_NODE = "default";
constexpr const char * JSON_FUNCTIONS_NODE = "functions";
constexpr const char * JSON_LOCOS_NODE = "locos";
constexpr const char * JSON_LOCO_NODE = "loco";
constexpr const char * JSON_MODE_NODE = "mode";
constexpr const char * JSON_VALUE_OFF = "Off";
constexpr const char * JSON_VALUE_ON = "On";

// This should really be defined inside TractionDefs.hxx and used by the call
// to TractionDefs::train_node_id_from_legacy().
static constexpr uint64_t const OLCB_NODE_ID_USER = 0x050101010000ULL;

// converts a Esp32PersistentTrainData to a json object
void to_json(json& j, const Esp32PersistentTrainData& t)
{
  j = json({
    { JSON_NAME_NODE, t.name },
    { JSON_ADDRESS_NODE, t.address },
    { JSON_IDLE_ON_STARTUP_NODE, t.automatic_idle },
    { JSON_DEFAULT_ON_THROTTLE_NODE, t.show_on_limited_throttles },
    { JSON_FUNCTIONS_NODE, t.functions },
    { JSON_MODE_NODE, t.mode },
  });
}

// converts a json payload to a Esp32PersistentTrainData object
void from_json(const json& j, Esp32PersistentTrainData& t)
{
  j.at(JSON_NAME_NODE).get_to(t.name);
  j.at(JSON_ADDRESS_NODE).get_to(t.address);
  j.at(JSON_IDLE_ON_STARTUP_NODE).get_to(t.automatic_idle);
  j.at(JSON_DEFAULT_ON_THROTTLE_NODE).get_to(t.show_on_limited_throttles);
  j.at(JSON_FUNCTIONS_NODE).get_to(t.functions);
  j.at(JSON_MODE_NODE).get_to(t.mode);
}

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
  else if (addrType == dcc::TrainAddressType::MM)
  {
    // TBD: should marklin types be explored further?
    return StringPrintf("marklin/%d", data_.address);
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

static constexpr const char * TRAIN_DB_JSON_FILE = "/fs/trains.json";

const char TRAIN_CDI_DATA[] = R"xmlpayload(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>github.com/atanisoft (Mike Dunston)</manufacturer>
<model>Virtual train node</model>
<hardwareVersion>)xmlpayload" SNIP_HW_VERSION R"xmlpayload(</hardwareVersion>
<softwareVersion>)xmlpayload" SNIP_SW_VERSION R"xmlpayload(</softwareVersion>
</identification>
<segment space='253'>
<group>
<description>Configures a single train</description>
<int size='2'>
<name>Address</name>
<description>Track protocol address of the train.</description>
<default>0</default>
</int>
<int size='1'>
<name>Protocol</name>
<description>Protocol to use on the track for driving this train.</description>
<default>10</default>
<map><relation><property>0</property><value>Unused</value></relation><relation><property>10</property><value>DCC 28-step</value></relation><relation><property>11</property><value>DCC 128-step</value></relation><relation><property>5</property><value>Marklin-Motorola I</value></relation><relation><property>6</property><value>Marklin-Motorola II</value></relation><relation><property>14</property><value>DCC 28-step (forced long address)</value></relation><relation><property>15</property><value>DCC 128-step (forced long address)</value></relation></map>
</int>
<string size='16'>
<name>Name</name>
<description>Identifies the train node on the LCC bus.</description>
</string>
<group>
<group>
<name>F0</name>
<description>F0 is permanently assigned to Light.</description>
<group offset='2'/>
</group>
<group replication='28'>
<name>Functions</name>
<description>Defines what each function button does.</description>
<repname>Fn</repname>
<int size='1'>
<name>Display</name>
<description>Defines how throttles display this function.</description>
<default>0</default>
<map><relation><property>0</property><value>Unavailable</value></relation><relation><property>1</property><value>Light</value></relation><relation><property>2</property><value>Beamer</value></relation><relation><property>3</property><value>Bell</value></relation><relation><property>4</property><value>Horn</value></relation><relation><property>5</property><value>Shunting mode</value></relation><relation><property>6</property><value>Pantograph</value></relation><relation><property>7</property><value>Smoke</value></relation><relation><property>8</property><value>Momentum off</value></relation><relation><property>9</property><value>Whistle</value></relation><relation><property>10</property><value>Sound</value></relation><relation><property>11</property><value>F</value></relation><relation><property>12</property><value>Announce</value></relation><relation><property>13</property><value>Engine</value></relation><relation><property>14</property><value>Light1</value></relation><relation><property>15</property><value>Light2</value></relation><relation><property>17</property><value>Uncouple</value></relation><relation><property>255</property><value>Unavailable_</value></relation></map>
</int>
<int size='1'>
<name>Momentary</name>
<description>Momentary functions are automatically turned off when you release the respective button on the throttles.</description>
<default>0</default>
<map><relation><property>0</property><value>Latching</value></relation><relation><property>1</property><value>Momentary</value></relation></map>
</int>
</group>
</group>
</group>
</segment>
<segment space='248' origin='2131755008'>
<name>Programming track operation</name>
<description>Use this component to read and write CVs on the programming track of the command station.</description>
<int size='4'>
<name>Operating mode</name>
<map>
<relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Direct mode</value></relation>
<relation><property>2</property><value>POM mode</value></relation>
<relation><property>10</property><value>Advanced mode</value></relation>
</map>
</int>
<int size='4'>
<name>CV number</name>
<description>Number of CV to read or write (1..1024).</description>
<min>0</min>
<max>1024</max>
<default>0</default>
</int>
<int size='4'>
<name>CV value</name>
<description>Set 'Operating mode' and 'CV number' first, then: hit 'Refresh' to read the entire CV, or enter a value and hit 'Write' to set the CV.</description>
<min>0</min>
<max>255</max>
<default>0</default>
</int>
<int size='4'>
<name>Bit change</name>
<description>Set 'Operating mode' and 'CV number' first, then: write 1064 to set the single bit whose value is 64, or 2064 to clear that bit. Write 100 to 107 to set bit index 0 to 7, or 200 to 207 to clear bit 0 to 7. Values outside of these two ranges do nothing.</description>
<min>100</min>
<max>2128</max>
<default>1000</default>
</int>
<string size='24'>
<name>Read bits decomposition</name>
<description>Hit Refresh on this line after reading a CV value to see which bits are set.</description>
</string>
<group>
<name>Advanced settings</name>
<int size='4'>
<name>Repeat count for verify packets</name>
<description>How many times a direct mode bit verify packet needs to be repeated for an acknowledgement to be generated.</description>
<min>0</min>
<max>255</max>
<default>3</default>
</int>
<int size='4'>
<name>Repeat count for reset packets after verify</name>
<description>How many reset packets to send after a verify.</description>
<min>0</min>
<max>255</max>
<default>6</default>
</int>
</group>
</segment>
</cdi>
)xmlpayload";
const size_t TRAIN_CDI_DATA_SIZE = sizeof(TRAIN_CDI_DATA);

const char TEMP_TRAIN_CDI_DATA[] = R"xmlpayload(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>github.com/atanisoft (Mike Dunston)</manufacturer>
<model>Virtual train node</model>
<hardwareVersion>)xmlpayload" SNIP_HW_VERSION R"xmlpayload(</hardwareVersion>
<softwareVersion>)xmlpayload" SNIP_SW_VERSION R"xmlpayload(</softwareVersion>
</identification>
<segment space='253'>
<name>Non-stored train</name>
<description>This train is not part of the train database, thus no configuration settings can be changed on it.</description>
</segment>
<segment space='248' origin='2131755008'>
<name>Programming track operation</name>
<description>Use this component to read and write CVs on the programming track of the command station.</description>
<int size='4'>
<name>Operating mode</name>
<map>
<relation><property>0</property><value>Disabled</value></relation>
<relation><property>1</property><value>Direct mode</value></relation>
<relation><property>2</property><value>POM mode</value></relation>
<relation><property>10</property><value>Advanced mode</value></relation>
</map>
</int>
<int size='4'>
<name>CV number</name>
<description>Number of CV to read or write (1..1024).</description>
<min>0</min>
<max>1024</max>
<default>0</default>
</int>
<int size='4'>
<name>CV value</name>
<description>Set 'Operating mode' and 'CV number' first, then: hit 'Refresh' to read the entire CV, or enter a value and hit 'Write' to set the CV.</description>
<min>0</min>
<max>255</max>
<default>0</default>
</int>
<int size='4'>
<name>Bit change</name>
<description>Set 'Operating mode' and 'CV number' first, then: write 1064 to set the single bit whose value is 64, or 2064 to clear that bit. Write 100 to 107 to set bit index 0 to 7, or 200 to 207 to clear bit 0 to 7. Values outside of these two ranges do nothing.</description>
<min>100</min>
<max>2128</max>
<default>1000</default>
</int>
<string size='24'>
<name>Read bits decomposition</name>
<description>Hit Refresh on this line after reading a CV value to see which bits are set.</description>
</string>
<group>
<name>Advanced settings</name>
<int size='4'>
<name>Repeat count for verify packets</name>
<description>How many times a direct mode bit verify packet needs to be repeated for an acknowledgement to be generated.</description>
<min>0</min>
<max>255</max>
<default>3</default>
</int>
<int size='4'>
<name>Repeat count for reset packets after verify</name>
<description>How many reset packets to send after a verify.</description>
<min>0</min>
<max>255</max>
<default>6</default>
</int>
</group>
</segment>
</cdi>
)xmlpayload";
const size_t TEMP_TRAIN_CDI_DATA_SIZE = sizeof(TEMP_TRAIN_CDI_DATA);

Esp32TrainDatabase::Esp32TrainDatabase(openlcb::SimpleStackBase *stack)
{
  trainCdiFile_.reset(new openlcb::ReadOnlyMemoryBlock(TRAIN_CDI_DATA, TRAIN_CDI_DATA_SIZE));
  tempTrainCdiFile_.reset(new openlcb::ReadOnlyMemoryBlock(TEMP_TRAIN_CDI_DATA, TEMP_TRAIN_CDI_DATA_SIZE));
  persistFlow_.emplace(stack->service()
                     , SEC_TO_NSEC(CONFIG_ROSTER_PERSISTENCE_INTERVAL_SEC)
                     , std::bind(&Esp32TrainDatabase::persist, this));

  LOG(INFO, "[TrainDB] Initializing...");
  struct stat statbuf;

  if (!stat(TRAIN_DB_JSON_FILE, &statbuf))
  {
    auto roster = read_file_to_string(TRAIN_DB_JSON_FILE);
    json stored_trains = json::parse(roster, nullptr, false);
    if (stored_trains.is_discarded())
    {
      LOG_ERROR("[TrainDB] database is corrupt, no trains loaded!");
      unlink(TRAIN_DB_JSON_FILE);
      return;
    }
    for (auto &entry : stored_trains)
    {
      auto data = entry.get<Esp32PersistentTrainData>();
      auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
      [data](const auto &train)
      {
        return train->get_legacy_address() == data.address;
      });
      if (ent == knownTrains_.end())
      {
        LOG(INFO, "[TrainDB] Registering %u - %s (idle: %s, limited: %s)"
          , data.address, data.name.c_str()
          , data.automatic_idle ? JSON_VALUE_ON : JSON_VALUE_OFF
          , data.show_on_limited_throttles ? JSON_VALUE_ON : JSON_VALUE_OFF);
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

  LOG(INFO, "[TrainDB] Found %d persistent roster entries."
    , knownTrains_.size());
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

std::shared_ptr<TrainDbEntry> Esp32TrainDatabase::create_if_not_found(unsigned address
                                                                    , string name
                                                                    , DccMode mode)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  LOG(INFO, "[TrainDB] Searching for roster entry for address: %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Found existing entry:%s.", (*entry)->identifier().c_str());
    return *entry;
  }
  auto index = knownTrains_.size();
  knownTrains_.emplace_back(
    new Esp32TrainDbEntry(Esp32PersistentTrainData(address, name, mode), this));
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
      LOG(INFO, "[TrainDB] %s", (*ent)->identifier().c_str());
    }
    return ent != knownTrains_.end();
  }
  return false;
}

void Esp32TrainDatabase::delete_entry(unsigned address)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
        Esp32PersistentTrainData(address, std::to_string(address), mode), this));
#else
    LOG(INFO
      , "[TrainDB] Adding temporary roster entry for locomotive %u."
      , address);
    // create the new entry and do not mark it as dirty so it doesn't
    // automatically persist. If the locomotive is later edited via the web UI
    // it will be marked as dirty and persisted at that point.
    knownTrains_.emplace_back(
      new Esp32TrainDbEntry(
        Esp32PersistentTrainData(address, std::to_string(address), mode)
      , this, false));
#endif
  }
  return index;
}

std::set<uint16_t> Esp32TrainDatabase::get_default_train_addresses(uint16_t limit)
{
  std::set<uint16_t> results;
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  for(auto entry : knownTrains_)
  {
    if (entry->get_data().show_on_limited_throttles)
    {
      if (limit)
      {
        results.insert(entry->get_legacy_address());
        limit--;
      }
      else
      {
        break;
      }
    }
  }
  return results;
}

void Esp32TrainDatabase::set_train_name(unsigned address, std::string name)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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

void Esp32TrainDatabase::set_train_auto_idle(unsigned address, bool idle)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Setting auto-idle: %s", idle ? JSON_VALUE_ON : JSON_VALUE_OFF);
    (*entry)->set_auto_idle(idle);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set idle state!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_show_on_limited_throttle(unsigned address
                                                          , bool show)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  LOG(INFO, "[TrainDB] Searching for train with address %u", address);
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    LOG(INFO, "[TrainDB] Setting visible on limited throttes: %s", show ? JSON_VALUE_ON : JSON_VALUE_OFF);
    (*entry)->set_show_on_limited_throttles(show);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set limited throttle!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_function_label(unsigned address, uint8_t fn_id, Symbols label)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
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
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  string res = "[";
  for (auto entry : knownTrains_)
  {
    if (res.length() > 1)
    {
      res += ",";
    }
    res += get_entry_as_json_locked(entry->get_legacy_address());
  }
  res += "]";
  return res;
}

string Esp32TrainDatabase::get_entry_as_json(unsigned address)
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  return get_entry_as_json_locked(address);
}

string Esp32TrainDatabase::get_entry_as_json_locked(unsigned address)
{
  auto entry = FIND_TRAIN(address);
  if (entry != knownTrains_.end())
  {
    auto train = (*entry);
    json j =
    {
      { JSON_NAME_NODE, train->get_train_name() },
      { JSON_ADDRESS_NODE, train->get_legacy_address() },
      { JSON_IDLE_ON_STARTUP_NODE, train->is_auto_idle() },
      { JSON_DEFAULT_ON_THROTTLE_NODE, train->is_show_on_limited_throttles() },
      { JSON_MODE_NODE,
        {
          { JSON_NAME_NODE, static_cast<DccMode>(train->get_legacy_drive_mode()) },
          { JSON_TYPE_NODE, train->get_legacy_drive_mode() }
        }
      }
    };
    for (size_t idx = 0; idx < DCC_MAX_FN; idx++)
    {
      j[JSON_FUNCTIONS_NODE].push_back(
      {
        { JSON_ID_NODE, idx },
        { JSON_NAME_NODE, static_cast<Symbols>(train->get_function_label(idx)) },
        { JSON_TYPE_NODE, train->get_function_label(idx) }
      });
    }
    return j.dump();
  }
  return "{}";
}

void Esp32TrainDatabase::persist()
{
  const std::lock_guard<std::mutex> lock(knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Checking if roster needs to be persisted...");
  auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [](const auto &train)
    {
      return train->is_dirty() && train->is_persisted();
    });
  if (ent != knownTrains_.end() || entryDeleted_)
  {
    LOG(VERBOSE, "[TrainDB] At least one entry requires persistence.");
    json j;
    size_t count = 0;
    for (auto entry : knownTrains_)
    {
      if (entry->is_persisted())
      {
        j.push_back(entry->get_data());
        count++;
      }
      entry->reset_dirty();
    }
    write_string_to_file(TRAIN_DB_JSON_FILE, j.dump());
    LOG(INFO, "[TrainDB] Persisted %zu entries.", count);
    entryDeleted_ = false;
  }
  else
  {
    LOG(VERBOSE, "[TrainDB] No entries require persistence");
  }
}

} // namespace esp32cs

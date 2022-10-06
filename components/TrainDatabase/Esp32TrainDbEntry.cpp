/*
 * SPDX-FileCopyrightText: 2019-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "TrainDatabase.hxx"

#include <algorithm>
#include <locodb/LocoDatabase.hxx>
#include <utils/StringPrintf.hxx>

namespace esp32cs
{

using locodb::DriveMode;
using locodb::Function;
using openlcb::NodeID;
using openlcb::TractionDefs;

Esp32TrainDbEntry::Esp32TrainDbEntry(Esp32TrainDatabase *db, uint16_t address,
                                     DriveMode mode,
                                     std::vector<Function> functions,
                                     std::string name, std::string description,
                                     bool auto_idle, bool modified)
  : LocoDatabaseEntry(name, description, address, mode, auto_idle), db_(db)
{
  set_modified(modified);
  // Set the mode to DCC-128 if the default was selected
  if (mode_ == DriveMode::DEFAULT ||
      mode_ == DriveMode::DCC_DEFAULT)
  {
    mode_ = DriveMode::DCC_128;
  }
  else if (mode_ == DriveMode::DCC_DEFAULT_LONG_ADDRESS)
  {
    mode_ = DriveMode::DCC_128_LONG_ADDRESS;
  }
  for (Function fn : functions)
  {
    functions_.push_back(fn);
  }
  if (functions_.empty())
  {
    functions_.push_back(Function::HEADLIGHT);
    functions_.push_back(Function::BELL);
    functions_.push_back(Function::HORN);
  }
  while (functions_.size() < locodb::MAX_LOCO_FUNCTIONS)
  {
    functions_.push_back(Function::UNKNOWN);
  }
}

static constexpr uint64_t const OLCB_NODE_ID_NAMESPACE   = 0x050101010000ULL;

NodeID Esp32TrainDbEntry::get_traction_node()
{
  if (mode_ == DriveMode::OLCBUSER)
  {
    return static_cast<NodeID>(OLCB_NODE_ID_NAMESPACE | address_);
  }
  else
  {
    return TractionDefs::train_node_id_from_legacy(
        drive_mode_to_address_type(mode_, address_), address_);
  }
}

ssize_t Esp32TrainDbEntry::file_offset()
{
  if (is_persistable())
  {
    return db_->get_index(address_);
  }
  return -1;
}

std::string Esp32TrainDbEntry::to_json(bool readable)
{
  string json = R"!^!({"addr":)!^!";
  json += integer_to_string(address_);
  json += R"!^!(,"name":")!^!";
  json += name_;
  json += R"!^!(","desc":")!^!";
  json += description_;
  json += R"!^!(","idle":)!^!";
  if (idle_)
  {
    json += R"!^!(true,)!^!";
  }
  else
  {
    json += R"!^!(false,)!^!";
  }
  json += R"!^!("mode":{"type":)!^!";
  json += integer_to_string(mode_);
  if (readable)
  {
    const char* label = drive_mode_to_string(mode_);
    if (label)
    {
      json += StringPrintf(R"!^!(,"name":"%s")!^!", label);
    }
    else
    {
      json += R"!^!(,"name":"DCC (default)")!^!";
    }
  }
  json += R"!^!(},"fn":[)!^!";
  size_t fn_id = 0;
  for (Function fn : functions_)
  {
    if (fn_id > 0)
    {
      json += ",";
    }
    json += R"!^!({"id":)!^!";
    json += integer_to_string(fn_id);
    json += R"!^!(,"type":)!^!";
    json += integer_to_string(fn);
    if (readable)
    {
      const char* label = locodb::function_to_string(fn);
      if (label)
      {
        json += StringPrintf(R"!^!(,"name":"%s")!^!", label);
      }
      else
      {
        json += StringPrintf(R"!^!(,"name":"f%d")!^!", fn_id);
      }
    }
    json += R"!^!(})!^!";
    fn_id++;
  }
  json += R"!^!(]})!^!";
  json.shrink_to_fit();
  return json;
}

} // namespace esp32cs

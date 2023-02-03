/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#include "TrainManager.hxx"
#include "LazyInitTrainNode.hxx"
#include "PersistentTrainConfigSpace.hxx"

#include <openlcb/Node.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <locodb/LocoDatabaseEntryCdi.hxx>
#include <utils/logging.h>

namespace trainmanager
{

using locodb::DriveMode;
using locodb::Function;
using locodb::LocoDatabase;
using locodb::TrainConfigDef;

#ifndef TRAINCONFIG_LOGLEVEL
#ifdef CONFIG_LOCOMGR_CONFIG_LOGGING
#define TRAINCONFIG_LOGLEVEL CONFIG_LOCOMGR_CONFIG_LOGGING
#else
#define TRAINCONFIG_LOGLEVEL VERBOSE
#endif // CONFIG_LOCOMGR_CONFIG_LOGGING
#endif // TRAINCONFIG_LOGLEVEL

static constexpr TrainConfigDef cfg_holder(TrainConfigDef::group_opts().offset());

PersistentTrainConfigSpace::PersistentTrainConfigSpace(TrainManager* parent) : parent_(parent)
{
  auto train = cfg_holder.train();
  auto functions = train.fn().all_functions();
  register_numeric(train.address(),
                    typed_reader<uint16_t>(train.address().offset()),
                    typed_writer<uint16_t>(train.address().offset()));
  register_numeric(train.mode(), typed_reader<uint8_t>(train.mode().offset()),
                    typed_writer<uint8_t>(train.mode().offset()));
  register_string(train.name(), string_reader(train.name().offset()),
                  string_writer(train.name().offset()));
  register_string(train.description(),
                  string_reader(train.description().offset()),
                  string_writer(train.description().offset()));
  register_numeric(functions.entry<0>().icon(),
                    typed_reader<uint8_t>(functions.entry<0>().icon().offset()),
                    typed_writer<uint8_t>(functions.entry<0>().icon().offset()));
  register_numeric(functions.entry<0>().is_momentary(),
                    typed_reader<uint8_t>(functions.entry<0>().is_momentary().offset()),
                    typed_writer<uint8_t>(functions.entry<0>().is_momentary().offset()));
  register_repeat(functions);
}

bool PersistentTrainConfigSpace::set_node(openlcb::Node* node)
{
  if (impl_ && impl_ == node)
  {
    // same node.
    return true;
  }
  impl_ = parent_->find_node(node);
  if (impl_ == nullptr)
  {
    return false;
  }
  else if (impl_->file_offset() < 0)
  {
    return false;
  }
  train_ =
    Singleton<LocoDatabase>::instance()->get_entry((size_t)impl_->file_offset());
  if (!train_)
  {
    return false;
  }
  return true;
}

template <typename T>
typename std::function<T(unsigned repeat, BarrierNotifiable *done)>
PersistentTrainConfigSpace::typed_reader(int index)
{
  return [this, index](unsigned repeat, BarrierNotifiable *done)
  {
    AutoNotify n(done);
    switch(index)
    {
      case cfg_holder.train().address().offset():
        return (T)train_->get_legacy_address();
      case cfg_holder.train().mode().offset():
        return (T)train_->get_legacy_drive_mode();
      case cfg_holder.train().fn().all_functions().entry<0>().icon().offset():
      {
        uint8_t label = train_->get_function_def(repeat + 1);
#if TRAINCONFIG_LOGLEVEL >= VERBOSE
        LOG(TRAINCONFIG_LOGLEVEL, "[FN: %d/%s] orig label: %d",
            repeat + 1, train_->identifier().c_str(), label);
#endif // TRAINCONFIG_LOGLEVEL >= VERBOSE
        if (label == Function::UNINITIALIZED || label == Function::MOMENTARY)
        {
          label = Function::UNKNOWN;
        }
        else if (label != Function::UNKNOWN)
        {
          label &= ~Function::MOMENTARY;
        }
#if TRAINCONFIG_LOGLEVEL >= VERBOSE
        LOG(TRAINCONFIG_LOGLEVEL, "[FN: %d/%s] returning label as: %d",
            repeat + 1, train_->identifier().c_str(), label);
#endif // TRAINCONFIG_LOGLEVEL >= VERBOSE
        return (T)label;
      }
      case cfg_holder.train().fn().all_functions().entry<0>().is_momentary().offset():
      {
        uint8_t label = train_->get_function_def(repeat + 1);
#if TRAINCONFIG_LOGLEVEL >= VERBOSE
        LOG(TRAINCONFIG_LOGLEVEL, "[FN: %d/%s] label: %d", repeat + 1,
            train_->identifier().c_str(), label);
#endif // TRAINCONFIG_LOGLEVEL >= VERBOSE
        if (label != Function::UNKNOWN &&
            label != Function::UNINITIALIZED &&
            label != Function::MOMENTARY)
        {
          if ((label & ~Function::MOMENTARY) != label)
          {
            return (T)1;
          }
        }
        return (T)0;
      }
    }
    return (T)0;
  };
}

std::function<void(unsigned repeat, string *contents, BarrierNotifiable *done)>
PersistentTrainConfigSpace::string_reader(int index)
{
  return [this, index](unsigned repeat, string *contents, BarrierNotifiable *done)
  {
    AutoNotify n(done);
    switch (index)
    {
      case cfg_holder.train().name().offset():
        *contents = train_->get_train_name();
        break;
      case cfg_holder.train().description().offset():
        *contents = train_->get_train_description();
        break;
    }
  };
}

template <typename T>
std::function<void(unsigned repeat, T contents, BarrierNotifiable *done)>
PersistentTrainConfigSpace::typed_writer(int index)
{
  return [this, index](unsigned repeat, T contents, BarrierNotifiable *done)
  {
    AutoNotify n(done);
    switch(index)
    {
      case cfg_holder.train().address().offset():
        train_->set_legacy_address(contents);
        break;
      case cfg_holder.train().mode().offset():
        train_->set_legacy_drive_mode(static_cast<DriveMode>(contents));
        break;
      case cfg_holder.train().fn().all_functions().entry<0>().icon().offset():
        train_->set_function_def(repeat + 1, static_cast<Function>(contents));
        break;
      case cfg_holder.train().fn().all_functions().entry<0>().is_momentary().offset():
        {
          uint8_t label = train_->get_function_def(repeat + 1);
          if (label != Function::UNKNOWN &&
              label != Function::UNINITIALIZED &&
              label != Function::MOMENTARY)
          {
            if (contents)
            {
              label |= Function::MOMENTARY;
            }
            else
            {
              label &= ~Function::MOMENTARY;
            }
            train_->set_function_def(repeat + 1, static_cast<Function>(label));
          }
        }
        break;
    }
  };
}

std::function<void(unsigned repeat, string contents, BarrierNotifiable *done)>
PersistentTrainConfigSpace::string_writer(int index)
{
  return [this, index](
    unsigned repeat, string contents, BarrierNotifiable *done)
    {
      AutoNotify n(done);
      // strip off nulls (if found)
      contents.erase(
        std::remove(contents.begin(), contents.end(), '\0'), contents.end());
      switch(index)
      {
        case cfg_holder.train().name().offset():
          train_->set_train_name(contents);
          break;
        case cfg_holder.train().description().offset():
          train_->set_train_description(contents);
          break;
      }
  };
}

} // namespace trainmanager
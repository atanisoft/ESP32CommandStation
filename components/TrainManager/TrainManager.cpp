/*
 * SPDX-FileCopyrightText: 2014 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "TrainManager.hxx"

#include "FdiXmlGenerator.hxx"
#include "LazyInitTrainNode.hxx"
#include "TrainCDISpace.hxx"
#include "TrainFDISpace.hxx"
#include "PersistentTrainConfigSpace.hxx"
#include "TrainIdentifyHandler.hxx"
#include "TrainPipHandler.hxx"
#include "TrainSnipHandler.hxx"

#include <algorithm>
#include <dcc/Loco.hxx>
#include <functional>
#include <locodb/LocoDatabaseEntryCdi.hxx>

#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/TractionDefs.hxx>
#include <openlcb/TractionTrain.hxx>
#include <trainsearch/TrainSearchProtocolServer.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/StringUtils.hxx>

namespace trainmanager
{

#ifndef TRAINMGR_LOGLEVEL
#ifdef CONFIG_TSP_LOGGING_LEVEL
#define TRAINMGR_LOGLEVEL CONFIG_TSP_LOGGING_LEVEL
#else
#define TRAINMGR_LOGLEVEL VERBOSE
#endif // TRAINMGR_LOGLEVEL
#endif // TRAINMGR_LOGLEVEL

using dcc::TrainAddressType;
using locodb::DriveMode;
using locodb::LocoDatabase;
using openlcb::MemoryConfigDefs;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::TrainImpl;
using openlcb::TractionDefs;

TrainManager::TrainManager(openlcb::TrainService* traction_service,
                           openlcb::SimpleInfoFlow* info_flow,
                           openlcb::MemoryConfigHandler* memory_config)
    : locomgr::LocoManager(traction_service),
      memoryConfigService_(memory_config),
      infoFlow_(info_flow),
      ro_train_cdi_(locodb::TRAINCONFIGDEF_CDI_DATA,
                    locodb::TRAINCONFIGDEF_CDI_SIZE + 1),
      ro_tmp_train_cdi_(locodb::TRAINTMPCONFIGDEF_CDI_DATA,
                        locodb::TRAINTMPCONFIGDEF_CDI_SIZE + 1)
{
  LOG(INFO, "[TrainCDI] Size: %zu", locodb::TRAINCONFIGDEF_CDI_SIZE);
  LOG(INFO, "[TempTrainCDI] Size: %zu", locodb::TRAINCONFIGDEF_CDI_SIZE);

  // loop through all db entries and create nodes for them
  auto traindb = Singleton<LocoDatabase>::instance();
  size_t index = 0;
  while (traindb->is_valid_train(index))
  {
    auto train = traindb->get_entry(index);
    auto train_impl =
      create_impl(train->file_offset(), train->get_legacy_drive_mode(),
                  train->get_legacy_address());
    if (!train_impl)
    {
      LOG_ERROR("[TrainManager] Failed to allocate new locomotive: %s (%s)",
                train->identifier().c_str(), train->get_train_name().c_str());
    }
    else if (train->is_automatic_idle())
    {
      train_impl->train();
    }
    index++;
  }
}

TrainManager::~TrainManager()
{
  OSMutexLock l(&trainsLock_);
  for (auto* t : trains_)
  {
    delete t;
  }
  // deregister everything via the set_enabled method
  set_enabled(false);
}

void TrainManager::delete_train(DriveMode drive_type, int address)
{
  OSMutexLock l(&trainsLock_);
  auto ent = std::find_if(trains_.begin(), trains_.end(),
    [drive_type, address](const auto &impl)
    {
      return impl->address() == address && impl->mode() == drive_type;
    });
  if (ent != trains_.end())
  {
    LazyInitTrainNode *impl = (*ent);
    impl->iface()->delete_local_node(impl);
#if TRAINMGR_LOGLEVEL >= VERBOSE
    LOG(TRAINMGR_LOGLEVEL, "[TrainManager] %s deleted as it used address:%d",
        utils::node_id_to_string(impl->node_id()).c_str(), address);
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
    delete impl;
    trains_.erase(ent);
  }
}

TrainImpl* TrainManager::find_train(NodeID node_id)
{
  auto ent = find_node(node_id, false);
  if (ent && ent->is_allocated())
  {
    return ent->train();
  }
  return nullptr;
}

TrainImpl* TrainManager::find_or_create_train(DriveMode drive_type, int address)
{
  {
    OSMutexLock l(&trainsLock_);
    auto ent = std::find_if(trains_.begin(), trains_.end(),
      [address](const auto &train)
      {
        return train->address() == address;
      });
    if (ent != trains_.end())
    {
#if TRAINMGR_LOGLEVEL >= VERBOSE
      LOG(TRAINMGR_LOGLEVEL,
          "[TrainManager] Found %s matching drive: %d with address %d",
          utils::node_id_to_string((*ent)->node_id()).c_str(),
          static_cast<int>(drive_type), address);
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
      return (*ent)->train();
    }
  }
  LOG(TRAINMGR_LOGLEVEL,
      "[TrainManager] No existing loco found for drive: %d address %d, trying "
      "to allocate", drive_type, address);
  // no active train was found with the drive type and address, attempt to
  // create a new one.
  auto impl = find_node(create_train_node(drive_type, address));
  if (impl)
  {
#if TRAINMGR_LOGLEVEL >= VERBOSE
    LOG(TRAINMGR_LOGLEVEL,
        "[TrainManager] %s created for drive: %d with address %d",
        utils::node_id_to_string(impl->node_id()).c_str(),
        static_cast<int>(drive_type), address);
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
    return impl->train();
  }
  LOG_ERROR("[TrainManager] Failed to locate/create locomotive for drive: %d "
            "with address %d, giving up",
            static_cast<int>(drive_type), address);
  return nullptr;
}

LazyInitTrainNode* TrainManager::find_node(Node* node) 
{
  {
    OSMutexLock l(&trainsLock_);
    auto ent = std::find_if(trains_.begin(), trains_.end(),
      [node](const auto &train)
      {
        return train == node;
      });
    if (ent != trains_.end())
    {
      return *ent;
    }
  }
  // no active train was found with the provided node reference, try to find
  // one via the node-id instead.
  if (node && node->node_id())
  {
    return find_node(node->node_id());
  }
  return nullptr;
}

LazyInitTrainNode* TrainManager::find_node(NodeID node_id, bool allocate)
{
  {
    OSMutexLock l(&trainsLock_);
    auto ent = std::find_if(trains_.begin(), trains_.end(),
      [node_id](const auto &train)
      {
        return train->node_id() == node_id;
      });
    if (ent != trains_.end())
    {
      return *ent;
    }
  }
  if (!allocate)
  {
    return nullptr;
  }

  // no active train was found having the provided node id, search for a train
  // in the db that should have the provided node id and return it instead if
  // found.
  TrainAddressType type = TrainAddressType::UNSUPPORTED;
  uint32_t addr =  0;
  if (TractionDefs::legacy_address_from_train_node_id(node_id, &type, &addr))
  {
    if (type == TrainAddressType::MM)
    {
      LOG_ERROR("[TrainManager] Node ID %s appears to be for an unsupported "
                "drive type:%d.", utils::node_id_to_string(node_id).c_str(),
                static_cast<int>(type));
      return nullptr;
    }
#if TRAINMGR_LOGLEVEL >= VERBOSE
    LOG(TRAINMGR_LOGLEVEL, "[TrainManager] Checking db for node id: %s",
        utils::node_id_to_string(node_id).c_str());
#endif // TRAINMGR_LOGLEVEL >= VERBOSE

    auto train = Singleton<LocoDatabase>::instance()->get_entry(node_id, addr);
    if (train != nullptr)
    {
#if TRAINMGR_LOGLEVEL >= VERBOSE
      LOG(TRAINMGR_LOGLEVEL, "[TrainManager] Matched %s",
          train->identifier().c_str());
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
      return create_impl(train->file_offset(), train->get_legacy_drive_mode(),
                         train->get_legacy_address());
    }
  }

  // no active train or db entry was found for the node id, give up
  return nullptr;
}

/// Returns a node id or 0 if the id is not known to be a train.
NodeID TrainManager::get_train_node_id(size_t offset)
{
  {
    OSMutexLock l(&trainsLock_);
    auto ent = std::find_if(trains_.begin(), trains_.end(),
    [offset](const auto &train)
    {
      return train->file_offset() == offset;
    });
    if (ent != trains_.end())
    {
      return (*ent)->node_id();
    }
  }

  LOG(TRAINMGR_LOGLEVEL,
      "[TrainManager] no active train with offset %zu, checking db", offset);
  auto ent = Singleton<LocoDatabase>::instance()->get_entry(offset);
  if (ent)
  {
#if TRAINMGR_LOGLEVEL >= VERBOSE
    LOG(TRAINMGR_LOGLEVEL,
        "[TrainManager] found existing db entry %s, creating node %s",
        ent->identifier().c_str(),
        utils::node_id_to_string(ent->get_traction_node()).c_str());
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
    create_impl(offset, ent->get_legacy_drive_mode(),
                ent->get_legacy_address());
    return ent->get_traction_node();
  }

  LOG(TRAINMGR_LOGLEVEL,
      "[TrainManager] no train node found for index %d, giving up", offset);
  return 0;
}

LazyInitTrainNode* TrainManager::create_impl(
  size_t train_id, DriveMode mode, int address)
{
  LazyInitTrainNode *impl =
    new LazyInitTrainNode(train_service(), train_id, mode, address);
  {
    OSMutexLock l(&trainsLock_);
    trains_.push_back(impl);
  }
#if TRAINMGR_LOGLEVEL >= VERBOSE
  LOG(TRAINMGR_LOGLEVEL, "[TrainManager] %s created for drive:%d with addr:%d",
      utils::node_id_to_string(impl->node_id()).c_str(),
      static_cast<int>(mode), address);
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
  return impl;
}

NodeID TrainManager::create_train_node(DriveMode drive_type, uint16_t address)
{
  if (((config_trainmgr_support_marklin() == CONSTANT_FALSE) &&
       (drive_type & DriveMode::MARKLIN_ANY)) ||
      ((config_trainmgr_support_dcc() == CONSTANT_FALSE) &&
       (drive_type & DriveMode::DCC_ANY)) || 
      ((config_trainmgr_support_openlcb_user() == CONSTANT_FALSE) &&
       (drive_type == DriveMode::OLCBUSER)))
  {
    LOG_ERROR("[TrainManager] Ignoring attempt to allocate unsupported drive "
              "type:%s (%d) using address:%d",
              locodb::drive_mode_to_string(drive_type),
              static_cast<int>(drive_type),
              address);
    return 0;
  }
  size_t offset =
    Singleton<LocoDatabase>::instance()->create_entry(address, drive_type);
  LazyInitTrainNode* impl = create_impl(offset, drive_type, address);
  if (!impl)
  {
    LOG_ERROR("[TrainManager] Failed to allocate new locomotive for "
              "drive:%s (%d), address:%d",
              locodb::drive_mode_to_string(drive_type),
              static_cast<int>(drive_type), address);
    // TODO: should we remove the new entry?
    return 0;
  }
#if TRAINMGR_LOGLEVEL >= VERBOSE
  LOG(TRAINMGR_LOGLEVEL, "[TrainManager] %s created for mode:%s address:%d",
      utils::node_id_to_string(impl->node_id()).c_str(),
      locodb::drive_mode_to_string(drive_type), address);
#endif // TRAINMGR_LOGLEVEL >= VERBOSE
  return impl->node_id();
}

void TrainManager::set_enabled(bool enabled)
{
  if (enabled)
  {
    snipHandler_ = std::make_unique<TrainSnipHandler>(this, infoFlow_);
    pipHandler_ = std::make_unique<TrainPipHandler>(this);
    pipHandler_ = std::make_unique<TrainPipHandler>(this);
    fdiSpace_ = std::make_unique<TrainFDISpace>(this);
    configSpace_ = std::make_unique<PersistentTrainConfigSpace>(this);
    cdiSpace_  = std::make_unique<TrainCDISpace>(this);
    trainIdentHandler_  = std::make_unique<TrainIdentifyHandler>(this);
    memoryConfigService_->registry()->insert(
        nullptr, MemoryConfigDefs::SPACE_FDI, fdiSpace_.get());
    memoryConfigService_->registry()->insert(
        nullptr, MemoryConfigDefs::SPACE_CDI, cdiSpace_.get());
    memoryConfigService_->registry()->insert(
        nullptr, MemoryConfigDefs::SPACE_CONFIG, configSpace_.get());
  }
  else
  {
    memoryConfigService_->registry()->erase(
        nullptr, MemoryConfigDefs::SPACE_FDI, fdiSpace_.get());
    memoryConfigService_->registry()->erase(
        nullptr, MemoryConfigDefs::SPACE_CDI, cdiSpace_.get());
    memoryConfigService_->registry()->erase(
        nullptr, MemoryConfigDefs::SPACE_CONFIG, configSpace_.get());
    snipHandler_.reset(nullptr);
    pipHandler_.reset(nullptr);
    pipHandler_.reset(nullptr);
    fdiSpace_.reset(nullptr);
    configSpace_.reset(nullptr);
    cdiSpace_.reset(nullptr);
    trainIdentHandler_.reset(nullptr);
  }
  trainSearchServer_.set_enabled(enabled);
}

void TrainManager::estop_all_trains()
{
  OSMutexLock l(&trainsLock_);
  for (auto* t : trains_)
  {
    if (t->is_allocated())
    {
      t->train()->set_emergencystop();
    }
  }
}

}  // namespace trainmanager
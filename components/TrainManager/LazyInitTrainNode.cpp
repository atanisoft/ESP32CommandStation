/*
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "LazyInitTrainNode.hxx"

#include <dcc/Address.hxx>
#include <dcc/Loco.hxx>
#include <locomgr/Defs.hxx>
#include <openlcb/TractionTrain.hxx>

namespace trainmanager
{

using dcc::Dcc128Train;
using dcc::Dcc28Train;
using dcc::DccLongAddress;
using dcc::DccShortAddress;

using locodb::DriveMode;
using locodb::drive_mode_to_address_type;

using openlcb::DefaultTrainNode;
using openlcb::NodeID;
using openlcb::TractionDefs;
using openlcb::TrainService;

#ifndef NODE_LOG_LEVEL
#ifdef CONFIG_TSP_LOGGING_LEVEL
#define NODE_LOG_LEVEL CONFIG_TSP_LOGGING_LEVEL
#else
#define NODE_LOG_LEVEL VERBOSE
#endif // CONFIG_TSP_LOGGING_LEVEL
#endif // NODE_LOG_LEVEL

LazyInitTrainNode::LazyInitTrainNode(
  TrainService *service, ssize_t offset, DriveMode mode,
  uint16_t address)
  : DefaultTrainNode(service, nullptr), offset_(offset), mode_(mode),
  addr_(address)
{
  service->register_train(this);

  // If automatic idle is enabled for new train impls, call to create the
  // new train impl instance.
  if (config_trainmgr_automatically_create_train_impl() == CONSTANT_TRUE)
  {
    train();
  }
}

LazyInitTrainNode::~LazyInitTrainNode()
{
  service_->unregister_train(this);
  delete train_;
}

NodeID LazyInitTrainNode::node_id()
{
  return TractionDefs::train_node_id_from_legacy(
    drive_mode_to_address_type(mode_, addr_), addr_);
}

uint16_t LazyInitTrainNode::address()
{
  return addr_;
}

DriveMode LazyInitTrainNode::mode()
{
  return mode_;
}

ssize_t LazyInitTrainNode::file_offset()
{
  return offset_;
}

bool LazyInitTrainNode::is_allocated()
{
  return train_ != nullptr;
}

openlcb::TrainImpl *LazyInitTrainNode::train()
{
  if (train_ == nullptr)
  {
    switch (mode_)
    {
      case DriveMode::DCC_14:
      case DriveMode::DCC_14_LONG_ADDRESS:
      // TODO: DCC-14 not implemented in OpenMRN (yet)
      case DriveMode::DCC_28:
      case DriveMode::DCC_28_LONG_ADDRESS:
      {
        LOG(NODE_LOG_LEVEL, "[Train:%d] Creating new DCC-14/28 instance",
            addr_);
        if ((mode_ & DriveMode::DCC_LONG_ADDRESS) ||
            addr_ >= DccShortAddress::ADDRESS_MAX)
        {
          train_ = new Dcc28Train(DccLongAddress(addr_));
        }
        else
        {
          train_ = new Dcc28Train(DccShortAddress(addr_));
        }
        break;
      }
      case DriveMode::DCC_128:
      case DriveMode::DCC_128_LONG_ADDRESS:
      {
        LOG(NODE_LOG_LEVEL, "[Train:%d] Creating new DCC-128 instance",
            addr_);
        if ((mode_ & DriveMode::DCC_LONG_ADDRESS) ||
            addr_ >= DccShortAddress::ADDRESS_MAX)
        {
          train_ = new Dcc128Train(DccLongAddress(addr_));
        }
        else
        {
          train_ = new Dcc128Train(DccShortAddress(addr_));
        }
        break;
      }
      default:
        train_ = nullptr;
        LOG_ERROR("[Train:%d] Unhandled train drive mode: %d.", addr_,
                  mode_);
    }
  }
  return train_;
}

} // namespace trainmanager
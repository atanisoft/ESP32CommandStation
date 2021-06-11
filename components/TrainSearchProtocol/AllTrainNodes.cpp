/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file AllTrainNodes.hxx
 *
 * A class that instantiates every train node from the TrainDb.
 *
 * @author Balazs Racz
 * @date 20 May 2014
 */

#include "AllTrainNodes.hxx"

#include "FdiXmlGenerator.hxx"
#include "FindProtocolServer.hxx"
#include "TrainDb.hxx"
#include <dcc/Loco.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/SimpleNodeInfo.hxx>
#include <openlcb/TractionDefs.hxx>
#include <openlcb/TractionTrain.hxx>
#include <StringUtils.hxx>
#include <utils/format_utils.hxx>

#include <algorithm>

namespace commandstation
{

using openlcb::Defs;
using openlcb::TractionDefs;

struct AllTrainNodes::Impl
{
 public:
  ~Impl()
  {
    delete node_;
    delete train_;
  }
  int id;
  openlcb::Node* node_{nullptr};
  openlcb::TrainImpl* train_{nullptr};
};

void AllTrainNodes::remove_train_impl(int address)
{
  OSMutexLock l(&trainsLock_);
  auto it = std::find_if(trains_.begin(), trains_.end(), [address](Impl *impl)
  {
    return impl->train_->legacy_address() == address;
  });
  if (it != trains_.end())
  {
    Impl *impl = (*it);
    impl->node_->iface()->delete_local_node(impl->node_);
    delete impl;
    trains_.erase(it);
  }
}

openlcb::TrainImpl* AllTrainNodes::get_train_impl(openlcb::NodeID id, bool allocate)
{
  auto it = find_node(id, allocate);
  if (it)
  {
    return it->train_;
  }
  return nullptr;
}

openlcb::TrainImpl* AllTrainNodes::get_train_impl(DccMode drive_type, int address)
{
  {
    OSMutexLock l(&trainsLock_);
    auto it = std::find_if(trains_.begin(), trains_.end(), [address](Impl *impl)
    {
      return impl->train_->legacy_address() == address;
    });
    if (it != trains_.end())
    {
      return (*it)->train_;
    }
  }
  return find_node(allocate_node(drive_type, address))->train_;
}

AllTrainNodes::Impl* AllTrainNodes::find_node(openlcb::Node* node) 
{
  {
    OSMutexLock l(&trainsLock_);
    auto it = std::find_if(trains_.begin(), trains_.end(),
      [node](Impl *impl)
      {
        return impl->node_ == node;
      });
    if (it != trains_.end())
    {
      return *it;
    }
  }
  // no active train was found with the provided node reference, try to find
  // one via the node-id instead.
  if (node != nullptr && node->node_id())
  {
    return find_node(node->node_id());
  }
  return nullptr;  
}

AllTrainNodes::Impl* AllTrainNodes::find_node(openlcb::NodeID node_id, bool allocate)
{
  {
    OSMutexLock l(&trainsLock_);
    auto it = std::find_if(trains_.begin(), trains_.end(), [node_id](Impl *impl)
    {
      return impl->node_->node_id() == node_id;
    });
    if (it != trains_.end())
    {
      return *it;
    }
  }
  if (!allocate)
  {
    return nullptr;
  }

  // no active train was found having the provided node id, search for a train
  // in the db that should have the provided node id and return it instead if
  // found.
  dcc::TrainAddressType type;
  uint32_t addr =  0;
  if (TractionDefs::legacy_address_from_train_node_id(node_id, &type, &addr))
  {
    LOG(INFO, "[TrainSearch] Checking db for node id: %s",
        esp32cs::node_id_to_string(node_id).c_str());
    auto train = db_->find_entry(node_id, addr);
    if (train != nullptr)
    {
      LOG(INFO, "[TrainSearch] Matched %s", train->identifier().c_str());
      return create_impl(train->file_offset()
                      , train->get_legacy_drive_mode()
                      , train->get_legacy_address());
    }
  }

  // no active train or db entry was found for the node id, give up
  return nullptr;
}

/// Returns a traindb entry or nullptr if the id is too high.
std::shared_ptr<TrainDbEntry> AllTrainNodes::get_traindb_entry(
  size_t id, Notifiable* done)
{
  AutoNotify n(done);
  return db_->get_entry(id);
}

/// Returns a node id or 0 if the id is not known to be a train.
openlcb::NodeID AllTrainNodes::get_train_node_id_ext(size_t id, bool allocate)
{
  {
    OSMutexLock l(&trainsLock_);
    if (id < trains_.size() && trains_[id]->node_)
    {
      return trains_[id]->node_->node_id();
    }
  }
  if (!allocate)
  {
    return 0;
  }

  LOG(INFO, "[TrainSearch] no active train with index %d, checking db", id);
  auto db_entry = db_->get_entry(id);
  if (db_entry != nullptr)
  {
    LOG(INFO, "[TrainSearch] found existing db entry %s, creating node %s",
        db_entry->identifier().c_str(),
        esp32cs::node_id_to_string(db_entry->get_traction_node()).c_str());
    create_impl(id, db_entry->get_legacy_drive_mode()
              , db_entry->get_legacy_address());
    return db_entry->get_traction_node();
  }

  LOG(INFO, "[TrainSearch] no train node found for index %d, giving up", id);
  return 0;
}

class AllTrainNodes::TrainSnipHandler : public openlcb::IncomingMessageStateFlow
{
 public:
  TrainSnipHandler(AllTrainNodes* parent, openlcb::SimpleInfoFlow* info_flow)
      : IncomingMessageStateFlow(parent->train_service()->iface()),
        parent_(parent),
        responseFlow_(info_flow)
  {
    iface()->dispatcher()->register_handler(
        this, Defs::MTI::MTI_IDENT_INFO_REQUEST, Defs::MTI::MTI_EXACT);
  }
  ~TrainSnipHandler()
  {
    iface()->dispatcher()->unregister_handler(
        this, Defs::MTI::MTI_IDENT_INFO_REQUEST, Defs::MTI::MTI_EXACT);
  }

  Action entry() override
  {
    // Let's find the train ID.
    impl_ = parent_->find_node(nmsg()->dstNode);
    if (!impl_) return release_and_exit();
    return allocate_and_call(responseFlow_, STATE(send_response_request));
  }

  Action send_response_request()
  {
    auto* b = get_allocation_result(responseFlow_);
    auto entry = parent_->db_->get_entry(impl_->id);
    if (entry.get())
    {
      snipName_ = entry->get_train_name();
    }
    else
    {
      snipName_.clear();
    }
    snipResponse_[6].data = snipName_.c_str();
    b->data()->reset(nmsg(), snipResponse_, Defs::MTI_IDENT_INFO_REPLY);
    // We must wait for the data to be sent out because we have a static member
    // that we are changing constantly.
    b->set_done(n_.reset(this));
    responseFlow_->send(b);
    release();
    return wait_and_call(STATE(send_done));
  }

  Action send_done()
  {
    return exit();
  }

 private:
  AllTrainNodes* parent_;
  openlcb::SimpleInfoFlow* responseFlow_;
  AllTrainNodes::Impl* impl_;
  BarrierNotifiable n_;
  string snipName_;
  static openlcb::SimpleInfoDescriptor snipResponse_[];
};

openlcb::SimpleInfoDescriptor AllTrainNodes::TrainSnipHandler::snipResponse_[] =
{
  {openlcb::SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
  {openlcb::SimpleInfoDescriptor::C_STRING, 0, 0,
  openlcb::SNIP_STATIC_DATA.manufacturer_name},
  {openlcb::SimpleInfoDescriptor::C_STRING, 41, 0, "Virtual train node"},
  {openlcb::SimpleInfoDescriptor::C_STRING, 0, 0, "n/a"},
  {openlcb::SimpleInfoDescriptor::C_STRING, 0, 0,
  openlcb::SNIP_STATIC_DATA.software_version},
  {openlcb::SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr},
  {openlcb::SimpleInfoDescriptor::C_STRING, 63, 1, nullptr},
  {openlcb::SimpleInfoDescriptor::C_STRING, 0, 0, "n/a"},
  {openlcb::SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}
};

class AllTrainNodes::TrainPipHandler : public openlcb::IncomingMessageStateFlow
{
 public:
  TrainPipHandler(AllTrainNodes* parent)
      : IncomingMessageStateFlow(parent->train_service()->iface()),
        parent_(parent)
  {
    iface()->dispatcher()->register_handler(
        this, Defs::MTI::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI::MTI_EXACT);
  }

  ~TrainPipHandler()
  {
    iface()->dispatcher()->unregister_handler(
        this, Defs::MTI::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI::MTI_EXACT);
  }

 private:
  Action entry() override {
    if (parent_->find_node(nmsg()->dstNode) == nullptr) {
      return release_and_exit();
    }

    return allocate_and_call(iface()->addressed_message_write_flow(),
                             STATE(fill_response_buffer));
  }

  Action fill_response_buffer() {
    // Grabs our allocated buffer.
    auto* b = get_allocation_result(iface()->addressed_message_write_flow());
    auto reply = pipReply_;
    // Fills in response. We use node_id_to_buffer because that converts a
    // 48-bit value to a big-endian byte string.
    b->data()->reset(Defs::MTI_PROTOCOL_SUPPORT_REPLY,
                     nmsg()->dstNode->node_id(), nmsg()->src,
                     openlcb::node_id_to_buffer(reply));

    // Passes the response to the addressed message write flow.
    iface()->addressed_message_write_flow()->send(b);

    return release_and_exit();
  }

  AllTrainNodes* parent_;
  static constexpr uint64_t pipReply_ =
      Defs::SIMPLE_PROTOCOL_SUBSET | Defs::DATAGRAM |
      Defs::MEMORY_CONFIGURATION | Defs::EVENT_EXCHANGE |
      Defs::SIMPLE_NODE_INFORMATION | Defs::TRACTION_CONTROL |
      Defs::TRACTION_FDI | Defs::CDI;
};

class AllTrainNodes::TrainFDISpace : public openlcb::MemorySpace
{
 public:
  TrainFDISpace(AllTrainNodes* parent) : parent_(parent) {}

  bool set_node(openlcb::Node* node) override
  {
    if (impl_ && impl_->node_ == node)
    {
      // same node.
      return true;
    }
    impl_ = parent_->find_node(node);
    if (impl_ != nullptr)
    {
      reset_file();
      return true;
    }
    return false;
  }

  address_t max_address() override
  {
    // We don't really know how long this space is; 16 MB is an upper bound.
    return 16 << 20;
  }

  size_t read(address_t source, uint8_t* dst, size_t len, errorcode_t* error,
              Notifiable* again) override
  {
    if (source <= gen_.file_offset())
    {
      reset_file();
    }
    ssize_t result = gen_.read(source, dst, len);
    if (result < 0)
    {
      LOG_ERROR("[TrainFDI] Read failure: %u, %zu: %zu (%s)", source, len
              , result, strerror(errno));
      *error = Defs::ERROR_PERMANENT;
      return 0;
    }
    if (result == 0)
    {
      LOG(VERBOSE, "[TrainFDI] Out-of-bounds read: %u, %zu", source, len);
      *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
    }
    else
    {
      *error = 0;
    }
    return result;
  }

 private:
  void reset_file()
  {
    auto e = parent_->db_->get_entry(impl_->id);
    e->start_read_functions();
    gen_.reset(std::move(e));
  }

  FdiXmlGenerator gen_;
  AllTrainNodes* parent_;
  // Train object structure.
  Impl* impl_{nullptr};
};

class AllTrainNodes::TrainConfigSpace : public openlcb::FileMemorySpace
{
 public:
  TrainConfigSpace(int fd, AllTrainNodes* parent, size_t file_end)
      : FileMemorySpace(fd, file_end), parent_(parent) {}

  bool set_node(openlcb::Node* node) override
  {
    if (impl_ && impl_->node_ == node)
    {
      // same node.
      return true;
    }
    impl_ = parent_->find_node(node);
    if (impl_ == nullptr)
    {
      return false;
    }
    auto entry = parent_->db_->get_entry(impl_->id);
    if (!entry)
    {
      return false;
    }
    int offset = entry->file_offset();
    if (offset < 0)
    {
      return false;
    }
    offset_ = offset;
    return true;
  }

  size_t read(address_t source, uint8_t* dst, size_t len, errorcode_t* error,
              Notifiable* again) override
  {
    return FileMemorySpace::read(source + offset_, dst, len, error, again);
  }

  size_t write(address_t destination, const uint8_t* data, size_t len,
               errorcode_t* error, Notifiable* again) override
  {
    return FileMemorySpace::write(destination + offset_, data, len, error,
                                  again);
  }

 private:
  unsigned offset_;
  AllTrainNodes* parent_;
  // Train object structure.
  Impl* impl_{nullptr};
};

class AllTrainNodes::TrainCDISpace : public openlcb::MemorySpace
{
 public:
  TrainCDISpace(AllTrainNodes* parent) : parent_(parent) {}

  bool set_node(openlcb::Node* node) override
  {
    if (impl_ && impl_->node_ == node)
    {
      // same node.
      return true;
    }
    impl_ = parent_->find_node(node);
    if (impl_ == nullptr)
    {
      return false;
    }
    auto entry = parent_->db_->get_entry(impl_->id);
    if (!entry) 
    {
      return false;
    }
    int offset = entry->file_offset();
    if (offset < 0)
    {
      proxySpace_ = parent_->ro_tmp_train_cdi_;
    }
    else
    {
      proxySpace_ = parent_->ro_train_cdi_;
    }
    return true;
  }

  address_t max_address() override
  {
    return proxySpace_->max_address();
  }

  size_t read(address_t source, uint8_t* dst, size_t len, errorcode_t* error,
              Notifiable* again) override
  {
    return proxySpace_->read(source, dst, len, error, again);
  }

  AllTrainNodes* parent_;
  // Train object structure.
  Impl* impl_{nullptr};
  openlcb::MemorySpace* proxySpace_;
};

class AllTrainNodes::TrainIdentifyHandler : public openlcb::IncomingMessageStateFlow
{
public:
  TrainIdentifyHandler(AllTrainNodes *parent)
    : IncomingMessageStateFlow(parent->train_service()->iface())
    , parent_(parent)
  {
    iface()->dispatcher()->register_handler(
            this, Defs::MTI::MTI_VERIFY_NODE_ID_GLOBAL, Defs::MTI::MTI_EXACT);
  }

  ~TrainIdentifyHandler()
  {
    iface()->dispatcher()->unregister_handler(
            this, Defs::MTI_VERIFY_NODE_ID_GLOBAL, Defs::MTI::MTI_EXACT);
  }

  /// Handler callback for incoming messages.
  IncomingMessageStateFlow::Action entry() override
  {
    openlcb::GenMessage *m = message()->data();
    if (!m->payload.empty() && m->payload.size() == 6)
    {
      target_ = openlcb::buffer_to_node_id(m->payload);
      LOG(INFO, "[TrainIdent] received global identify for node %s",
          esp32cs::node_id_to_string(target_).c_str());
      openlcb::NodeID masked = target_ & TractionDefs::NODE_ID_MASK;
      if ((masked == TractionDefs::NODE_ID_DCC ||
           masked == TractionDefs::NODE_ID_MARKLIN_MOTOROLA ||
           masked == 0x050100000000ULL) && // TODO: move this constant into TractionDefs
          parent_->find_node(target_) != nullptr)
      {
        LOG(INFO, "[TrainIdent] matched a known train db entry");
        release();
        return allocate_and_call(iface()->global_message_write_flow(),
                                STATE(send_train_ident));
      }
    }
    return release_and_exit();
  }
private:
  AllTrainNodes *parent_;
  openlcb::NodeID target_;

  IncomingMessageStateFlow::Action send_train_ident()
  {
    auto *b =
        get_allocation_result(iface()->global_message_write_flow());
    openlcb::GenMessage *m = b->data();
    m->reset(Defs::MTI_VERIFIED_NODE_ID_NUMBER, target_
           , openlcb::node_id_to_buffer(target_));
    iface()->global_message_write_flow()->send(b);
    return exit();
  }
};

AllTrainNodes::AllTrainNodes(TrainDb* db,
                             openlcb::TrainService* traction_service,
                             openlcb::SimpleInfoFlow* info_flow,
                             openlcb::MemoryConfigHandler* memory_config,
                             openlcb::MemorySpace* ro_train_cdi,
                             openlcb::MemorySpace* ro_tmp_train_cdi)
    : AllTrainNodesInterface(traction_service),
      db_(db),
      memoryConfigService_(memory_config),
      ro_train_cdi_(ro_train_cdi),
      ro_tmp_train_cdi_(ro_tmp_train_cdi),
      findProtocolServer_(this),
      snipHandler_(new TrainSnipHandler(this, info_flow)),
      pipHandler_(new TrainPipHandler(this)),
      fdiSpace_(new TrainFDISpace(this)),
      cdiSpace_(new TrainCDISpace(this)),
      trainIdentHandler_(new TrainIdentifyHandler(this))
{
  HASSERT(ro_train_cdi_->read_only());
  HASSERT(ro_tmp_train_cdi_->read_only());
  memoryConfigService_->registry()->insert(
      nullptr, openlcb::MemoryConfigDefs::SPACE_FDI, fdiSpace_.get());
  memoryConfigService_->registry()->insert(
      nullptr, openlcb::MemoryConfigDefs::SPACE_CDI, cdiSpace_.get());
}

AllTrainNodes::Impl* AllTrainNodes::create_impl(int train_id, DccMode mode,
                                                int address)
{
  Impl* impl = new Impl;
  impl->id = train_id;
  switch (mode)
  {
    case MARKLIN_OLD:
    {
      LOG(INFO, "[TrainSearch] Allocating New Marklin (old) train %d", address);
      impl->train_ = new dcc::MMOldTrain(dcc::MMAddress(address));
      break;
    }
    case MARKLIN_DEFAULT:
    case MARKLIN_NEW:
      /// @todo (balazs.racz) implement marklin twoaddr train drive mode.
    case MARKLIN_TWOADDR:
    {
      LOG(INFO, "[TrainSearch] Allocating New Marklin (new) train %d", address);
      impl->train_ = new dcc::MMNewTrain(dcc::MMAddress(address));
      break;
    }
      /// @todo (balazs.racz) implement dcc 14 train drive mode.
    case DCC_14:
    case DCC_14_LONG_ADDRESS:
    case DCC_28:
    case DCC_28_LONG_ADDRESS:
    {
      LOG(INFO, "[TrainSearch] Allocating New DCC-14/28 train %d", address);
      if ((mode & DCC_LONG_ADDRESS) || address >= 128)
      {
        impl->train_ = new dcc::Dcc28Train(dcc::DccLongAddress(address));
      }
      else
      {
        impl->train_ = new dcc::Dcc28Train(dcc::DccShortAddress(address));
      }
      break;
    }
    case DCC_128:
    case DCC_128_LONG_ADDRESS:
    {
      LOG(INFO, "[TrainSearch] Allocating New DCC-128 train %d", address);
      if ((mode & DCC_LONG_ADDRESS) || address >= 128)
      {
        impl->train_ = new dcc::Dcc128Train(dcc::DccLongAddress(address));
      }
      else
      {
        impl->train_ = new dcc::Dcc128Train(dcc::DccShortAddress(address));
      }
      break;
    }
    default:
      impl->train_ = nullptr;
      LOG_ERROR("[TrainSearch] Unhandled train drive mode: %d.", mode);
  }
  if (impl->train_)
  {
    {
      OSMutexLock l(&trainsLock_);
      trains_.push_back(impl);
    }
    impl->node_ = new openlcb::TrainNodeForProxy(train_service(), impl->train_);
    return impl;
  }
  else
  {
    delete impl;
    return nullptr;
  }
}

size_t AllTrainNodes::size()
{
  return std::max(trains_.size(), db_->size());
}

bool AllTrainNodes::is_valid_train_node(openlcb::Node *node)
{
  return find_node(node) != nullptr;
}

bool AllTrainNodes::is_valid_train_node(openlcb::NodeID node_id, bool allocate)
{
  return find_node(node_id, allocate) != nullptr;
}

openlcb::NodeID AllTrainNodes::allocate_node(DccMode drive_type,
                                             unsigned address)
{
  Impl* impl = create_impl(-1, drive_type, address);
  if (!impl) return 0; // failed.
  impl->id = db_->add_dynamic_entry(address, drive_type);
  return impl->node_->node_id();
}

AllTrainNodes::~AllTrainNodes()
{
  OSMutexLock l(&trainsLock_);
  for (auto* t : trains_) {
    delete t;
  }
  memoryConfigService_->registry()->erase(
      nullptr, openlcb::MemoryConfigDefs::SPACE_FDI, fdiSpace_.get());
  memoryConfigService_->registry()->erase(
      nullptr, openlcb::MemoryConfigDefs::SPACE_CDI, cdiSpace_.get());
}

}  // namespace commandstation
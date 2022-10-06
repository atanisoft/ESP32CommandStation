/*
 * SPDX-FileCopyrightText: 2014 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TRAINMANAGERIMPL_HXX_
#define TRAINMANAGERIMPL_HXX_

#include <memory>
#include <vector>

#include <openlcb/Defs.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <locodb/LocoDatabase.hxx>
#include <locomgr/LocoManager.hxx>
#include <trainsearch/TrainSearchProtocolServer.hxx>
#include <utils/Singleton.hxx>
#include <utils/Uninitialized.hxx>

namespace openlcb
{
class DefaultTrainNode;
class Node;
class SimpleInfoFlow;
class TrainService;
class TrainImpl;
}

namespace trainmanager
{

class LazyInitTrainNode;
class TrainCDISpace;
class TrainFDISpace;
class PersistentTrainConfigSpace;
class TrainIdentifyHandler;
class TrainPipHandler;
class TrainSnipHandler;

class TrainManager : public locomgr::LocoManager
{
 public:
  TrainManager(openlcb::TrainService* traction_service,
               openlcb::SimpleInfoFlow* info_flow,
               openlcb::MemoryConfigHandler* memory_config);
  ~TrainManager();

  /// Return the number of known locomotives or those being serviced.
  size_t size()
  {
    return std::max(trains_.size(),
                    Singleton<locodb::LocoDatabase>::instance()->size());
  }

  /// Returns the number of locomotives that are actively being serviced.
  size_t active_locos()
  {
    return trains_.size();
  }

  /// Returns a node id or 0 if the id is not known to be a train.
  openlcb::NodeID get_train_node_id(size_t offset) override;

  /// Creates a new train node based on the given address and drive mode.
  ///
  /// @param drive_type describes what kind of train node this should be
  /// @param address is the hardware (legacy) address
  /// @return 0 if the allocation fails (invalid arguments)
  openlcb::NodeID create_train_node(locodb::DriveMode drive_type,
                                    uint16_t address) override;

  /// Searches for a @ref TrainImpl matching the openlcb node ID.
  ///
  /// @param node_id @ref NodeID to search for.
  openlcb::TrainImpl* find_train(openlcb::NodeID node_id);

  /// Finds or creates a TrainImpl for the requested address and drive_type.
  ///
  /// @param drive_type is the drive type for the loco to create if it doesn't exist.
  /// @param address is the legacy address of the loco to find or create.
  openlcb::TrainImpl* find_or_create_train(locodb::DriveMode drive_type,
                                           int address);

  /// Removes a TrainImpl for the requested address if it exists.
  void delete_train(locodb::DriveMode drive_type, int address);

  /// @return true if the provided node is a known/active train.
  bool is_valid_train_node(openlcb::Node *node)
  {
    return find_node(node) != nullptr;
  }

  /// @return true if the provided node id is a known/active train.
  bool is_valid_train_node(openlcb::NodeID node_id, bool allocate = true)
  {
    return find_node(node_id, allocate) != nullptr;
  }

  /// Enables/Disables the @ref AllTrainNodes based on persistent
  /// configuration settings.
  ///
  /// @param enabled When true the @ref AllTrainNodes will listen for and
  /// respond to OpenLCB Events related to train search.
  void set_enabled(bool enabled);

  void estop_all_trains();

 private:
  /// A child can look up if a local node is actually a Train node. If so, the
  /// Impl structure will be returned. If the node is not known (or not a train
  /// node maintained by this object), we return nullptr.
  LazyInitTrainNode* find_node(openlcb::Node* node);

  /// Extension to the find_node implementation that exposes the option to not
  /// allocate a node when no existing node is found.
  LazyInitTrainNode* find_node(openlcb::NodeID node_id, bool allocate = true);

  /// Helper function to create lok objects. Adds a new Impl structure to
  /// trains_.
  LazyInitTrainNode* create_impl(size_t train_id, locodb::DriveMode mode,
                                 int address);

  // Externally owned.
  openlcb::MemoryConfigHandler* memoryConfigService_;
  openlcb::SimpleInfoFlow* infoFlow_;

  openlcb::ReadOnlyMemoryBlock ro_train_cdi_;
  openlcb::ReadOnlyMemoryBlock ro_tmp_train_cdi_;

  /// All train nodes that we know about.
  std::vector<LazyInitTrainNode *> trains_;
  
  /// Lock to protect trains_.
  OSMutex trainsLock_;

  /// Train Search Protocol implementation
  trainsearch::TrainSearchProtocolServer trainSearchServer_;

  // Implementation objects that we carry for various protocols.
  friend class TrainSnipHandler;
  std::unique_ptr<TrainSnipHandler> snipHandler_;

  friend class TrainPipHandler;
  std::unique_ptr<TrainPipHandler> pipHandler_;
  
  friend class TrainFDISpace;
  std::unique_ptr<TrainFDISpace> fdiSpace_;

  friend class PersistentTrainConfigSpace;
  std::unique_ptr<PersistentTrainConfigSpace> configSpace_;

  friend class TrainCDISpace;
  std::unique_ptr<TrainCDISpace> cdiSpace_;

  friend class TrainIdentifyHandler;
  std::unique_ptr<TrainIdentifyHandler> trainIdentHandler_;
};

}  // namespace trainmanager

#endif // TRAINMANAGERIMPL_HXX_

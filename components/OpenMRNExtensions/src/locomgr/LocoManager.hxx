/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef TRAINMANAGER_HXX_
#define TRAINMANAGER_HXX_

#include <memory>

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabaseEntry.hxx"
#include "locomgr/Defs.hxx"
#include <openlcb/Defs.hxx>
#include <utils/Destructable.hxx>
#include <utils/Singleton.hxx>

class Notifiable;

namespace openlcb
{
  class If;
  class MemoryConfigHandler;
  class Node;
  class SimpleInfoFlow;
  class TrainImpl;
  class TrainService;
}

namespace locomgr
{

// This should really be defined inside TractionDefs.hxx and used by the call
// to TractionDefs::train_node_id_from_legacy().
static constexpr uint64_t const NODE_ID_OLCB = 0x050100000000ULL;

/// Abstract class for the LocoManager that prevents pulling in transitive
/// dependencies.
class LocoManager : public Destructable, public Singleton<LocoManager>
{
public:
  /// Constructor.
  /// @param service points to the traction service. Externally owned
  /// (ownership is not taken).
  LocoManager(openlcb::TrainService *service)
      : trainService_(service)
  {
  }

  /// @return the traction service instance.
  openlcb::TrainService *train_service();

  /// @return the traction service instance.
  openlcb::If *iface();

  /// @return true if the @ref Node is known by this @ref TrainManager.
  /// @param node @ref Node to verify.
  bool is_known_train_node(openlcb::Node *node);

  /// @return maximum (or current) number of trains managed by this
  /// service. Trains are indexed 0..size().
  virtual size_t size() = 0;

  /// Returns the number of locomotives that are actively being serviced.
  virtual size_t active_locos() = 0;

  /// @return the openlcb train node ID for a given train index, or 0 if
  /// the train index is not valid.
  /// @param index 0..size() - 1.
  virtual openlcb::NodeID get_train_node_id(size_t index) = 0;

  /// Allocates a new legacy train node.
  /// @param mode which protocol mode to use.
  /// @param address legacy address (to be interpreted for the given protocol
  /// mode).
  /// @return the openlcb train node ID, or 0 if the arguments are not valid.
  virtual openlcb::NodeID create_train_node(
    locodb::DriveMode mode, uint16_t address) = 0;

  /// Finds a @ref TrainImpl matching the provided openlcb train node ID.
  /// @param node_id @ref NodeID to search for.
  /// @return @ref TrainImpl matching the openlcb train node ID or nullptr if
  /// it does not exist.
  virtual openlcb::TrainImpl* find_train(openlcb::NodeID id) = 0;

  /// Finds or creates a TrainImpl for the requested address and drive_type.
  /// @param drive_type is the drive type for the loco to create if it doesn't exist.
  /// @param address is the legacy address of the loco to find or create.
  /// @return @ref TrainImpl for the requested train or nullptr on failure.
  virtual openlcb::TrainImpl* find_or_create_train(
    locodb::DriveMode drive_type, int address) = 0;

  /// Removes a @ref TrainImpl matching the provided parameters if present.
  /// @param drive_type is the @ref DriveMode to remove.
  /// @param address is the legacy train address to remove.
  virtual void delete_train(locodb::DriveMode drive_type, int address) = 0;

  /// Enables/Disables the @ref AllTrainNodes based on persistent
  /// configuration settings.
  ///
  /// @param enabled When true the @ref LocoManager will listen for and
  /// respond to OpenLCB Events related to train search.
  virtual void set_enabled(bool enabled) = 0;

  /// Sets all active trains to eStop mode.
  virtual void estop_all_trains() = 0;

protected:
  /// Pointer to the traction service instance. Externally owned.
  openlcb::TrainService *trainService_;
};

} // namespace locomgr

#endif // TRAINMANAGER_HXX_
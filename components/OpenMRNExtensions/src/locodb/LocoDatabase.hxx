/*
 * SPDX-FileCopyrightText: 2014 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _LOCODB_LOCODATABASE_HXX_
#define _LOCODB_LOCODATABASE_HXX_

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabaseEntry.hxx"

#include <executor/Service.hxx>
#include <memory>
#include <openlcb/Defs.hxx>
#include <utils/ConfigUpdateListener.hxx>
#include <utils/Singleton.hxx>

namespace locodb
{

/// This class defines a common API for a Locomotive Database. This is used by
/// the @ref TrainSearchProtocolServer and @ref LocoManager to lookup available
/// locomotives.
class LocoDatabase : public Singleton<LocoDatabase>
{
 public:
  /// @returns the number of traindb entries. The valid train IDs will then be
  /// 0 <= id < size().
  virtual size_t size() = 0;

  /// Returns true if a train of a specific identifier is known to the traindb.
  /// @param train_id is the train identifier. Valid values: anything.
  /// Typical values: 0..NUM_TRAINS
  virtual bool is_valid_train(size_t train_id) = 0;

  /// Returns true if a train of a specific identifier is known to the traindb.
  /// @param train_id is the node id of the train being queried.
  virtual bool is_valid_train(openlcb::NodeID train_id) = 0;

  /// Returns true if a train of a specific identifier is known to the traindb.
  /// @param train_id is the node id of the train being queried.
  virtual int get_entry_offset(openlcb::NodeID train_id) = 0;

  /// Returns a train DB entry if the train ID is known, otherwise nullptr. The
  /// ownership of the entry is not transferred.
  /// @return @ref LocoDatabaseEntry for the train or null if not found.
  virtual std::shared_ptr<LocoDatabaseEntry> get_entry(
    const std::string &name) = 0;

  /// Returns a train DB entry if the train ID is known, otherwise nullptr. The
  /// ownership of the entry is not transferred.
  /// @param train_id is the train identifier.
  /// @param done will be notified exactly once, either inline (if the answer
  /// is ready) or out of line (if it is not).
  /// @return @ref LocoDatabaseEntry for the train or null if not found.
  virtual std::shared_ptr<LocoDatabaseEntry> get_entry(size_t train_id) = 0;

  /// Searches for an entry by the traction node ID. Returns nullptr if not
  /// found.
  /// @param traction_node_id node id of the train to search for.
  /// @param hint is a train_id that might be a match.
  /// @return @ref LocoDatabaseEntry for the train or null if not found.
  virtual std::shared_ptr<LocoDatabaseEntry> get_entry(
    openlcb::NodeID traction_node_id, unsigned hint = 0) = 0;

  /// Inserts a given entry into the train database.
  /// @param address the locomotive address to create.
  /// @param mode the operating mode for the new locomotive.
  /// @returns the new train_id for the given entry.
  virtual size_t create_entry(uint16_t address, DriveMode mode) = 0;

  /// Removes a train from the database by it's identifier.
  /// @param train_id is the train identifier. Valid values: anything.
  /// Typical values: 0..NUM_TRAINS
  virtual void remove_entry(size_t train_id) = 0;

  /// Removes a train from the database that matches the specified criteria.
  /// @param address the locomotive address to remove.
  /// @param mode the operating mode for the locomotive to be removed.
  virtual void remove_entry(uint16_t address, DriveMode mode) = 0;
};

}  // namespace locodb

#endif // _LOCODB_LOCODATABASE_HXX_
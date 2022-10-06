/*
 * SPDX-FileCopyrightText: 2014-2016 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _COMMANDSTATION_FINDPROTOCOLDEFS_HXX_
#define _COMMANDSTATION_FINDPROTOCOLDEFS_HXX_

#include <openlcb/EventHandler.hxx>
#include "locodb/Defs.hxx"
#include "locodb/LocoDatabaseEntry.hxx"

namespace trainsearch
{

/// Defines constants and utility functions used as part of Train Search
/// requests.
///
/// https://github.com/openlcb/documents/blob/master/drafts/TrainSearchS.pdf
struct TrainSearchDefs
{
  /// Defined EventId prefix used for Train Search requests.
  static constexpr openlcb::EventId EVENT_SEARCH_BASE = 0x090099FF00000000U;

  /// Offset in the bits at which to start searching for request parameters.
  static constexpr uint8_t TRAIN_FIND_MASK = 32;

  /// Offset at which to stop searching for request parameters and to start
  /// processing request flags.
  static constexpr uint8_t TRAIN_FIND_MASK_LOW = 8;

  /// Search request flag definitions.
  enum : uint8_t
  {
    /// Flag that indicates the locomotive should be allocated if not found.
    ALLOCATE = 0x80,

    /// Flag that indicates exact match only.
    EXACT = 0x40,

    /// Flag that indicates address match only.
    ADDRESS_ONLY = 0x20,

    // Bits 0-4 are a DriveMode enum.
    
    /// Match response information. This bit is not used in the network
    /// protocol. This bit will be set in the matched result, while cleared for
    /// no-match.
    MATCH_ANY = 0x01,
  };

  static_assert((EVENT_SEARCH_BASE & ((1ULL << TRAIN_FIND_MASK) - 1)) == 0,
                "EVENT_SEARCH_BASE is not all zero on the bottom");

  static_assert(((EVENT_SEARCH_BASE >> TRAIN_FIND_MASK) & 1) == 1,
                "The lowermost bit of the EVENT_SEARCH_BASE must be 1 or "
                "else the event produced range encoding must be updated.");

  /// Search nibble definitions.
  enum
  {
    /// Nibble is not used.
    NIBBLE_UNUSED = 0xf,
    
    /// Nibble is a white space character.
    NIBBLE_SPACE = 0xe,

    /// Nibble is a wildcard (match anything).
    NIBBLE_STAR = 0xd,

    /// Undefined.
    NIBBLE_QN = 0xc,

    /// Undefined.
    NIBBLE_HASH = 0xb,
  };

  /// @param event is an openlcb event ID
  /// @return true if that event ID belong to the find protocol event range.
  static inline bool is_find_event(openlcb::EventId event)
  {
    return (event >> TRAIN_FIND_MASK) == (EVENT_SEARCH_BASE >> TRAIN_FIND_MASK);
  }

  /// Compares an incoming search query's drive mode bits to an actual drive
  /// mode of a locomotive. Decides whether they match using tri-state logic,
  /// i.e. taking into account "no restriction" queries.
  ///
  /// @param event the incoming query.
  /// @param mode the drive mode of a locomotive.
  ///
  /// @return true if this locomotive matches the restrictions in the query
  /// (true if there were no restrictions in the query).
  static bool match_event_to_drive_mode(openlcb::EventId event,
                                        locodb::DriveMode mode);

  /// Compares an incoming search query to a given train node.
  ///
  /// @param event the incoming query.
  /// @param train @ref LocoDatabaseEntry to be queried.
  ///
  /// @return 0 for a no-match. Returns a bitfield of match types for a match.
  /// valid bits are MATCH_ANY (always set), ADDRESS_ONLY (set when the match
  /// occurred in the address), EXACT (clear for prefix match).
  static uint8_t match_query_to_node(openlcb::EventId event,
                                     locodb::LocoDatabaseEntry* train);

  /// Compares an incoming search query to a train node described by the major
  /// parameters only.
  ///
  /// @param event the incoming query.
  /// @param name the name to search for.
  /// @param address the address to search for.
  /// @param mode should be set to 0 for ignore, or DCC_LONG_ADDRESS.
  ///
  /// @return a bitfield of match types for a match. valid bits are MATCH_ANY
  /// (always set), ADDRESS_ONLY (set when the match occurred in the address),
  /// EXACT (clear for prefix match).
  static uint8_t match_query_to_train(openlcb::EventId event,
                                      const string& name, unsigned address,
                                      locodb::DriveMode mode);

  /// Converts a find protocol query to an address and desired @ref DriveMode
  /// information. Will take into account prefix zeros for forcing a dcc long
  /// address, as well as all mode and flag bits coming in via the query.
  ///
  /// @param event the incoming query.
  /// @param mode (can't be null) will be filled in with the Dcc Mode: the
  /// bottom 3 bits as specified by the incoming query, or zero if the query
  /// did not specify a preference. If the query started with a prefix of zero
  /// (typed by the user) or DCC_FORCE_LONG_ADDRESS was set in the query, the
  /// DriveMode will have the force long address bit set.
  ///
  /// @return the new legacy_address.
  static unsigned query_to_address(openlcb::EventId query,
                                   locodb::DriveMode* mode);

  /// Translates an address as punched in by a (dumb) throttle to a query to
  /// issue on the OpenLCB bus as a find protocol request.
  ///
  /// @param address is the numeric value that the user typed.
  /// @param exact should be true if only exact matches shall be retrieved.
  /// @param mode should be set most of the time to OLCBUSER to specify that we
  /// don't care about the address type, but can also be set to
  /// DCC_LONG_ADDRESS.
  ///
  /// @return an event ID representing the search. This event ID will be zero if
  /// the user input is invalid.
  static openlcb::EventId address_to_query(unsigned address, bool exact,
                                           locodb::DriveMode mode);

  /// Translates an address as punched in by a (dumb) throttle to a query to
  /// issue on the OpenLCB bus as a find protocol request. This method ensures
  /// that the allocate bit is set so that if the address (node) does not yet
  /// exist, it may be created.
  ///
  /// @param address is the numeric value that the user typed.
  /// @param exact should be true if only exact matches shall be retrieved.
  /// @param mode should be set most of the time to OLCBUSER to specify that we
  /// don't care about the address type, but can also be set to
  /// DCC_LONG_ADDRESS.
  ///
  /// @return an event ID representing the search. This event ID will be zero if
  /// the user input is invalid.
  static openlcb::EventId address_to_allocate(unsigned address, bool exact,
                                              locodb::DriveMode mode)
  {
    return address_to_query(address, exact, mode) | ALLOCATE;
  }

  /// Translates a sequence of input digits punched in by a throttle to a query
  /// to issue on the OpenLCB bus as a find protocol request.
  ///
  /// @param input is the sequence of numbers that the user typed. This is
  /// expected to have form like '415' or '021' or '474014'
  ///
  /// @return an event ID representing the search. This event ID could be
  /// IS_TRAIN_EVENT.
  static openlcb::EventId input_to_search(const string& input);

  /// Translates a sequence of input digits punched in by a throttle to an
  /// allocate request to issue on the OpenLCB bus.
  ///
  /// @param input is the sequence of numbers that the user typed. This is
  /// expected to have form like '415' or '021' or '474014'. You can add a
  /// leading zero to force DCC long address, a trailing M to force a Marklin
  /// locomotive.
  ///
  /// @return an event ID representing the search. This event ID will be zero if
  /// the user input is invalid.
  static openlcb::EventId input_to_allocate(const string& input);

  /// Specifies what kind of train to allocate when the drive mode is left as
  /// default / unspecified.
  static uint8_t DEFAULT_DRIVE_MODE;

  /// Specifies what kind of train to allocate when the drive mode is set as
  /// MARKLIN_ANY.
  static uint8_t DEFAULT_MARKLIN_DRIVE_MODE;

  /// Specifies what kind of train to allocate when the drive mode is set as
  /// DCC_ANY.
  static uint8_t DEFAULT_DCC_DRIVE_MODE;

 private:
  /// Helper function for the input_to_* calls.
  static openlcb::EventId input_to_event(const string& input);
  
  // Not instantiatable class.
  TrainSearchDefs();
};

} // namespace trainsearch

#endif  // _COMMANDSTATION_FINDPROTOCOLDEFS_HXX_

/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabase.hxx"
#include "trainsearch/Defs.hxx"

#include <cctype>
#include <openlcb/TractionDefs.hxx>
#include <utils/logging.h>

namespace trainsearch
{

#ifndef TSP_LOG_LEVEL
#ifdef CONFIG_TSP_LOGGING_LEVEL
#define TSP_LOG_LEVEL CONFIG_TSP_LOGGING_LEVEL
#else
#define TSP_LOG_LEVEL VERBOSE
#endif // CONFIG_TSP_LOGGING_LEVEL
#endif // TSP_LOG_LEVEL

using dcc::TrainAddressType;
using locodb::DriveMode;
using locodb::LocoDatabaseEntry;
using openlcb::EventId;
using openlcb::TractionDefs;

/// Specifies what kind of train to allocate when the drive mode is left as
/// default / unspecified.
uint8_t TrainSearchDefs::DEFAULT_DRIVE_MODE = DriveMode::DCC_128;

/// Specifies what kind of train to allocate when the drive mode is set as
/// MARKLIN_ANY.
uint8_t TrainSearchDefs::DEFAULT_MARKLIN_DRIVE_MODE =
  DriveMode::MARKLIN_NEW;

/// Specifies what kind of train to allocate when the drive mode is set as
/// DCC_ANY.
uint8_t TrainSearchDefs::DEFAULT_DCC_DRIVE_MODE = DriveMode::DCC_128;

namespace
{

/// @returns true if the user has any digits as a query.
static inline bool has_query(EventId event)
{
  for (int shift = TrainSearchDefs::TRAIN_FIND_MASK - 4;
       shift >= TrainSearchDefs::TRAIN_FIND_MASK_LOW; shift -= 4)
  {
    uint8_t nibble = (event >> shift) & 0xf;
    if ((0 <= nibble) && (nibble <= 9))
    {
      return true;
    }
  }
  return false;
}

/// @returns the same bitmask as match_query_to_node.
static inline uint8_t attempt_match(
  const string name, unsigned pos, EventId event)
{
  int count_matches = 0;
  for (int shift = TrainSearchDefs::TRAIN_FIND_MASK - 4;
       shift >= TrainSearchDefs::TRAIN_FIND_MASK_LOW; shift -= 4)
  {
    uint8_t nibble = (event >> shift) & 0xf;
    if ((0 <= nibble) && (nibble <= 9))
    {
      while (pos < name.size() && !std::isdigit(name[pos]))
      {
        ++pos;
      }
      if (pos > name.size())
      {
        return 0;
      }
      if ((name[pos] - '0') != nibble)
      {
        return 0;
      }
      ++pos;
      ++count_matches;
      continue;
    }
    else
    {
      continue;
    }
  }
  // If we are here, we have exhausted (and matched) all digits that were sent
  // by the query.
  if (count_matches == 0)
  {
    // Query had no digits. This is handled elsewhere.
    return 0;
  }
  // Look for more digits in the name.
  while (pos < name.size() && !std::isdigit(name[pos]))
  {
    ++pos;
  }
  if (pos < name.size())
  {
    if (event & TrainSearchDefs::EXACT)
    {
      return 0;
    }
    return TrainSearchDefs::MATCH_ANY;
  }
  else
  {
    return TrainSearchDefs::EXACT | TrainSearchDefs::MATCH_ANY;
  }
}

}  // namespace

class ExternalTrainDbEntry : public LocoDatabaseEntry
{
 public:
  ExternalTrainDbEntry(
    const string& name, uint16_t address, DriveMode mode = DriveMode::DCC_128)
    : LocoDatabaseEntry(name, name, address, mode)
  {

  }

  /// Retrieves the NMRAnet NodeID for the virtual node that represents a
  /// particular train known to the database.
  openlcb::NodeID get_traction_node() override
  {
    return 0;
  }

  /// Returns the largest valid function ID for this train, or -1 if the train
  /// has no functions.
  int get_max_fn() override
  {
    return 0;
  }
};

// static
bool TrainSearchDefs::match_event_to_drive_mode(
  EventId event, DriveMode mode)
{
  DriveMode req_mode =
    static_cast<DriveMode>(event & locodb::PROTOCOL_MASK);
  if (req_mode == DriveMode::DEFAULT)
  {
    return true;
  }
  if (mode == DriveMode::DEFAULT)
  {
    return true;
  }
  return locodb::drive_mode_to_protocol(mode) ==
         locodb::drive_mode_to_protocol(req_mode);
}

// static
unsigned TrainSearchDefs::query_to_address(
  EventId event, DriveMode* mode)
{
  unsigned supplied_address = 0;
  bool has_prefix_zero = false;
  for (int shift = TRAIN_FIND_MASK - 4; shift >= TRAIN_FIND_MASK_LOW;
       shift -= 4)
  {
    uint8_t nibble = (event >> shift) & 0xf;
    if (0 == nibble && 0 == supplied_address)
    {
      has_prefix_zero = true;
    }
    if ((0 <= nibble) && (nibble <= 9))
    {
      supplied_address *= 10;
      supplied_address += nibble;
      continue;
    }
    // For the moment we just ignore every non-numeric character. Including
    // gluing together all digits entered by the user into one big number.
  }
  uint8_t drive_type = event & DriveMode::PROTOCOL_MASK;
  if (event & ALLOCATE)
  {
    // If we are allocating, then we fill in defaults for drive modes.
    if (drive_type == DriveMode::DEFAULT)
    {
      drive_type = DEFAULT_DRIVE_MODE;
    }
    else if (drive_type == DriveMode::MARKLIN_DEFAULT)
    {
      drive_type = DEFAULT_MARKLIN_DRIVE_MODE;
    }
    else if (drive_type == DriveMode::DCC_DEFAULT)
    {
      drive_type = DEFAULT_DCC_DRIVE_MODE;
    }
    else if (drive_type == (DriveMode::DCC_DEFAULT |
                            DriveMode::DCC_LONG_ADDRESS))
    {
      drive_type |= (DEFAULT_DCC_DRIVE_MODE & DriveMode::DCC_SS_MASK);
    }
  }
  
  if (has_prefix_zero &&  //
      (((drive_type & DriveMode::DCC_ANY_MASK) == DriveMode::DCC_ANY) ||
        (drive_type == DriveMode::DEFAULT)))
  {
    drive_type |= DriveMode::DCC_DEFAULT;
    drive_type |= DriveMode::DCC_LONG_ADDRESS;
  }
  *mode = static_cast<DriveMode>(drive_type);
  return supplied_address;
}

// static
EventId TrainSearchDefs::address_to_query(
  unsigned address, bool exact, DriveMode mode)
{
  EventId event = EVENT_SEARCH_BASE;
  int shift = TRAIN_FIND_MASK_LOW;
  while (address)
  {
    event |= (address % 10) << shift;
    shift += 4;
    address /= 10;
  }
  while (shift < TRAIN_FIND_MASK)
  {
    event |= UINT64_C(0xF) << shift;
    shift += 4;
  }
  if (exact)
  {
    event |= EXACT;
  }
  event |= mode & DriveMode::PROTOCOL_MASK;
  return event;
}

// static
uint8_t TrainSearchDefs::match_query_to_node(
  EventId event, LocoDatabaseEntry* train)
{
  // empty search should match everything.
  if (event == TractionDefs::IS_TRAIN_EVENT)
  {
    return MATCH_ANY | ADDRESS_ONLY | EXACT;
  }
  unsigned legacy_address = train->get_legacy_address();
  DriveMode mode;
  unsigned supplied_address = query_to_address(event, &mode);
  bool req_has_query = has_query(event);
  bool has_address_prefix_match = false;
  auto actual_drive_mode = train->get_legacy_drive_mode();
  auto desired_address_type =
    locodb::drive_mode_to_address_type(mode, supplied_address);
  auto actual_address_type =
    locodb::drive_mode_to_address_type(actual_drive_mode, legacy_address);
  bool address_type_match =
      (actual_address_type == TrainAddressType::UNSUPPORTED ||
       desired_address_type == TrainAddressType::UNSPECIFIED ||
       desired_address_type == actual_address_type);
  if (!match_event_to_drive_mode(event, actual_drive_mode))
  {
    // The request specified a restriction on the locomotive mode and this
    // restriction does not match the current loco.
    return 0;
  }
  if (supplied_address == legacy_address)
  {
    if (address_type_match)
    {
      // If the caller did not specify the drive mode, or the drive mode
      // matches.
      return MATCH_ANY | ADDRESS_ONLY | EXACT;
    }
#if TSP_LOG_LEVEL >= VERBOSE
    else
    {
      LOG(TSP_LOG_LEVEL,
          "exact match failed due to mode: desired %d actual %d",
          static_cast<int>(desired_address_type),
          static_cast<int>(actual_address_type));
    }
#endif
    has_address_prefix_match = ((event & EXACT) == 0);
  }
  if (((event & EXACT) == 0) && address_type_match)\
  {
    // Search for the supplied number being a prefix of the existing addresses.
    unsigned address_prefix = legacy_address / 10;
    while (address_prefix)
    {
      if (address_prefix == supplied_address)
      {
        has_address_prefix_match = true;
        break;
      }
      address_prefix /= 10;
    }
  }
  if ((mode != DriveMode::DEFAULT) &&
      (event & EXACT) && (event & ALLOCATE) && (event & ADDRESS_ONLY))
  {
    // Request specified a drive mode and allocation. We check the drive mode
    // to match.
    if (desired_address_type != actual_address_type)
    {
      return 0;
    }
  }
  if (event & ADDRESS_ONLY)
  {
    if (((event & EXACT) != 0) || (!has_address_prefix_match))
    {
      return 0;
    }
    return MATCH_ANY | ADDRESS_ONLY;
  }
  if (!req_has_query)
  {
    if ((event & EXACT) != 0)
    {
      return 0;
    }
    return MATCH_ANY;
  }
  // Match against the train name string.
  uint8_t first_name_match = 0xFF;
  uint8_t best_name_match = 0;
  string name = train->get_train_name();
  // Find the beginning of numeric components in the train name
  unsigned pos = 0;
  while (pos < name.size())
  {
    if (std::isdigit(name[pos]))
    {
      uint8_t current_match = attempt_match(name, pos, event);
      if (first_name_match == 0xff)
      {
        first_name_match = current_match;
        best_name_match = current_match;
      }
      if ((!best_name_match && current_match) || (current_match & EXACT))
      {
        // We overwrite the best name match if there was no previous match, or
        // if the current match is exact. This is somewhat questionable,
        // because it will allow an exact match on the middle of the train name
        // even though there may have been numbers before. However, this is
        // arguably okay in case the train name is a model number and a cab
        // number, and we're matching against the cab number, e.g.
        // "Re 4/4 11239" which should be exact-matching the query 11239.
        best_name_match = current_match;
      }
      // Skip through the sequence of numbers
      while (pos < name.size() && std::isdigit(name[pos]))
      {
        ++pos;
      }
    }
    else
    {
      // non number: skip
      ++pos;
    }
  }
  if (first_name_match == 0xff)
  {
    // No numbers in the train name.
    best_name_match = 0;
  }
  if (((best_name_match & EXACT) == 0) && has_address_prefix_match &&
      ((event & EXACT) == 0))
  {
    // We prefer a partial address match over a non-exact name match.  If
    // address_prefix_match == true then the query had the EXACT bit false.
    return MATCH_ANY | ADDRESS_ONLY;
  }
  if ((event & EXACT) && !(best_name_match & EXACT))
  {
    return 0;
  }
  return best_name_match;
}

// static
EventId TrainSearchDefs::input_to_event(const string& input)
{
  EventId event = EVENT_SEARCH_BASE;
  int shift = TRAIN_FIND_MASK - 4;
  unsigned pos = 0;
  bool has_space = true;
  uint32_t qry = 0xFFFFFFFF;
  while (shift >= TRAIN_FIND_MASK_LOW && pos < input.size())
  {
    if (std::isdigit(input[pos]))
    {
      qry <<= 4;
      qry |= input[pos] - '0';
      has_space = false;
      shift -= 4;
    }
    else
    {
      if (!has_space)
      {
        qry <<= 4;
        qry |= 0xF;
        shift -= 4;
      }
      has_space = true;
    }
    pos++;
  }
  event |= uint64_t(qry & 0xFFFFFF) << TRAIN_FIND_MASK_LOW;
  unsigned flags = 0;
  if (input.size() > 2 && input[0] == '0' && input[1] == '0')
  {
    flags |= DriveMode::MARKLIN_NEW;
  }
  else if ((input[0] == '0') || (input.back() == 'L'))
  {
    flags |= DriveMode::DCC_ANY;
    flags |= DriveMode::DCC_LONG_ADDRESS;
  }
  else if (input.back() == 'M')
  {
    flags |= DriveMode::MARKLIN_NEW;
  }
  else if (input.back() == 'm')
  {
    flags |= DriveMode::MARKLIN_OLD;
  }
  else if (input.back() == 'S')
  {
    flags |= DriveMode::DCC_ANY;
  }
  event &= ~UINT64_C(0xff);
  event |= (flags & 0xff);

  return event;
}

EventId TrainSearchDefs::input_to_search(const string& input)
{
  if (input.empty())
  {
    return TractionDefs::IS_TRAIN_EVENT;
  }
  auto event = input_to_event(input);
  return event;
}

EventId TrainSearchDefs::input_to_allocate(const string& input)
{
  if (input.empty())
  {
    return 0;
  }
  auto event = input_to_event(input);
  event |= (TrainSearchDefs::ALLOCATE |
            TrainSearchDefs::EXACT |
            TrainSearchDefs::ADDRESS_ONLY);
  return event;
}

uint8_t TrainSearchDefs::match_query_to_train(
  EventId event, const string& name, unsigned address, DriveMode mode)
{
  ExternalTrainDbEntry entry(name, address, mode);
  return match_query_to_node(event, &entry);
}

} // namespace trainsearch
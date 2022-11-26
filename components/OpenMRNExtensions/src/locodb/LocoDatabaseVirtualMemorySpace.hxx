/*
 * SPDX-FileCopyrightText: 2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _LOCODB_LOCODATABASEVIRTUALMEMORYSPACE_HXX_
#define _LOCODB_LOCODATABASEVIRTUALMEMORYSPACE_HXX_

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabaseCDICommon.hxx"
#include "locodb/LocoDatabaseEntry.hxx"

#include <executor/Notifiable.hxx>
#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/constants.hxx>

namespace openlcb
{
    class SimpleStackBase;
}

namespace locodb
{
/// Memory space at which the locomotive database can be accessed via CDI.
static constexpr int VIRTUAL_MEMORYSPACE_SEGMENT = 0xBB;

/// Offset within the memory space.
static constexpr int VIRTUAL_MEMORYSPACE_OFFSET = 2048;

/// Definition for the locomotive database entries.
CDI_GROUP(TrainDatabaseSegment, Segment(VIRTUAL_MEMORYSPACE_SEGMENT),
          Offset(VIRTUAL_MEMORYSPACE_OFFSET));
/// Index of the loaded locomotive database entry.
CDI_GROUP_ENTRY(index, openlcb::Uint32ConfigEntry, Name("Entry Number"),
                Description("Locomotive Database entry number, write to this "
                            "value and refresh other fields to load current "
                            "values."));
/// Maximum valid index in the locomotive database.
CDI_GROUP_ENTRY(max_index, openlcb::Uint32ConfigEntry, Name("Maximum Entry Number"),
                Description("Maximum value Locomotive Database entry number, "
                            "read-only. Attempting to write to this value will be "
                            "ignored."));
/// Legacy address assigned to the loaded locomotive database entry.
CDI_GROUP_ENTRY(address, openlcb::Uint16ConfigEntry, Name("Address"),
                Description("Track protocol address of the train."),
                Default(0));
/// Legacy drive mode assigned to the loaded locomotive database entry.
CDI_GROUP_ENTRY(mode, openlcb::Uint8ConfigEntry, Name("Protocol"),
                Description("Protocol to use on the track for driving this train."),
                MapValues(DCC_DRIVE_MODE_MAP), Default(DriveMode::DCC_128));
/// Name assigned to the loaded locomotive database entry.
CDI_GROUP_ENTRY(name, openlcb::StringConfigEntry<63>, Name("Name"),
                Description("Identifies the train node on the LCC bus."));
/// Description assigned to the currently loaded locomotive database entry.
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<64>, Name("Description"),
                Description("Describes the train node on the LCC bus."));
/// Collection of all functions that are assigned to the loaded locomotive
/// database entry.
CDI_GROUP_ENTRY(functions, TrainCdiAllFunctionGroup);
CDI_GROUP_END();

/// Virtual memory space used for accessing the locomotive database using CDI
/// requests.
class LocoDatabaseVirtualMemorySpace : public openlcb::VirtualMemorySpace
{
public:
    /// Constructor.
    ///
    /// @param stack is the @ref SimpleStackBase that this memory space should
    /// be registered with.
    LocoDatabaseVirtualMemorySpace(openlcb::SimpleStackBase *stack);

private:
    /// @ref LocoDatabaseEntry that was loaded based on a write to @ref index_.
    std::shared_ptr<LocoDatabaseEntry> entry_;

    /// Index into the @ref LocoDatabase used by @ref entry_.
    size_t index_{0};

    /// Returns a function that can read numeric values from the
    /// @ref LocoDatabaseEntry that has been loaded.
    ///
    /// @param name is the name of the field being registered.
    /// @param offset is the offset into the memory space for the field.
    template <typename T>
    typename std::function<T(unsigned, BarrierNotifiable *)>
    numeric_reader(const char *name, int offset);

    /// Returns a function that can write numeric values from CDI clients,
    /// a write to the index will trigger a load of the corresponding
    /// @ref LocoDatabaseEntry.
    ///
    /// @param name is the name of the field being registered.
    /// @param offset is the offset into the memory space for the field.
    template <typename T>
    std::function<void(unsigned, T, BarrierNotifiable *)>
    numeric_writer(const char *name, int offset);

    /// Returns a function that can read string values from the
    /// @ref LocoDatabaseEntry that has been loaded.
    ///
    /// @param name is the name of the field being registered.
    /// @param offset is the offset into the memory space for the field.
    std::function<void(unsigned, string *, BarrierNotifiable *)>
    string_reader(const char *name, int offset);

    /// Returns a function that can write string values from CDI clients.
    ///
    /// @param name is the name of the field being registered.
    /// @param offset is the offset into the memory space for the field.
    std::function<void(unsigned, string, BarrierNotifiable *)>
    string_writer(const char *name, int offset);
};

} // namespace locodb

#endif // _LOCODB_LOCODATABASEVIRTUALMEMORYSPACE_HXX_
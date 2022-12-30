/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef _LOCODB_LOCODATABASECDICOMMON_HXX_
#define _LOCODB_LOCODATABASECDICOMMON_HXX_

#include <openlcb/ConfigRepresentation.hxx>

namespace locodb
{

/// Map definining the visible names and values for momentary or latching
/// locomotive functions.
static const char MOMENTARY_MAP[] =
    "<relation><property>0</property><value>Latching</value></relation>"
    "<relation><property>1</property><value>Momentary</value></relation>";

/// Map of locomotive function types and corresponding display name.
static const char FNDISPLAY_MAP[] =
    "<relation><property>0</property><value>Not Available</value></relation>"
    "<relation><property>1</property><value>Headlight</value></relation>"
    "<relation><property>4</property><value>Engine</value></relation>"
    "<relation><property>6</property><value>Announce</value></relation>"
    "<relation><property>7</property><value>Shunting Mode</value></relation>"
    "<relation><property>8</property><value>Momentum</value></relation>"
    "<relation><property>9</property><value>Uncouple</value></relation>"
    "<relation><property>10</property><value>Smoke</value></relation>"
    "<relation><property>11</property><value>Pantograph</value></relation>"
    "<relation><property>12</property><value>Far Light</value></relation>"
    "<relation><property>13</property><value>Bell</value></relation>"
    "<relation><property>14</property><value>Horn</value></relation>"
    "<relation><property>15</property><value>Whistle</value></relation>"
    "<relation><property>16</property><value>Light</value></relation>"
    "<relation><property>17</property><value>Mute</value></relation>"
    "<relation><property>127</property><value>Unknown</value></relation>"
    "<relation><property>255</property><value>Undefined</value></relation>";

/// Map of locomotive drive modes (protocols) and corresponding display names.
static const char DCC_DRIVE_MODE_MAP[] =
    "<relation><property>9</property><value>DCC (14 speed steps)</value></relation>"
    "<relation><property>10</property><value>DCC (28 speed steps)</value></relation>"
    "<relation><property>11</property><value>DCC (128 speed steps)</value></relation>"
    "<relation><property>13</property><value>DCC (14 speed steps, long address)</value></relation>"
    "<relation><property>14</property><value>DCC (28 speed steps, long address)</value></relation>"
    "<relation><property>15</property><value>DCC (128 speed steps, long address)</value></relation>"
    "<relation><property>4</property><value>Marklin (default)</value></relation>"
    "<relation><property>5</property><value>Marklin v1 (F0 only)</value></relation>"
    "<relation><property>6</property><value>Marklin v2 (F0-F4)</value></relation>"
    "<relation><property>7</property><value>Marklin v2+ (F0-F8, two addresses)</value></relation>";

/// Group definining a single locomotive function.
CDI_GROUP(TrainCdiFunctionGroup, Name("Functions"),
          Description("Defines what each function button does."));
/// This entry defines the icon or type for the function.
CDI_GROUP_ENTRY(icon, openlcb::Uint8ConfigEntry, Name("Display"),
                Description("Defines how throttles display this function."),
                Default(Function::NONEXISTANT),
                MapValues(FNDISPLAY_MAP));
/// This entry defines the function as latching or momentary.
CDI_GROUP_ENTRY(is_momentary, openlcb::Uint8ConfigEntry, Name("Momentary"),
                Description(
                    "Momentary functions are automatically turned off when you "
                    "release the respective button on the throttles."),
                MapValues(MOMENTARY_MAP), Default(0));
CDI_GROUP_END();

/// This defines a repeated group for locomotive functions 1-28.
using TrainCdiRepFunctionGroup =
    openlcb::RepeatedGroup<TrainCdiFunctionGroup, MAX_LOCO_FUNCTIONS - 1>;

/// Group defining locomotive function zero.
CDI_GROUP(F0Group, Name("F0"),
          Description("F0 is permanently assigned to the Headlight."));
/// This entry is an empty placeholder to keep all locomotive function
/// definitions in the CDI uniform in size.
CDI_GROUP_ENTRY(blank, openlcb::EmptyGroup<TrainCdiFunctionGroup::size()>);
CDI_GROUP_END();

/// Group defining all locomotive functions.
CDI_GROUP(TrainCdiAllFunctionGroup);
/// This entry is for locomotive function zero.
CDI_GROUP_ENTRY(f0, F0Group);
/// This entry if for all remaining locomotive functions.
CDI_GROUP_ENTRY(all_functions, TrainCdiRepFunctionGroup, RepName("Fn"));
CDI_GROUP_END();

} // namespace locodb

#endif // _LOCODB_LOCODATABASECDICOMMON_HXX_
/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef TRAINDB_CDI_HXX_
#define TRAINDB_CDI_HXX_

#include "locodb/Defs.hxx"
#include "locodb/LocoDatabaseCDICommon.hxx"

#include <openlcb/ConfigRepresentation.hxx>

namespace locodb
{

#if 0

static const char PROG_MODE_MAP_VALUES[] =
    "<relation><property>0</property><value>Disabled</value></relation>"
    "<relation><property>1</property><value>Direct mode</value></relation>"
    "<relation><property>2</property><value>POM mode</value></relation>"
    "<relation><property>10</property><value>Advanced mode</value></relation>";

CDI_GROUP(ProgrammingTrackSpaceConfigAdvanced);
CDI_GROUP_ENTRY(repeat_verify, openlcb::Uint8ConfigEntry,
                Name("Verify Packet Repeat Count"),
                Description("Number of times direct mode bit verification "
                            "packets need to be repeated for the DCC decoder "
                            "to generate an acknowledgement pulse."),
                Default(3), Min(0), Max(255));
CDI_GROUP_ENTRY(repeat_cooldown_reset, openlcb::Uint8ConfigEntry,
                Name("Reset Packet Count"),
                Description("Number of DCC reset packets to generate after "
                            "verification of CV value."),
                Default(6), Min(0), Max(255));
CDI_GROUP_END();

CDI_GROUP(ProgrammingTrackSpaceConfig,
          Segment(openlcb::MemoryConfigDefs::SPACE_DCC_CV),
          Name("Programming track operation"),
          Description("Use this component to read and write CVs on the "
                      "programming track of the command station."));
CDI_GROUP_ENTRY(mode, openlcb::Uint8ConfigEntry, Name("Mode"),
                MapValues(PROG_MODE_MAP_VALUES),
                Default(0), Min(0), Max(128));
CDI_GROUP_ENTRY(cv, openlcb::Uint16ConfigEntry, Name("CV Number"),
                Description("Number of CV to read or write (1..1024)."),
                Default(0), Min(0), Max(1024));
CDI_GROUP_ENTRY(value, openlcb::Uint16ConfigEntry, Name("CV Value"),
                Description("Set 'Mode' and 'CV Number' first, then: hit "
                            "'Refresh' to read the entire CV, or enter a "
                            "value and hit 'Write' to set the CV."),
                Default(0), Min(0), Max(255));
CDI_GROUP_ENTRY(bit_write_value, openlcb::Uint16ConfigEntry,
                Name("Bit change"),
                Description("Set 'Mode' and 'CV Number' first, then: write "
                            "1064 to set the single bit whose value is 64, or "
                            "2064 to clear that bit. Write 100 to 107 to set "
                            "bit index 0 to 7, or 200 to 207 to clear bit 0 to 7."
                            "Values outside of these two ranges do nothing."),
                Default(1000), Min(100), Max(2128));
CDI_GROUP_ENTRY(bit_value_string, openlcb::StringConfigEntry<24>,
                Name("Read bits decomposition"),
                Description("Hit Refresh on this line after reading a CV value "
                            "to see which bits are set."));
CDI_GROUP_ENTRY(advanced, ProgrammingTrackSpaceConfigAdvanced,
                Name("Advanced settings"));
CDI_GROUP_END();
#endif

CDI_GROUP(TrainSegment, Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG));
CDI_GROUP_ENTRY(address, openlcb::Uint16ConfigEntry, Name("Address"),
                Description("Track protocol address of the train."),
                Default(0));
CDI_GROUP_ENTRY(mode, openlcb::Uint8ConfigEntry, Name("Protocol"),
                Description("Protocol to use on the track for driving this train."),
                MapValues(DCC_DRIVE_MODE_MAP), Default(DriveMode::DCC_128));
CDI_GROUP_ENTRY(name, openlcb::StringConfigEntry<63>, Name("Name"),
                Description("Identifies the train node on the LCC bus."));
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<64>, Name("Description"),
                Description("Describes the train node on the LCC bus."));
CDI_GROUP_ENTRY(fn, TrainCdiAllFunctionGroup);
CDI_GROUP_END();

CDI_GROUP(TrainConfigDef, MainCdi());
// We do not support ACDI and we do not support adding the <identification>
// information in here because both of these vary train by train.
CDI_GROUP_ENTRY(ident, openlcb::Identification, Model("Virtual train node"));
CDI_GROUP_ENTRY(train, TrainSegment, Name("Train Settings"));
#if 0
CDI_GROUP_ENTRY(cv, ProgrammingTrackSpaceConfig);
#endif
CDI_GROUP_END();

CDI_GROUP(TmpTrainSegment, Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG),
          Offset(0),
          Description("This train is not part of the train database, thus no "
                      "configuration settings can be changed on it."));
CDI_GROUP_END();

/// This alternate CDI for a virtual train node will be in use for trains that
/// are not coming from the database. It will not offer any settings for the
/// user.
CDI_GROUP(TrainTmpConfigDef, MainCdi());
CDI_GROUP_ENTRY(ident, openlcb::Identification, Model("Virtual Train Node"));
CDI_GROUP_ENTRY(train, TmpTrainSegment, Name("Non-persistent train settings"));
#if 0
CDI_GROUP_ENTRY(cv, ProgrammingTrackSpaceConfig);
#endif
CDI_GROUP_END();

const char TRAINCONFIGDEF_CDI_DATA[] = R"xmlpayload(<?xml version="1.0" encoding="utf-8"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>Virtual Train Node</manufacturer>
<model>Virtual Train Node</model>
<hardwareVersion>1.0</hardwareVersion>
<softwareVersion>1.0</softwareVersion>
</identification>
<segment space='253'>
<name>Train Settings</name>
<description>Configures a single train</description>
<int size='2'>
<name>Address</name>
<description>Track protocol address of the train.</description>
<default>0</default>
</int>
<int size='1'>
<name>Protocol</name>
<description>Protocol to use on the track for driving this train.</description>
<default>11</default>
<map><relation><property>9</property><value>DCC (14 speed steps)</value></relation><relation><property>10</property><value>DCC (28 speed steps)</value></relation><relation><property>11</property><value>DCC (128 speed steps)</value></relation><relation><property>13</property><value>DCC (14 speed steps, long address)</value></relation><relation><property>14</property><value>DCC (28 speed steps, long address)</value></relation><relation><property>15</property><value>DCC (128 speed steps, long address)</value></relation><relation><property>4</property><value>Marklin (default)</value></relation><relation><property>5</property><value>Marklin v1 (F0 only)</value></relation><relation><property>6</property><value>Marklin v2 (F0-F4)</value></relation><relation><property>7</property><value>Marklin v2+ (F0-F8, two addresses)</value></relation></map>
</int>
<string size='63'>
<name>Name</name>
<description>Identifies the train node on the LCC bus.</description>
</string>
<string size='64'>
<name>Description</name>
<description>Describes the train node on the LCC bus.</description>
</string>
<group>
<group>
<name>F0</name>
<description>F0 is permanently assigned to the Headlight.</description>
<group offset='2'/>
</group>
<group replication='28'>
<name>Functions</name>
<description>Defines what each function button does.</description>
<repname>Fn</repname>
<int size='1'>
<name>Display</name>
<description>Defines how throttles display this function.</description>
<default>0</default>
<map><relation><property>0</property><value>Not Available</value></relation><relation><property>1</property><value>Headlight</value></relation><relation><property>4</property><value>Engine</value></relation><relation><property>6</property><value>Announce</value></relation><relation><property>7</property><value>Shunting Mode</value></relation><relation><property>8</property><value>Momentum</value></relation><relation><property>9</property><value>Uncouple</value></relation><relation><property>10</property><value>Smoke</value></relation><relation><property>11</property><value>Pantograph</value></relation><relation><property>12</property><value>Far Light</value></relation><relation><property>13</property><value>Bell</value></relation><relation><property>14</property><value>Horn</value></relation><relation><property>15</property><value>Whistle</value></relation><relation><property>16</property><value>Light</value></relation><relation><property>17</property><value>Mute</value></relation><relation><property>127</property><value>Unknown</value></relation><relation><property>255</property><value>Undefined</value></relation></map>
</int>
<int size='1'>
<name>Momentary</name>
<description>Momentary functions are automatically turned off when you release the respective button on the throttles.</description>
<default>0</default>
<map><relation><property>0</property><value>Latching</value></relation><relation><property>1</property><value>Momentary</value></relation></map>
</int>
</group>
</group>
</segment>)xmlpayload"
#if 0
R"xmlpayload(<segment space='248'>
<name>Programming track operation</name>
<description>Use this component to read and write CVs on the programming track of the command station.</description>
<int size='1'>
<name>Mode</name>
<min>0</min>
<max>128</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Direct mode</value></relation><relation><property>2</property><value>POM mode</value></relation><relation><property>10</property><value>Advanced mode</value></relation></map>
</int>
<int size='2'>
<name>CV Number</name>
<description>Number of CV to read or write (1..1024).</description>
<min>0</min>
<max>1024</max>
<default>0</default>
</int>
<int size='2'>
<name>CV Value</name>
<description>Set 'Mode' and 'CV Number' first, then: hit 'Refresh' to read the entire CV, or enter a value and hit 'Write' to set the CV.</description>
<min>0</min>
<max>255</max>
<default>0</default>
</int>
<int size='2'>
<name>Bit change</name>
<description>Set 'Mode' and 'CV Number' first, then: write 1064 to set the single bit whose value is 64, or 2064 to clear that bit. Write 100 to 107 to set bit index 0 to 7, or 200 to 207 to clear bit 0 to 7.Values outside of these two ranges do nothing.</description>
<min>100</min>
<max>2128</max>
<default>1000</default>
</int>
<string size='24'>
<name>Read bits decomposition</name>
<description>Hit Refresh on this line after reading a CV value to see which bits are set.</description>
</string>
<group>
<name>Advanced settings</name>
<int size='1'>
<name>Verify Packet Repeat Count</name>
<description>Number of times direct mode bit verification packets need to be repeated for the DCC decoder to generate an acknowledgement pulse.</description>
<min>0</min>
<max>255</max>
<default>3</default>
</int>
<int size='1'>
<name>Reset Packet Count</name>
<description>Number of DCC reset packets to generate after verification of CV value.</description>
<min>0</min>
<max>255</max>
<default>6</default>
</int>
</group></segment>)xmlpayload"
#endif
R"xmlpayload(</cdi>
)xmlpayload";

const size_t TRAINCONFIGDEF_CDI_SIZE = sizeof(TRAINCONFIGDEF_CDI_DATA);

const char TRAINTMPCONFIGDEF_CDI_DATA[] = R"xmlpayload(<?xml version="1.0" encoding="utf-8"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>Virtual Train Node</manufacturer>
<model>Virtual Train Node</model>
<hardwareVersion>1.0</hardwareVersion>
<softwareVersion>1.0</softwareVersion>
</identification>
<segment space='253'>
<name>Non-persistent train settings</name>
<description>This train is not part of the train database, thus no configuration settings can be changed on it.</description>
</segment>
)xmlpayload"
#if 0
R"xmlpayload(<segment space='248'>
<name>Programming track operation</name>
<description>Use this component to read and write CVs on the programming track of the command station.</description>
<int size='1'>
<name>Mode</name>
<min>0</min>
<max>128</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Direct mode</value></relation><relation><property>2</property><value>POM mode</value></relation><relation><property>10</property><value>Advanced mode</value></relation></map>
</int>
<int size='2'>
<name>CV Number</name>
<description>Number of CV to read or write (1..1024).</description>
<min>0</min>
<max>1024</max>
<default>0</default>
</int>
<int size='2'>
<name>CV Value</name>
<description>Set 'Mode' and 'CV Number' first, then: hit 'Refresh' to read the entire CV, or enter a value and hit 'Write' to set the CV.</description>
<min>0</min>
<max>255</max>
<default>0</default>
</int>
<int size='2'>
<name>Bit change</name>
<description>Set 'Mode' and 'CV Number' first, then: write 1064 to set the single bit whose value is 64, or 2064 to clear that bit. Write 100 to 107 to set bit index 0 to 7, or 200 to 207 to clear bit 0 to 7.Values outside of these two ranges do nothing.</description>
<min>100</min>
<max>2128</max>
<default>1000</default>
</int>
<string size='24'>
<name>Read bits decomposition</name>
<description>Hit Refresh on this line after reading a CV value to see which bits are set.</description>
</string>
<group>
<name>Advanced settings</name>
<int size='1'>
<name>Verify Packet Repeat Count</name>
<description>Number of times direct mode bit verification packets need to be repeated for the DCC decoder to generate an acknowledgement pulse.</description>
<min>0</min>
<max>255</max>
<default>3</default>
</int>
<int size='1'>
<name>Reset Packet Count</name>
<description>Number of DCC reset packets to generate after verification of CV value.</description>
<min>0</min>
<max>255</max>
<default>6</default>
</int>
</group></segment>)xmlpayload"
#endif
R"xmlpayload(</cdi>
)xmlpayload";
const size_t TRAINTMPCONFIGDEF_CDI_SIZE = sizeof(TRAINTMPCONFIGDEF_CDI_DATA);

}  // namespace locodb

#endif  // TRAINDB_CDI_HXX_
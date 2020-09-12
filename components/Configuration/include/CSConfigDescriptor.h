/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef CS_CDI_H_
#define CS_CDI_H_

#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/TractionCvCdi.hxx>
#include <freertos_drivers/esp32/Esp32WiFiConfiguration.hxx>
#include <TrackOutputDescriptor.h>
#include "ThermalConfiguration.hxx"

namespace esp32cs
{
    using TrackOutputs = openlcb::RepeatedGroup<TrackOutputConfig, 2>;

    /// Defines the main segment in the configuration CDI. This is laid out at
    /// origin 128 to give space for the ACDI user data at the beginning.
    CDI_GROUP(CommandStationSegment,
              Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG),
              Offset(128));
    /// Each entry declares the name of the current entry, then the type and
    /// then optional arguments list.
    CDI_GROUP_ENTRY(internal_config, openlcb::InternalConfigData);
    /// LCC WiFi configuration.
    CDI_GROUP_ENTRY(wifi_lcc, WiFiConfiguration, Name("LCC WiFi Configuration"));
    /// H-Bridge configuration.
    CDI_GROUP_ENTRY(hbridge, TrackOutputs, Name("H-Bridge Configuration"));
    /// Thermal Monitoring configuration.
    CDI_GROUP_ENTRY(thermal, ThermalConfiguration, Name("Thermal Configuration"));
    CDI_GROUP_END();

    /// This segment is only needed temporarily until there is program code to set
    /// the ACDI user data version byte.
    CDI_GROUP(VersionSeg, Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG),
        Name("Version information"));
    CDI_GROUP_ENTRY(acdi_user_version, openlcb::Uint8ConfigEntry,
        Name("ACDI User Data version"),
        Description("Set to 2 and do not change."));
    CDI_GROUP_END();

    /// The main structure of the ESP32 Command Station CDI.
    CDI_GROUP(Esp32ConfigDef, MainCdi());
    /// Adds the <identification> tag with the values from SNIP_STATIC_DATA
    /// above.
    CDI_GROUP_ENTRY(ident, openlcb::Identification);
    /// Adds an <acdi> tag.
    CDI_GROUP_ENTRY(acdi, openlcb::Acdi);
    /// Adds a segment for changing the values in the ACDI user-defined
    /// space. UserInfoSegment is defined in the system header.
    CDI_GROUP_ENTRY(userinfo, openlcb::UserInfoSegment);
    /// Adds the main configuration segment.
    CDI_GROUP_ENTRY(seg, CommandStationSegment);
    /// Adds the versioning segment.
    CDI_GROUP_ENTRY(version, VersionSeg);
    CDI_GROUP_END();
}

#endif // CS_CDI_H_

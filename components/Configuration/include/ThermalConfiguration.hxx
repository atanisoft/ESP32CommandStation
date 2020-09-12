/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#ifndef THERMAL_CONFIGURATION_HXX_
#define THERMAL_CONFIGURATION_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace esp32cs
{

/// Thermal Configuration for an external temperature sensor.
CDI_GROUP(ThermalConfiguration);

/// This is the warning temperature in celsius.
CDI_GROUP_ENTRY(temperature_warning, openlcb::Int8ConfigEntry,
    Default(50), Min(0), Max(125),
    Name("Warning Temperature"),
    Description("Temperature (in celsius) to use for thermal warning."));

/// This is the shutdown temperature in celsius.
CDI_GROUP_ENTRY(temperature_shutdown, openlcb::Int8ConfigEntry,
    Default(80), Min(0), Max(125),
    Name("Shutdown Temperature"),
    Description("Temperature (in celsius) at which to shutdown active "
                "monitoring and switch to low-power mode until temperatures "
                "return to below the shutdown level."));

/// This event will be produced when the temperature is above the warning
/// temperature but below the shutdown temperature.
CDI_GROUP_ENTRY(event_warning, openlcb::EventConfigEntry, //
    Name("Warning Temperature Exceeded"),
    Description("This event will be produced when the temperature has exceeded "
                "the warning temperature but is below the shutdown temperature."));

/// This event will be produced when the temperature drops below the warning
/// temperature.
CDI_GROUP_ENTRY(event_warning_clear, openlcb::EventConfigEntry, //
    Name("Warning Temperature (cleared)"),
    Description("This event will be produced when the temperature has dropped "
                "below the warning temperature.\n"
                "Note: This event will not be generated if the warning "
                "temperature has not been exceeded previously."));

/// This event will be produced when the temperature is above the shutdown
/// temperature.
CDI_GROUP_ENTRY(event_shutdown, openlcb::EventConfigEntry, //
    Name("Shutdown Temperature Exceeded"),
    Description("This event will be produced when the temperature has exceeded "
                "the shutdown temperature or there is a failure in reading the "
                "temperature."));

/// This event will be produced when the temperature drops below the shutdown
/// temperature.
CDI_GROUP_ENTRY(event_shutdown_clear, openlcb::EventConfigEntry, //
    Name("Shutdown (Cleared)"),
    Description("This event will be produced when the temperature has dropped "
                "below the shutdown temperature.\n"
                "Note: This event will not be generated if the shutdown "
                "temperature has not been exceeded previously."));

CDI_GROUP_END();

} // namespace esp32s2io

#endif // THERMAL_CONFIGURATION_HXX_
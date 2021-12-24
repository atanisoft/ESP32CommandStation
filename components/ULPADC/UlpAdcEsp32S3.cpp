/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2021 Mike Dunston

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

#include "UlpAdc.hxx"
#include "sdkconfig.h"

namespace esp32cs
{

/// Initialize and start the ULP co-processor for monitoring current sense ADC
/// inputs. When thresholds are breached the ULP will raise an interrupt to the
/// ESP32 SoC. When the interrupt is triggered the ESP32 SoC will evaluate the
/// current state and raise event(s) as needed.
void initialize_ulp_adc()
{

}

void deinitialize_ulp_adc()
{
}

uint16_t get_last_ops_reading()
{
  return 4095;
}

uint32_t get_ops_load()
{
  return -1;
}

uint16_t get_ops_short_threshold()
{
  return 4095;
}

uint16_t get_ops_shutdown_threshold()
{
  return 4095;
}

uint16_t get_ops_warning_threshold()
{
  return 4095;
}

uint16_t get_last_prog_reading()
{
  return 4095;
}

uint16_t get_last_tempsensor_reading()
{
  return 4095;
}

} // namespace esp32cs
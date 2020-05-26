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

#ifndef TRACK_POWER_BIT_INTERFACE_H_
#define TRACK_POWER_BIT_INTERFACE_H_

#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/Node.hxx>
#include <os/Gpio.hxx>

namespace esp32cs
{

class TrackPowerBit : public openlcb::BitEventInterface
{
public:
  TrackPowerBit(openlcb::Node *node, const Gpio *gpio)
    : openlcb::BitEventInterface(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT
                               , openlcb::Defs::EMERGENCY_OFF_EVENT)
    , node_(node)
    , gpio_(gpio)
  {
  }

  openlcb::EventState get_current_state() override
  {
    if (gpio_->is_set())
    {
      return openlcb::EventState::VALID;
    }
    return openlcb::EventState::INVALID;
  }
  
  void set_state(bool new_value) override;

  openlcb::Node *node()
  {
    return node_;
  }

private:
  openlcb::Node *node_;
  const Gpio *gpio_;
};

} // namespace esp32cs

#endif // TRACK_POWER_BIT_INTERFACE_H_
/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

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

#ifndef OUTPUTS_H_
#define OUTPUTS_H_

#include <DCCppProtocol.h>
#include <driver/gpio.h>
#include <utils/StringPrintf.hxx>

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(OutputCommandAdapter, "Z", 0)
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(OutputExCommandAdapter, "Zex", 1)

const uint8_t OUTPUT_IFLAG_INVERT = BIT0;
const uint8_t OUTPUT_IFLAG_RESTORE_STATE = BIT1;
const uint8_t OUTPUT_IFLAG_FORCE_STATE = BIT2;

class Output
{
public:
  Output(uint16_t, gpio_num_t, uint8_t);
  Output(std::string &);
  std::string set(bool=false, bool=true);
  void update(gpio_num_t, uint8_t);
  std::string toJson(bool=false);
  uint16_t getID()
  {
    return _id;
  }
  gpio_num_t getPin()
  {
    return _pin;
  }
  uint8_t getFlags()
  {
    return _flags;
  }
  bool isActive()
  {
    return _active;
  }
  std::string get_state_for_dccpp()
  {
    return StringPrintf("<Y %d %d>", _id, !_active);
  }
  std::string getStateAsJson();
  std::string getFlagsAsString()
  {
    std::string flags = "";
    if((_flags & OUTPUT_IFLAG_INVERT) == OUTPUT_IFLAG_INVERT)
    {
      flags += "activeLow";
    }
    else
    {
      flags += "activeHigh";
    }
    if((_flags & OUTPUT_IFLAG_RESTORE_STATE) == OUTPUT_IFLAG_RESTORE_STATE)
    {
      if((_flags & OUTPUT_IFLAG_FORCE_STATE) == OUTPUT_IFLAG_FORCE_STATE)
      {
        flags += ",force(on)";
      }
      else
      {
        flags += ",force(off)";
      }
    }
    else
    {
      flags += ",restoreState";
    }
    return flags;
  }
private:
  uint16_t _id;
  gpio_num_t _pin;
  uint8_t _flags;
  bool _active;
};

class OutputManager
{
  public:
    static void init();
    static void clear();
    static uint16_t store();
    static std::string set(uint16_t, bool=false);
    static Output *getOutput(uint16_t);
    static bool toggle(uint16_t);
    static std::string getStateAsJson();
    static std::string get_state_for_dccpp();
    static bool createOrUpdate(const uint16_t, const gpio_num_t, const uint8_t);
    static bool remove(const uint16_t);
};

#endif // OUTPUTS_H_
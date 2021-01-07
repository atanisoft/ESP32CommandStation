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

#include "sdkconfig.h"

#if defined(CONFIG_GPIO_OUTPUTS)

#include <FileSystemManager.h>
#include <DCCppProtocol.h>
#include <JsonConstants.h>
#include <driver/gpio.h>

#include "GPIOValidation.h"
#include "Outputs.h"

std::vector<std::unique_ptr<Output>> outputs;
#include <json.hpp>

static constexpr const char * OUTPUTS_JSON_FILE = "outputs.json";

void OutputManager::init()
{
  LOG(INFO, "[Output] Initializing outputs");
  nlohmann::json root = nlohmann::json::parse(
    Singleton<FileSystemManager>::instance()->load(OUTPUTS_JSON_FILE));
  if(root.contains(JSON_COUNT_NODE))
  {
    for(auto output : root[JSON_OUTPUTS_NODE])
    {
      string data = output.dump();
      outputs.push_back(std::make_unique<Output>(data));
    }
  }
  LOG(INFO, "[Output] Loaded %d outputs", outputs.size());
}

void OutputManager::clear()
{
  outputs.clear();
}

uint16_t OutputManager::store()
{
  nlohmann::json root;
  uint16_t outputStoredCount = 0;
  for (const auto& output : outputs)
  {
    root[JSON_OUTPUTS_NODE].push_back(output->toJson());
    outputStoredCount++;
  }
  root[JSON_COUNT_NODE] = outputStoredCount;
  Singleton<FileSystemManager>::instance()->store(OUTPUTS_JSON_FILE
                                                , root.dump());
  return outputStoredCount;
}

string OutputManager::set(uint16_t id, bool active)
{
  for (const auto& output : outputs)
  {
    if(output->getID() == id)
    {
      return output->set(active);
    }
  }
  return COMMAND_FAILED_RESPONSE;
}

Output *OutputManager::getOutput(uint16_t id)
{
  auto const &ent = std::find_if(outputs.begin(), outputs.end(),
  [id](std::unique_ptr<Output> & output) -> bool
  {
    return output->getID() == id;
  });
  if (ent != outputs.end())
  {
    return ent->get();
  }
  return nullptr;
}

bool OutputManager::toggle(uint16_t id)
{
  auto output = getOutput(id);
  if (output)
  {
    output->set(!output->isActive());
    return true;
  }
  return false;
}

std::string OutputManager::getStateAsJson()
{
  string state = "[";
  for (const auto& output : outputs)
  {
    if (state.length() > 1)
    {
      state += ",";
    }
    state += output->toJson(true);
  }
  state += "]";
  return state;
}

string OutputManager::get_state_for_dccpp()
{
  string status;
  for (const auto& output : outputs)
  {
    status += output->get_state_for_dccpp();
  }
  return status;
}

bool OutputManager::createOrUpdate(const uint16_t id, const gpio_num_t pin, const uint8_t flags)
{
  for (const auto& output : outputs)
  {
    if(output->getID() == id)
    {
      output->update(pin, flags);
      return true;
    }
  }
  if(is_restricted_pin(pin))
  {
    return false;
  }
  outputs.push_back(std::make_unique<Output>(id, pin, flags));
  return true;
}

bool OutputManager::remove(const uint16_t id)
{
  const auto & ent = std::find_if(outputs.begin(), outputs.end(),
  [id](std::unique_ptr<Output> & output) -> bool
  {
    return output->getID() == id;
  });
  if (ent != outputs.end())
  {
    LOG(INFO, "[Output] Removing Output(%d)", (*ent)->getID());
    outputs.erase(ent);
    return true;
  }
  return false;
}

Output::Output(uint16_t id, gpio_num_t pin, uint8_t flags) : _id(id), _pin(pin), _flags(flags), _active(false)
{
  gpio_pad_select_gpio(_pin);
  ESP_ERROR_CHECK(gpio_set_direction(_pin, GPIO_MODE_OUTPUT));

  if((_flags & OUTPUT_IFLAG_RESTORE_STATE) == OUTPUT_IFLAG_RESTORE_STATE)
  {
    if((_flags & OUTPUT_IFLAG_FORCE_STATE) == OUTPUT_IFLAG_FORCE_STATE)
    {
      set(true, false);
    }
    else
    {
      set(false, false);
    }
  }
  else
  {
    set(false, false);
  }
  LOG(CONFIG_GPIO_OUTPUT_LOG_LEVEL
    , "[Output] Output(%d) on pin %d created, flags: %s"
    , _id, _pin, getFlagsAsString().c_str());
}

Output::Output(string &data)
{
  nlohmann::json object = nlohmann::json::parse(data);
  _id = object[JSON_ID_NODE].get<uint16_t>();
  _pin = (gpio_num_t)object[JSON_PIN_NODE].get<uint8_t>();
  _flags = object[JSON_FLAGS_NODE].get<uint8_t>();
  gpio_pad_select_gpio((gpio_num_t)_pin);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)_pin, GPIO_MODE_OUTPUT));
  if((_flags & OUTPUT_IFLAG_RESTORE_STATE) == OUTPUT_IFLAG_RESTORE_STATE)
  {
    set((_flags & OUTPUT_IFLAG_FORCE_STATE) == OUTPUT_IFLAG_FORCE_STATE, false);
  }
  else
  {
    set(object[JSON_STATE_NODE].get<bool>(), false);
  }
  LOG(CONFIG_GPIO_OUTPUT_LOG_LEVEL
    , "[Output] Output(%d) on pin %d loaded, flags: %s"
    , _id, _pin, getFlagsAsString().c_str());
}

string Output::set(bool active, bool announce)
{
  _active = active;
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)_pin, _active));
  LOG(INFO, "[Output] Output(%d) set to %s", _id
    , _active ? JSON_VALUE_ON : JSON_VALUE_OFF);
  if(announce)
  {
    return StringPrintf("<Y %d %d>", _id, !_active);
  }
  return COMMAND_NO_RESPONSE;
}

void Output::update(gpio_num_t pin, uint8_t flags)
{
  // reset the current pin
  ESP_ERROR_CHECK(gpio_reset_pin(_pin));
  _pin = pin;
  _flags = flags;
  // setup the new pin
  gpio_pad_select_gpio(_pin);
  ESP_ERROR_CHECK(gpio_set_direction(_pin, GPIO_MODE_OUTPUT));
  if((_flags & OUTPUT_IFLAG_RESTORE_STATE) != OUTPUT_IFLAG_RESTORE_STATE)
  {
    set(false, false);
  }
  else
  {
    set((_flags & OUTPUT_IFLAG_FORCE_STATE) == OUTPUT_IFLAG_FORCE_STATE, false);
  }
  LOG(CONFIG_GPIO_OUTPUT_LOG_LEVEL
    , "[Output] Output(%d) on pin %d updated, flags: %s", _id, _pin
    , getFlagsAsString().c_str());
}

string Output::toJson(bool readableStrings)
{
  nlohmann::json object =
  {
    { JSON_ID_NODE, _id },
    { JSON_PIN_NODE, (uint8_t)_pin },
  };
  if(readableStrings)
  {
    object[JSON_FLAGS_NODE] = getFlagsAsString();
    object[JSON_STATE_NODE] = isActive() ? JSON_VALUE_ON : JSON_VALUE_OFF;
  }
  else
  {
    object[JSON_FLAGS_NODE] = _flags;
    object[JSON_STATE_NODE] = _active;
  }
  return object.dump();
}

string Output::getStateAsJson()
{
  return StringPrintf("<Y %d %d %d %d>", _id, _pin, _flags, !_active);
}

/**********************************************************************
  <Z ID PIN IFLAG>: creates a new output ID, with specified PIN and IFLAG values.
                    if output ID already exists, it is updated with specificed
                    PIN and IFLAG.
                    Note: output state will be immediately set to ACTIVE/INACTIVE
                    and pin will be set to HIGH/LOW according to IFLAG value
                    specifcied (see below).
        returns: <O> if successful and <X> if unsuccessful (e.g. out of memory)

  <Z ID>:           deletes definition of output ID
        returns: <O> if successful and <X> if unsuccessful (e.g. ID does not exist)

  <Z>:              lists all defined output pins
        returns: <Y ID PIN IFLAG STATE> for each defined output pin or <X> if no
        output pins defined

where

  ID: the numeric ID (0-32767) of the output
  PIN: the pin number to use for the output
  STATE: the state of the output (0=INACTIVE / 1=ACTIVE)
  IFLAG: defines the operational behavior of the output based on bits 0, 1, and
  2 as follows:

    IFLAG, bit 0:   0 = forward operation (ACTIVE=HIGH / INACTIVE=LOW)
                    1 = inverted operation (ACTIVE=LOW / INACTIVE=HIGH)

    IFLAG, bit 1:   0 = state of pin restored on power-up to either ACTIVE or
                        INACTIVE depending on state before power-down; state of
                        pin set to INACTIVE when first created.
                    1 = state of pin set on power-up, or when first created, to
                        either ACTIVE of INACTIVE depending on IFLAG, bit 2.

    IFLAG, bit 2:   0 = state of pin set to INACTIVE upon power-up or when
                        first created.
                    1 = state of pin set to ACTIVE upon power-up or when
                        first created.

To change the state of outputs that have been defined use:

  <Z ID STATE>:     sets output ID to either ACTIVE or INACTIVE state
                    returns: <Y ID STATE>, or <X> if turnout ID does not exist
where
  ID: the numeric ID (0-32767) of the turnout to control
  STATE: the state of the output (0=INACTIVE / 1=ACTIVE)

**********************************************************************/

DCC_PROTOCOL_COMMAND_HANDLER(OutputCommandAdapter,
[](const vector<string> arguments)
{
  if(arguments.empty())
  {
    // list all outputs
    return OutputManager::get_state_for_dccpp();
  }
  else
  {
    uint16_t outputID = std::stoi(arguments[0]);
    if (arguments.size() == 1 && OutputManager::remove(outputID))
    {
      // delete output
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
    else if (arguments.size() == 2)
    {
      // set output state
      return OutputManager::set(outputID, arguments[1][0] == 1);
    }
    else if (arguments.size() == 3)
    {
      // create output
      OutputManager::createOrUpdate(outputID
                                  , (gpio_num_t)std::stoi(arguments[1])
                                  , std::stoi(arguments[2]));
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

DCC_PROTOCOL_COMMAND_HANDLER(OutputExCommandAdapter,
[](const vector<string> arguments)
{
  uint16_t outputID = std::stoi(arguments[0]);
  auto output = OutputManager::getOutput(outputID);
  if (output)
  {
    return output->set(!output->isActive());
  }
  return COMMAND_FAILED_RESPONSE;
})
#endif // CONFIG_GPIO_OUTPUTS
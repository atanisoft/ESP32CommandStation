/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 Mike Dunston

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

#include "ESP32CommandStation.h"

#include <json.hpp>

using nlohmann::json;

/**********************************************************************
ESP32 COMMAND STATION supports multiple Locomotive Consists, using either
command station consisting or decoder assisted consisting.

When either the leading or trailing locmotive is directly addressed all
locomotives in the consist will respond accordingly based on the configuration
of the consist. Therefore, the front light and rear lights of the lead and trail
locmotives will be active based on direction of travel and orientation of the
locomotive. As example:
CONSIST: 10
LEAD   : 4567 (FWD)
TRAIL  : 8765 (REV)
OTHER  : 1234 (REV), 2456 (REV)
When consist 10 moves in the FWD direction (in relation to lead loco) the
following functions will be active:
4567 FL
8765 NONE
1234 NONE
2456 NONE

When consist 10 moves in the REV direction (in relation to lead loco) the
following functions will be active:
4567 NONE
8765 FR
1234 NONE
2456 NONE

Configuration of advanced consist:
each locomotive will have the following CV values:
CV19 consist ID
CV21 ALL BITs ON
CV22 BIT 0 OFF, ALL others ON
CV29 BIT 0 set based on consist orientation.
CV29 BIT 1 set to ON (control FL via F0 only)

Configuration for command station managed consist:
NO CV changes, when consist is addressed (either by LEAD or TRAIL loco), all
locomotives in consist will be updated concurrently via multiple packet queuing.
**********************************************************************/
/*
// TODO remove this constant
static constexpr const char * CONSIST_ENTRY_JSON_FILE = "consist-%d.json";

LocomotiveConsist::~LocomotiveConsist()
{
  releaseLocomotives();
  string filename = StringPrintf(CONSIST_ENTRY_JSON_FILE, legacy_address());
  if(configStore->exists(filename.c_str())) {
    configStore->remove(filename.c_str());
  }
}

string LocomotiveConsist::get_state_for_dccpp() {
  // <U ID LEAD TRAIL [{OTHER}]>
  auto speed = get_speed();
  LOG(INFO, "[Consist %d] speed: %d, direction: %s, decoderAssisted: %s"
    , legacy_address(), (speed.get_dcc_128() & 0x7F)
    , speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD
    , _decoderAssisstedConsist ? JSON_VALUE_TRUE : JSON_VALUE_FALSE);
  string statusCmd = StringPrintf("<U %d", legacy_address() * _decoderAssisstedConsist ? -1 : 1);
  for (const auto& loco : _locos) {
    LOG(INFO, "LOCO: %d, ORIENTATION: %s", loco->legacy_address(),
      loco->isOrientationForward() ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE);
    statusCmd += StringPrintf(" %d", loco->legacy_address() * loco->isOrientationForward() ? 1 : -1);
  }
  statusCmd += ">";
  return statusCmd;
}

string LocomotiveConsist::toJson(bool includeFunctions) {
  json object;
  auto speed = get_speed();
  object[JSON_ADDRESS_NODE] = legacy_address();
  object[JSON_SPEED_NODE] = (speed.get_dcc_128() & 0x7F);
  object[JSON_DIRECTION_NODE] = speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD;
  object[JSON_ORIENTATION_NODE] = isOrientationForward() ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE;
  object[JSON_CONSIST_NODE] = JSON_VALUE_TRUE;
  object[JSON_DECODER_ASSISTED_NODE] = _decoderAssisstedConsist ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
  for (const auto& loco : _locos)
  {
    object[JSON_LOCOS_NODE].push_back(loco->toJson(includeFunctions));
  }
  return object.dump();
}

LocomotiveConsist *LocomotiveConsist::fromJson(string &content, openlcb::TrainService *trainService)
{
  json object = json::parse(content);
  LocomotiveConsist * consist =
    new LocomotiveConsist(object[JSON_ADDRESS_NODE].get<uint16_t>()
                        , trainService
                        , !object[JSON_DECODER_ASSISTED_NODE].get<string>().compare(JSON_VALUE_TRUE));
  for(auto member : object[JSON_LOCOS_NODE])
  {
    string content;
    if (member.contains(JSON_FILE_NODE))
    {
      content = member[JSON_FILE_NODE].dump();
    }
    else
    {
      content = read_file_to_string(member[JSON_FILE_NODE].get<string>());
    }
    consist->_locos.push_back(Locomotive::fromJson(content, trainService));
  }
  return consist;
}

bool LocomotiveConsist::isAddressInConsist(uint16_t locoAddress)
{
  for (const auto& loco : _locos)
  {
    if (loco->legacy_address() == locoAddress)
    {
      return true;
    }
  }
  return false;
}

string LocomotiveConsist::updateThrottle(uint16_t locoAddress, int8_t speed, bool forward)
{
  auto req_speed = get_speed();
  req_speed.set_dcc_128(speed);
  req_speed.set_direction(forward ? dcc::SpeedType::FORWARD : dcc::SpeedType::REVERSE);
  if (!_decoderAssisstedConsist)
  {
    // if it is a basic consist then sending a throttle request to any
    // locomotive in the consist will cause all locomotives to update
    for (const auto& loco : _locos)
    {
      loco->set_speed(req_speed);
    }
  }
  else if (_locos[0]->legacy_address() == locoAddress ||
           _locos[1]->legacy_address() == locoAddress ||
           legacy_address() == locoAddress)
  {
    // only if we are addressing the lead or trail locomotive should we react to
    // the throttle adjustment
    set_speed(req_speed);
  }
  return get_state_for_dccpp();
}

void LocomotiveConsist::addLocomotive(uint16_t locoAddress
                                    , bool forward
                                    , uint8_t position)
{
  Locomotive *loco = locoManager->getLocomotive(locoAddress, false);
  loco->setOrientationForward(forward);
  _locos.push_back(loco);
  if(_decoderAssisstedConsist)
  {
    // write the loco consist address
    if (forward)
    {
      writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_ADDRESS, legacy_address());
    }
    else
    {
      // if the locomotive is in reverse orientation set bit 7 on the consist
      // address to inform the decoder of this change
      // see s-9.2.2 CV 19 details
      writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_ADDRESS,
        legacy_address() + CONSIST_ADDRESS_REVERSED_ORIENTATION);
    }
    // toggle FL/FR based on position, if it is the lead or trail locomotive
    // enable the function.
    if (position <= 1)
    {
      _locos[position]->set_fn(0, true);
      writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
        CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::FL_BIT, false);
    }
    else
    {
      _locos[position]->set_fn(0, false);
      writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
        CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::FL_BIT, true);
    }
    // enable F1-F8 for the consist address
    writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_F1_F8, 0xFF);
    // enable F9-F12 for the consist address
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F9_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F10_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F11_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F12_BIT, true);
  }
}

bool LocomotiveConsist::removeLocomotive(uint16_t address)
{
  auto entry = std::find_if(_locos.begin(), _locos.end(),
    [address](Locomotive *loco)
    {
      return loco->legacy_address() == address;
    }
  );
  if(entry != _locos.end())
  {
    _locos.erase(entry);
    if(_decoderAssisstedConsist)
    {
      // if we are in an advanced consist, send a programming packet to clear
      // the consist address from the decoder
      writeOpsCVByte(address, CV_NAMES::CONSIST_ADDRESS, CONSIST_ADDRESS_NO_ADDRESS);
    }
    return true;
  }
  return false;
}

void LocomotiveConsist::releaseLocomotives()
{
  for(auto ent : _locos)
  {
    delete ent;
  }
  _locos.clear();
}
*/
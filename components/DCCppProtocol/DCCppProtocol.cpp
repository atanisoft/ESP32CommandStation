/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

The DCC++ protocol specification is
COPYRIGHT (c) 2013-2016 Gregg E. Berman
and has been adapter for use in ESP32 COMMAND STATION.

**********************************************************************/

#include "DCCppProtocol.h"
#include "DCCProgrammer.h"

#include "sdkconfig.h"

#include <algorithm>
#include <AllTrainNodes.hxx>
#include <LCCStackManager.h>
#include <DCCSignalVFS.h>
#include <esp_ota_ops.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <memory>
#include <HttpStringUtils.h>
#include <openlcb/SimpleStack.hxx>
#if CONFIG_GPIO_OUTPUTS
#include <Outputs.h>
#endif // CONFIG_GPIO_OUTPUTS
#if CONFIG_GPIO_SENSORS
#include <Sensors.h>
#include <RemoteSensors.h>
#if CONFIG_GPIO_S88
#include <S88Sensors.h>
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS
#include <Turnouts.h>

std::vector<std::unique_ptr<DCCPPProtocolCommand>> registeredCommands;

// <R {CV} {CALLBACK} {CALLBACK-SUB}> command handler, this command attempts
// to read a CV value from the PROGRAMMING track. The returned value will be
// the actual CV value or -1 when there is a failure reading or verifying the
// CV.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ReadCVCommand, "R", 3)
DCC_PROTOCOL_COMMAND_HANDLER(ReadCVCommand,
[](const vector<string> arguments)
{
  uint16_t cv = std::stoi(arguments[0]);
  uint16_t callback = std::stoi(arguments[1]);
  uint16_t callbackSub = std::stoi(arguments[2]);
  int16_t value = readCV(cv);
  return StringPrintf("<r%d|%d|%d %d>",
    callback,
    callbackSub,
    cv,
    value);
})

// <W {CV} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this command
// attempts to write a CV value on the PROGRAMMING track. The returned value
// is either the actual CV value written or -1 if there is a failure writing or
// verifying the CV value.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(WriteCVByteProgCommand, "W", 4)
DCC_PROTOCOL_COMMAND_HANDLER(WriteCVByteProgCommand,
[](const vector<string> arguments)
{
  uint16_t cv = std::stoi(arguments[0]);
  int16_t value = std::stoi(arguments[1]);
  uint16_t callback = std::stoi(arguments[2]);
  uint16_t callbackSub = std::stoi(arguments[3]);
  if (!writeProgCVByte(cv, value))
  {
    LOG_ERROR("[PROG] Failed to write CV %d as %d", cv, value);
    value = -1;
  }
  return StringPrintf("<r%d|%d|%d %d>", callback, callbackSub, cv, value);
})

// <W {CV} {BIT} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this
// command attempts to write a single bit value for a CV on the PROGRAMMING
// track. The returned value is either the actual bit value of the CV or -1 if
// there is a failure writing or verifying the CV value.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(WriteCVBitProgCommand, "B", 5)
DCC_PROTOCOL_COMMAND_HANDLER(WriteCVBitProgCommand,
[](const vector<string> arguments)
{
  int cv = std::stoi(arguments[0]);
  uint8_t bit = std::stoi(arguments[1]);
  int8_t value = std::stoi(arguments[2]);
  uint16_t callback = std::stoi(arguments[3]);
  uint16_t callbackSub = std::stoi(arguments[4]);
  if (!writeProgCVBit(cv, bit, value))
  {
    LOG_ERROR("[PROG] Failed to write CV %d BIT %d as %d", cv, bit, value);
    value = -1;
  }
  return StringPrintf("<r%d|%d|%d %d %d>", callback, callbackSub, cv, bit
                    , value);
})

// <w {LOCO} {CV} {VALUE}> command handler, this command sends a CV write packet
// on the MAIN OPERATIONS track for a given LOCO. No verification is attempted.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(WriteCVByteOpsCommand, "w", 3)
DCC_PROTOCOL_COMMAND_HANDLER(WriteCVByteOpsCommand,
[](const vector<string> arguments)
{
  writeOpsCVByte(std::stoi(arguments[0]), std::stoi(arguments[1])
               , std::stoi(arguments[2]));
  return COMMAND_SUCCESSFUL_RESPONSE;
})

// <b {LOCO} {CV} {BIT} {VALUE}> command handler, this command sends a CV bit
// write packet on the MAIN OPERATIONS track for a given LOCO. No verification
// is attempted.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(WriteCVBitOpsCommand, "b", 4)
DCC_PROTOCOL_COMMAND_HANDLER(WriteCVBitOpsCommand,
[](const vector<string> arguments)
{
  writeOpsCVBit(std::stoi(arguments[0]), std::stoi(arguments[1])
              , std::stoi(arguments[2]), arguments[3][0] == '1');
  return COMMAND_SUCCESSFUL_RESPONSE;
})

string convert_loco_to_dccpp_state(openlcb::TrainImpl *impl, size_t id)
{
  dcc::SpeedType speed(impl->get_speed());
  if (speed.mph())
  {
    return StringPrintf("<T %d %d %d>", id, (int)speed.mph() + 1
                      , speed.direction() == dcc::SpeedType::FORWARD);
  }
  return StringPrintf("<T %d 0 %d>", id
                    , speed.direction() == dcc::SpeedType::FORWARD);
}

// <F> command handler, this command sends the current free heap space as response.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(FreeHeapCommand, "F", 0)
DCC_PROTOCOL_COMMAND_HANDLER(FreeHeapCommand,
[](const vector<string> arguments)
{
  return StringPrintf("<f %d>", os_get_free_heap());
})

// <estop> command handler, this command sends an estop packet to all active
// locomotives.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(EStopCommand, "estop", 0)
DCC_PROTOCOL_COMMAND_HANDLER(EStopCommand,
[](const vector<string> arguments)
{
  esp32cs::initiate_estop();
  return COMMAND_SUCCESSFUL_RESPONSE;
})

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(CurrentDrawCommand, "c", 0)
DCC_PROTOCOL_COMMAND_HANDLER(CurrentDrawCommand,
[](const vector<string> arguments)
{
  return esp32cs::get_track_state_for_dccpp();
})

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(PowerOnCommand, "1", 0)
DCC_PROTOCOL_COMMAND_HANDLER(PowerOnCommand,
[](const vector<string> arguments)
{
  esp32cs::enable_ops_track_output();
  // hardcoded response since enable/disable is deferred until the next
  // check interval.
  return StringPrintf("<p1 %s>", CONFIG_OPS_TRACK_NAME);
})

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(PowerOffCommand, "0", 0)
DCC_PROTOCOL_COMMAND_HANDLER(PowerOffCommand,
[](const vector<string> arguments)
{
  esp32cs::disable_track_outputs();
  // hardcoded response since enable/disable is deferred until the next
  // check interval.
  return StringPrintf("<p0 %s>", CONFIG_OPS_TRACK_NAME);
})

#define GET_LOCO_VIA_EXECUTOR(NAME, address)                                          \
  openlcb::TrainImpl *NAME = nullptr;                                                 \
  {                                                                                   \
    SyncNotifiable n;                                                                 \
    Singleton<esp32cs::LCCStackManager>::instance()->stack()->executor()->add(        \
    new CallbackExecutable([&]()                                                      \
    {                                                                                 \
      NAME = Singleton<commandstation::AllTrainNodes>::instance()->get_train_impl(    \
                                        commandstation::DccMode::DCC_128, address);   \
      n.notify();                                                                     \
    }));                                                                              \
    n.wait_for_notification();                                                        \
  }

// <t {REGISTER} {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ThrottleCommandAdapter, "t", 4)
DCC_PROTOCOL_COMMAND_HANDLER(ThrottleCommandAdapter,
[](const vector<string> arguments)
{
  int reg_num = std::stoi(arguments[0]);
  uint16_t loco_addr = std::stoi(arguments[1]);
  uint8_t req_speed = std::stoi(arguments[2]);
  uint8_t req_dir = std::stoi(arguments[3]);

  GET_LOCO_VIA_EXECUTOR(impl, loco_addr);
  LOG(INFO, "[DCC++ loco %d] Set speed to %d", loco_addr, req_speed);
  LOG(INFO, "[DCC++ loco %d] Set direction to %s", loco_addr
      , req_dir ? "FWD" : "REV");
  auto speed = dcc::SpeedType::from_mph(req_speed);
  if (!req_dir)
  {
    speed.set_direction(dcc::SpeedType::REVERSE);
  }
  impl->set_speed(speed);
  return convert_loco_to_dccpp_state(impl, reg_num);
});

// <tex {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ThrottleExCommandAdapter, "tex", 3)
DCC_PROTOCOL_COMMAND_HANDLER(ThrottleExCommandAdapter,
[](const vector<string> arguments)
{
  uint16_t loco_addr = std::stoi(arguments[0]);
  int8_t req_speed = std::stoi(arguments[1]);
  int8_t req_dir = std::stoi(arguments[2]);

  GET_LOCO_VIA_EXECUTOR(impl, loco_addr);

  if (req_speed >= 0)
  {
    auto speed = dcc::SpeedType::from_mph(req_speed);
    if (req_dir == 0)
    {
      speed.set_direction(dcc::SpeedType::REVERSE);
    }
    LOG(INFO, "[DCC++ loco %d] Set speed to %d (%s)", loco_addr, abs(req_speed)
      , req_speed > 0 ? "FWD" : "REV");
    impl->set_speed(speed);
  }
  else if (req_dir >= 0)
  {
    dcc::SpeedType speed(impl->get_speed());
    LOG(INFO, "[DCC++ loco %d] Set direction to %s", loco_addr
      , req_dir ? "FWD" : "REV");
    speed.set_direction(req_dir ? dcc::SpeedType::FORWARD : dcc::SpeedType::REVERSE);
    impl->set_speed(speed);
  }
  return convert_loco_to_dccpp_state(impl, 0);
})

// <f {LOCO} {BYTE} [{BYTE2}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(FunctionCommandAdapter, "f", 2)
DCC_PROTOCOL_COMMAND_HANDLER(FunctionCommandAdapter,
[](const vector<string> arguments)
{
  uint16_t loco_addr = std::stoi(arguments[0]);
  uint8_t func_byte = std::stoi(arguments[1]);
  uint8_t first{1};
  uint8_t last{4};
  uint8_t bits{func_byte};

  GET_LOCO_VIA_EXECUTOR(impl, loco_addr);

  // check this is a request for functions F13-F28
  if(arguments.size() > 2)
  {
    bits = std::stoi(arguments[2]);
    if((func_byte & 0xDE) == 0xDE)
    {
      first = 13;
      last = 20;
    }
    else
    {
      first = 21;
      last = 28;
    }
  }
  else
  {
    // this is a request for functions FL,F1-F12
    // for safety this guarantees that first nibble of function byte will always
    // be of binary form 10XX which should always be the case for FL,F1-F12
    if((func_byte & 0xB0) == 0xB0)
    {
      first = 5;
      last = 8;
    }
    else if((func_byte & 0xA0) == 0xA0)
    {
      first = 9;
      last = 12;
    }
    else
    {
      impl->set_fn(0, func_byte & BIT(4));
    }
  }
  for(uint8_t id = first; id <= last; id++)
  {
    LOG(INFO, "[DCC++ loco %d] Set function %d to %d", loco_addr, id
      , (int)(bits & BIT(id - first)));
    impl->set_fn(id, bits & BIT(id - first));
  }
  return COMMAND_NO_RESPONSE;
});

// <fex {LOCO} {FUNC} {STATE}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(FunctionExCommandAdapter, "fex", 3)
DCC_PROTOCOL_COMMAND_HANDLER(FunctionExCommandAdapter,
[](const vector<string> arguments)
{
  int loco_addr = std::stoi(arguments[0]);
  int function = std::stoi(arguments[1]);
  int state = std::stoi(arguments[2]);

  GET_LOCO_VIA_EXECUTOR(impl, loco_addr);
  LOG(INFO, "[DCC++ loco %d] Set function %d to %d", loco_addr, function
    , state);
  impl->set_fn(function, state);
  return COMMAND_NO_RESPONSE;
});

// wrapper to handle the following command structures:
// CREATE: <C {ID} {LEAD LOCO} {TRAIL LOCO}  [{OTHER LOCO}]>
// DELETE: <C {ID} {LOCO}>
// DELETE: <C {ID}>
// QUERY : <C 0 {LOCO}>
// SHOW  : <C>
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ConsistCommandAdapter, "C", 0)
DCC_PROTOCOL_COMMAND_HANDLER(ConsistCommandAdapter,
[](const vector<string> arguments)
{
  // TODO: reimplement
  /*
  if (arguments.empty())
  {
    return locoManager->getConsistStateAsDCCpp();
  }
  else if (arguments.size() == 1 &&
           locoManager->removeLocomotiveConsist(std::stoi(arguments[1])))
  {
    return COMMAND_SUCCESSFUL_RESPONSE;
  }
  else if (arguments.size() == 2)
  {
    int8_t consistAddress = std::stoi(arguments[0]);
    uint16_t locomotiveAddress = std::stoi(arguments[1]);
    if (consistAddress == 0)
    {
      // query which consist loco is in
      auto consist = locoManager->getConsistForLoco(locomotiveAddress);
      if (consist)
      {
        return StringPrintf("<V %d %d>",
          consist->legacy_address() * consist->isDecoderAssistedConsist() ? -1 : 1,
          locomotiveAddress);
      }
    }
    else
    {
      // remove loco from consist
      auto consist = locoManager->getConsistByID(consistAddress);
      if (consist->isAddressInConsist(locomotiveAddress))
      {
        consist->removeLocomotive(locomotiveAddress);
        return COMMAND_SUCCESSFUL_RESPONSE;
      }
    }
    // if we get here either the query or remove failed
    return COMMAND_FAILED_RESPONSE;
  }
  else if (arguments.size() >= 3)
  {
    // create or update consist
    uint16_t consistAddress = std::stoi(arguments[0]);
    auto consist = locoManager->getConsistByID(consistAddress);
    if (consist)
    {
      // existing consist, need to update
      consist->releaseLocomotives();
    }
    else
    {
      // verify if all provided locos are not already in a consist
      for(int index = 1; index < arguments.size(); index++)
      {
        int32_t locomotiveAddress = std::stoi(arguments[index]);
        if(locoManager->isAddressInConsist(abs(locomotiveAddress)))
        {
          LOG_ERROR("[Consist] Locomotive %d is already in a consist.", abs(locomotiveAddress));
          return COMMAND_FAILED_RESPONSE;
        }
      }
      consist = locoManager->createLocomotiveConsist(consistAddress);
      if(!consist)
      {
        LOG_ERROR("[Consist] Unable to create new Consist");
        return COMMAND_FAILED_RESPONSE;
      }
    }
    // add locomotives to consist
    for(int index = 1; index < arguments.size(); index++)
    {
      int32_t locomotiveAddress = std::stoi(arguments[index]);
      consist->addLocomotive(abs(locomotiveAddress), locomotiveAddress > 0,
        index - 1);
    }
    return COMMAND_SUCCESSFUL_RESPONSE;
  }
  */
  return COMMAND_FAILED_RESPONSE;
})

/*
  <T ID BOARD INDEX>:          creates a new turnout ID, with specified BOARD
                               and INDEX. If turnout ID already exists, it is
                               updated with specificed BOARD and INDEX
      returns: <O> if successful and <X> if unsuccessful (ie: out of memory)

  <T ID>:                      deletes definition of turnout ID
      returns: <O> if successful and <X> if unsuccessful (ie: ID does not exist)

  <T>:                         lists all defined turnouts
      returns: <H ID ADDRESS SUBADDRESS THROW> for each defined turnout or <X>
               if no turnouts defined
  <T ID THROW>:                sets turnout ID to either the "thrown" or
                               "unthrown" position
      returns: <H ID THROW>, or <X> if turnout ID does not exist
where
  ID:         the numeric ID (0-32767) of the turnout to control
  BOARD:      the primary address of the decoder controlling this turnout
              (0-511)
  INDEX:      the subaddress of the decoder controlling this turnout (0-3)
  THROW:      0 (unthrown) or 1 (thrown)
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(TurnoutCommandAdapter, "T", 0)
DCC_PROTOCOL_COMMAND_HANDLER(TurnoutCommandAdapter,
[](const vector<string> arguments)
{
  auto turnoutManager = Singleton<TurnoutManager>::instance();
  if (arguments.empty())
  {
    // list all turnouts
    return turnoutManager->get_state_for_dccpp();
  }
  else
  {
    // index starts at one, reduce the index parameter by one.
    uint16_t index = std::stoi(arguments[0]) - 1;
    if (arguments.size() == 1)
    {
      auto turnout = turnoutManager->getByIndex(index);
      if (turnout && turnoutManager->remove(turnout->getAddress()))
      {
        // delete turnout
        return COMMAND_SUCCESSFUL_RESPONSE;
      }
    }
    else if (arguments.size() == 2)
    {
      // throw turnout
      auto turnout = turnoutManager->getByIndex(index);
      if (turnout)
      {
        turnout->set(std::stoi(arguments[1]));
        return COMMAND_SUCCESSFUL_RESPONSE;
      }
    }
    else if (arguments.size() == 3)
    {
      // board can be 0-511 and port 0-3, however board 0 is problematic
      int16_t board = std::stoi(arguments[1]);
      int8_t port = std::stoi(arguments[2]);
      // validate input parameters and reject values that are out of range
      if (board <= 0 || board > 511 || port < 0 || port > 3)
      {
        LOG_ERROR("[DCC++ T] Rejecting invalid board(%d), port(%d)", board
                , port);
        return COMMAND_FAILED_RESPONSE;
      }
      // create/update turnout
      uint16_t addr = decodeDCCAccessoryAddress(board, port);
      if (addr == 0 || addr > 2044)
      {
        LOG_ERROR("[DCC++ T] Address %d is out of range, rejecting", addr);
        return COMMAND_FAILED_RESPONSE;
      }
      LOG(VERBOSE, "[DCC++ T] decoded %d:%d to %d", board, port, addr);
      turnoutManager->createOrUpdate(addr);
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

/*
  <Tex ADDRESS>:         Toggle turnout.
  <Tex ADDRESS TYPE>:    Create/Update a Turnout
  all will return : <O> if successful and <X> if unsuccessful.

where
  ADDRESS:    the DCC decoder address for the turnout
  TYPE:       turnout type:
              0 : LEFT
              1 : RIGHT
              2 : WYE
              3 : MULTI
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(TurnoutExCommandAdapter, "Tex", 1)
DCC_PROTOCOL_COMMAND_HANDLER(TurnoutExCommandAdapter,
[](const vector<string> arguments)
{
  if (!arguments.empty())
  {
    uint16_t addr = std::stoi(arguments[0]);
    if (addr == 0 || addr > 2044)
    {
      LOG_ERROR("[DCC++ Turnout] Address %d is out of range, rejecting", addr);
      return COMMAND_FAILED_RESPONSE;
    }
    if (arguments.size() == 1)
    {
      return Singleton<TurnoutManager>::instance()->toggle(addr);
    }
    TurnoutType type = (TurnoutType)std::stoi(arguments[1]);
    if (Singleton<TurnoutManager>::instance()->createOrUpdate(addr, type))
    {
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

/*
 <a BOARD INDEX THROW>: Throws a turnout (accessory decoder)
      returns: <H ID THROW>

Note: When this is received a Turnout will be created based on the decoded
DCC address for the accessory decoder.
where
  BOARD:      the primary address of the decoder controlling this turnout
              (0-511)
  INDEX:      the subaddress of the decoder controlling this turnout (0-3)
  THROW:      0 (unthrown) or 1 (thrown)
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(AccessoryCommand, "a", 3)
DCC_PROTOCOL_COMMAND_HANDLER(AccessoryCommand,
[](const vector<string> arguments)
{
  return Singleton<TurnoutManager>::instance()->set(
      decodeDCCAccessoryAddress(std::stoi(arguments[0])
                              , std::stoi(arguments[1]))
    , std::stoi(arguments[2])
  );
})


// <e> command handler, this command will clear all stored Turnouts, Outputs,
// Sensors and S88 Sensors (if enabled) after sending this command. Note, when
// running with the PCB configuration only turnouts will be cleared.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ConfigErase, "e", 0)
DCC_PROTOCOL_COMMAND_HANDLER(ConfigErase,
[](const vector<string> arguments)
{
  Singleton<TurnoutManager>::instance()->clear();
#if CONFIG_GPIO_SENSORS
  SensorManager::clear();
  SensorManager::store();
#if CONFIG_GPIO_S88
  S88BusManager::instance()->clear();
  S88BusManager::instance()->store();
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS
#if CONFIG_GPIO_OUTPUTS
  OutputManager::clear();
  OutputManager::store();
#endif // CONFIG_GPIO_OUTPUTS
  return COMMAND_SUCCESSFUL_RESPONSE;
})

// <E> command handler, this command stores all currently defined Turnouts,
// Sensors, S88 Sensors (if enabled) and Outputs to persistent storage for use
// by the Command Station in subsequent startups. Note, when running with the
// PCB configuration only turnouts will be stored.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ConfigStore, "E", 0)
DCC_PROTOCOL_COMMAND_HANDLER(ConfigStore,
[](const vector<string> arguments)
{
  return StringPrintf("<e %d %d %d>"
                    , Singleton<TurnoutManager>::instance()->count()
#if CONFIG_GPIO_SENSORS
                    , SensorManager::store()
#if CONFIG_GPIO_S88
                    + S88BusManager::instance()->store()
#endif // CONFIG_GPIO_S88
#else
                    , 0
#endif // CONFIG_GPIO_SENSORS
#if CONFIG_GPIO_OUTPUTS
                    , OutputManager::store()
#else
                    , 0
#endif // CONFIG_GPIO_OUTPUTS
    );
})

// <s> command handler, this command sends the current status for all parts of
// the ESP32 Command Station. JMRI uses this command as a keep-alive heartbeat
// command.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(StatusCommand, "s", 0)
DCC_PROTOCOL_COMMAND_HANDLER(StatusCommand,
[](const vector<string> arguments)
{
  wifi_mode_t mode;
  const esp_app_desc_t *app_data = esp_ota_get_app_description();
  string status = StringPrintf("<iDCC++ ESP32 Command Station: V-%s / %s %s>"
                , app_data->version, app_data->date, app_data->time);
  status += esp32cs::get_track_state_for_dccpp();
  auto trains = Singleton<commandstation::AllTrainNodes>::instance();
  for (size_t id = 0; id < trains->size(); id++)
  {
    auto nodeid = trains->get_train_node_id(id);
    if (nodeid)
    {
      auto impl = trains->get_train_impl(nodeid);
      status += convert_loco_to_dccpp_state(impl, id);
    }
  }
  status += Singleton<TurnoutManager>::instance()->get_state_for_dccpp();
#if CONFIG_GPIO_OUTPUTS
  status += OutputManager::get_state_for_dccpp();
#endif // CONFIG_GPIO_OUTPUTS
  if (esp_wifi_get_mode(&mode) == ESP_OK)
  {
    tcpip_adapter_ip_info_t ip_info;
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)
    {
      if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info) == ESP_OK)
      {
        status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
      }
      if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info) == ESP_OK)
      {
        status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
      }
    }
    else if (mode != WIFI_MODE_NULL &&
             tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info) == ESP_OK)
    {
      status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
    }
  }
  return status;
})

void DCCPPProtocolHandler::init()
{
  registerCommand(new ThrottleCommandAdapter());
  registerCommand(new ThrottleExCommandAdapter());
  registerCommand(new FunctionCommandAdapter());
  registerCommand(new FunctionExCommandAdapter());
  registerCommand(new ConsistCommandAdapter());
  registerCommand(new AccessoryCommand());
  registerCommand(new PowerOnCommand());
  registerCommand(new PowerOffCommand());
  registerCommand(new CurrentDrawCommand());
  registerCommand(new StatusCommand());
  registerCommand(new ReadCVCommand());
  registerCommand(new WriteCVByteProgCommand());
  registerCommand(new WriteCVBitProgCommand());
  registerCommand(new WriteCVByteOpsCommand());
  registerCommand(new WriteCVBitOpsCommand());
  registerCommand(new ConfigErase());
  registerCommand(new ConfigStore());
#if defined(CONFIG_GPIO_OUTPUTS)
  registerCommand(new OutputCommandAdapter());
  registerCommand(new OutputExCommandAdapter());
#endif
  registerCommand(new TurnoutCommandAdapter());
  registerCommand(new TurnoutExCommandAdapter());
#if defined(CONFIG_GPIO_SENSORS)
  registerCommand(new SensorCommandAdapter());
#if defined(CONFIG_GPIO_S88)
  registerCommand(new S88BusCommandAdapter());
#endif
  registerCommand(new RemoteSensorsCommandAdapter());
#endif
  registerCommand(new FreeHeapCommand());
  registerCommand(new EStopCommand());
}

string DCCPPProtocolHandler::process(const string &commandString)
{
  vector<string> parts;
  http::tokenize(commandString, parts);
  string commandID = parts.front();
  parts.erase(parts.begin());
  LOG(VERBOSE, "Command: %s, argument count: %d", commandID.c_str()
    , parts.size());
  auto handler = getCommandHandler(commandID);
  if (handler)
  {
    if (parts.size() >= handler->getMinArgCount())
    {
      return handler->process(parts);
    }
    else
    {
      LOG_ERROR("%s requires (at least) %zu args but %zu args were provided, "
                "reporting failure", commandID.c_str()
              , handler->getMinArgCount(), parts.size());
      return COMMAND_FAILED_RESPONSE;
    }
  }
  LOG_ERROR("No command handler for [%s]", commandID.c_str());
  return COMMAND_FAILED_RESPONSE;
}

void DCCPPProtocolHandler::registerCommand(DCCPPProtocolCommand *cmd)
{
  for (const auto& command : registeredCommands)
  {
    if(!command->getID().compare(cmd->getID()))
    {
      LOG_ERROR("Ignoring attempt to register second command with ID: %s",
        cmd->getID().c_str());
      return;
    }
  }
  LOG(VERBOSE, "Registering interface command %s", cmd->getID().c_str());
  registeredCommands.emplace_back(cmd);
}

DCCPPProtocolCommand *DCCPPProtocolHandler::getCommandHandler(const string &id)
{
  auto command = std::find_if(registeredCommands.begin()
                            , registeredCommands.end()
                            , [id](const std::unique_ptr<DCCPPProtocolCommand> &cmd)
                              {
                                return cmd->getID() == id;
                              });
  if (command != registeredCommands.end())
  {
    return command->get();
  }
  return nullptr;
}

DCCPPProtocolConsumer::DCCPPProtocolConsumer()
{
  _buffer.resize(256);
}

std::string DCCPPProtocolConsumer::feed(uint8_t *data, size_t len)
{
  for(size_t i = 0; i < len; i++)
  {
    _buffer.emplace_back(data[i]);
  }
  return processData();
}

string DCCPPProtocolConsumer::processData()
{
  auto s = _buffer.begin();
  auto consumed = _buffer.begin();
  string response;
  for(; s != _buffer.end();)
  {
    s = std::find(s, _buffer.end(), '<');
    auto e = std::find(s, _buffer.end(), '>');
    if(s != _buffer.end() && e != _buffer.end())
    {
      // discard the <
      s++;
      // discard the >
      *e = 0;
      std::string str(reinterpret_cast<char*>(&*s));
      response += DCCPPProtocolHandler::process(std::move(str));
      consumed = e;
    }
    s = e;
  }
  // drop everything we used from the buffer.
  _buffer.erase(_buffer.begin(), consumed);
  return response;
}

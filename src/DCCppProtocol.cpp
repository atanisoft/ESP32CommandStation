/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

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

#include "DCCppESP32.h"

#include "MotorBoard.h"
#include "SignalGenerator.h"
#include "Locomotive.h"
#include "Turnouts.h"
#include "Outputs.h"
#include "Sensors.h"

LinkedList<DCCPPProtocolCommand *> registeredCommands([](DCCPPProtocolCommand *command) {delete command; });

// <e> command handler, this command will clear all stored configuration data
// on the ESP32. All Turnouts, Outputs and Sensors will need to be reconfigured
// after sending this command.
class ConfigErase : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    stopDCCSignalGenerators();
    configStore.clear();
    TurnoutManager::clear();
    SensorManager::clear();
    OutputManager::clear();
    wifiInterface.printf(F("<O>"));
    startDCCSignalGenerators();
  }
  String getID() {
    return "e";
  }
};

// <E> command handler, this command stores all currently defined Turnouts,
// Sensors and Outputs into the ESP32 for use on subsequent startups.
class ConfigStore : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    stopDCCSignalGenerators();
    wifiInterface.printf(F("<e %d %d %d>"),
      TurnoutManager::store(),
      SensorManager::store(),
      OutputManager::store());
    startDCCSignalGenerators();
  }
  String getID() {
    return "E";
  }
};

// <R {CV} {CALLBACK} {CALLBACK-SUB}> command handler, this command attempts
// to read a CV value from the PROGRAMMING track. The returned value will be
// the actual CV value or -1 when there is a failure reading or verifying the CV.
class ReadCVCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    int cvNumber = arguments[0].toInt();
    wifiInterface.printf(F("<r%d|%d|%d %d>"),
      arguments[1].toInt(),
      arguments[2].toInt(),
      cvNumber,
      readCV(cvNumber));
  }

  String getID() {
    return "R";
  }
};

// <W {CV} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this command
// attempts to write a CV value on the PROGRAMMING track. The returned value
// is either the actual CV value written or -1 if there is a failure writing or
// verifying the CV value.
class WriteCVByteProgCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    int cvNumber = arguments[0].toInt();
    uint8_t cvValue = arguments[1].toInt();
    if(!writeProgCVByte(cvNumber, cvValue)) {
      cvValue = -1;
    }
    wifiInterface.printf(F("<r%d|%d|%d %d>"),
      arguments[2].toInt(),
      arguments[3].toInt(),
      cvNumber,
      cvValue);
  }

  String getID() {
    return "W";
  }
};

// <W {CV} {BIT} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this
// command attempts to write a single bit value for a CV on the PROGRAMMING
// track. The returned value is either the actual bit value of the CV or -1 if
// there is a failure writing or verifying the CV value.
class WriteCVBitProgCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    int cvNumber = arguments[0].toInt();
    uint8_t bit = arguments[1].toInt();
    int8_t bitValue = arguments[1].toInt();
    if(!writeProgCVBit(cvNumber, bit, bitValue == 1)) {
      bitValue = -1;
    }
    wifiInterface.printf(F("<r%d|%d|%d %d %d>"),
      arguments[2].toInt(),
      arguments[3].toInt(),
      cvNumber,
      bit,
      bitValue);
  }

  String getID() {
    return "B";
  }
};

// <w {LOCO} {CV} {VALUE}> command handler, this command sends a CV write packet
// on the MAIN OPERATIONS track for a given LOCO. No verification is attempted.
class WriteCVByteOpsCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    writeOpsCVByte(arguments[1].toInt(),
      arguments[1].toInt(),
      arguments[2].toInt());
  }

  String getID() {
    return "w";
  }
};

// <w {LOCO} {CV} {BIT} {VALUE}> command handler, this command sends a CV bit
// write packet on the MAIN OPERATIONS track for a given LOCO. No verification
// is attempted.
class WriteCVBitOpsCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    writeOpsCVBit(arguments[1].toInt(),
      arguments[1].toInt(),
      arguments[2].toInt(),
      arguments[3].toInt() == 1);
  }

  String getID() {
    return "b";
  }
};

// <s> command handler, this command sends the current status for all parts of
// the DCC++ESP32 BASE STATION. JMRI uses this command as a keep-alive heartbeat
// command.
class StatusCommand : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String> arguments) {
    wifiInterface.printf(F("<iDCC++ BASE STATION FOR ESP32: V-%s / %s %s>"), VERSION, __DATE__, __TIME__);
    MotorBoardManager::showStatus();
    LocomotiveManager::showStatus();
    TurnoutManager::showStatus();
    OutputManager::showStatus();
    wifiInterface.showInitInfo();
  }

  String getID() {
    return "s";
  }
};

void DCCPPProtocolHandler::init() {
  registerCommand(new ThrottleCommandAdapter());
  registerCommand(new FunctionCommandAdapter());
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
  registerCommand(new OutputCommandAdapter());
  registerCommand(new TurnoutCommandAdapter());
  registerCommand(new SensorCommandAdapter());
#if S88_ENABLED
  registerCommand(new S88SensorCommandAdapter());
#endif
}

void DCCPPProtocolHandler::process(const String commandString) {
  std::vector<String> parts;
  if(commandString.indexOf(' ') > 0) {
    int index = 0;
    while(index < commandString.length()) {
      int previousIndex = index;
      index = commandString.indexOf(' ', previousIndex);
      if(index < 0) {
        index = commandString.length();
      }
      parts.push_back(commandString.substring(previousIndex, index));
      // move past the space
      index++;
    }
  } else {
    parts.push_back(commandString);
  }
  String commandID = parts.front();
  parts.erase(parts.begin());
  //log_i("Command: %s, argument count: %d", commandID.c_str(), parts.size());
  bool processed = false;
  for (const auto& command : registeredCommands) {
    if(commandID == command->getID()) {
      command->process(parts);
      processed = true;
    }
  }
  if(!processed) {
    log_e("No command handler for [%s]", commandID.c_str());
    wifiInterface.printf(F("<X>"));
  }
}

void DCCPPProtocolHandler::registerCommand(DCCPPProtocolCommand *cmd) {
  for (const auto& command : registeredCommands) {
		if(command->getID() == cmd->getID()) {
      log_e("Ignoring attempt to register second command with ID: %s",
        cmd->getID().c_str());
      return;
    }
	}
  log_v("Registering interface command %s", cmd->getID().c_str());
  registeredCommands.add(cmd);
}

DCCPPProtocolCommand *DCCPPProtocolHandler::getCommandHandler(const String id) {
  for (const auto& command : registeredCommands) {
    if(command->getID() == id) {
      return command;
    }
  }
  return NULL;
}

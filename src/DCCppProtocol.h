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

#pragma once

#include <vector>
#include <WString.h>
#include <Stream.h>

// Class definition for a single protocol command
class DCCPPProtocolCommand {
public:
  virtual ~DCCPPProtocolCommand() {}
  virtual void process(const std::vector<String>) = 0;
  virtual String getID() = 0;
};

// Class definition for the Protocol Interpreter
class DCCPPProtocolHandler {
public:
  static void init();
  static void process(const String &);
  static void registerCommand(DCCPPProtocolCommand *);
  static DCCPPProtocolCommand *getCommandHandler(const String &);
};

class DCCPPProtocolConsumer {
public:
  DCCPPProtocolConsumer();
  void feed(uint8_t *, size_t);
  void update();
private:
  void processData();
  std::vector<uint8_t> _buffer;
};
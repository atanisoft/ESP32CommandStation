#pragma once

#include <WString.h>
#include <FS.h>
#include <ArduinoJson.h>

// Class definition for the Configuration Management system in DCC++ESP32
class ConfigurationManager {
public:
  ConfigurationManager();
  virtual ~ConfigurationManager();
  void init();
  void clear();

  JsonObject &load(const String &, DynamicJsonBuffer &);
  void store(const String &, const JsonObject &);
private:
  FS &_fs;
  bool _usingSpiffs;
};

extern ConfigurationManager configStore;

/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#ifndef _CDIHELPER_H_
#define _CDIHELPER_H_

#include <openlcb/SimpleStack.hxx>
#include <utils/FileUtils.hxx>

class CDIHelper
{
public:
/// Creates the XML representation of the configuration structure and saves
/// it to a file on the filesystem. Must be called after SPIFFS.begin() but
/// before calling the {\link create_config_file_if_needed} method. The
/// config file will be re-written whenever there was a change in the
/// contents. It is also necessary to declare the static compiled-in CDI to
/// be empty:
/// ```
///    namespace openlcb {
///    // This will stop openlcb from exporting the CDI memory space
///    // upon start.
///    extern const char CDI_DATA[] = "";
///    }  // namespace openlcb
/// ```
/// @param cfg is the global configuration instance (usually called cfg).
/// @param filename is where the xml file can be stored on the
/// filesystem. For example "/spiffs/cdi.xml".
template <class ConfigDef>
static void create_config_descriptor_xml(
    const ConfigDef &config, const char *filename
  , openlcb::SimpleStackBase *stack = nullptr)
{
  string cdi_string;
  ConfigDef cfg(config.offset());
  cfg.config_renderer().render_cdi(&cdi_string);

  cdi_string += '\0';

  bool need_write = false;
  LOG(INFO, "[CDI] Checking %s...", filename);
  FILE *ff = fopen(filename, "rb");
  if (!ff)
  {
    LOG(INFO, "[CDI] File %s does not exist", filename);
    need_write = true;
  }
  else
  {
    fclose(ff);
    string current_str = read_file_to_string(filename);
    if (current_str != cdi_string)
    {
      LOG(INFO, "[CDI] File %s is not up-to-date", filename);
      need_write = true;
    }
#if LOGLEVEL == VERBOSE
    else
    {
      LOG(INFO, "[CDI] File %s appears up-to-date (len %u vs %u)", filename
        , current_str.size(), cdi_string.size());
    }
#endif
  }
  if (need_write)
  {
    LOG(INFO, "[CDI] Updating %s (len %u)", filename,
        cdi_string.size());
    write_string_to_file(filename, cdi_string);
  }

  if (stack)
  {
    LOG(INFO, "[CDI] Registering CDI with stack...");
    // Creates list of event IDs for factory reset.
    auto *v = new vector<uint16_t>();
    cfg.handle_events([v](unsigned o) { v->push_back(o); });
    v->push_back(0);
    stack->set_event_offsets(v);
    // We leak v because it has to stay alive for the entire lifetime of
    // the stack.

    // Exports the file memory space.
    openlcb::MemorySpace *space = new openlcb::ROFileMemorySpace(filename);
    stack->memory_config_handler()->registry()->insert(
        stack->node(), openlcb::MemoryConfigDefs::SPACE_CDI, space);
  }
}
};

#endif // _CDIHELPER_H_
/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file CDIHelper.h
 *
 * Standalone CDI XML generation class.
 *
 * @author Balazs Racz
 * @date 24 July 2018
 */

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
static bool create_config_descriptor_xml(
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
    else
    {
      LOG(INFO, "[CDI] File %s appears up-to-date (len %u vs %u)", filename
        , current_str.size(), cdi_string.size());
    }
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
  return need_write;
}
};

#endif // _CDIHELPER_H_
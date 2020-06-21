/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2020 Mike Dunston

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

#ifndef CONFIG_MGR_H_
#define CONFIG_MGR_H_

#include <driver/sdmmc_types.h>
#include <utils/Singleton.hxx>

static constexpr char VFS_MOUNT[] = "/cfg";
static constexpr char CS_CONFIG_DIR[] = "/cfg/ESP32CS";
static constexpr char LCC_CFG_DIR[] = "/cfg/LCC";
static constexpr char LCC_CDI_XML[] = "/cfg/LCC/cdi.xml";
static constexpr char LCC_CONFIG_FILE[] = "/cfg/LCC/config";

/// This class manages the FileSystem used for persistent configuration data.
///
/// The persistent filesystem will be mounted under @ref VFS_MOUNT, this is
/// used by all modules (including OpenMRNLite).
class FileSystemManager : public Singleton<FileSystemManager>
{
public:
  FileSystemManager();
  void shutdown();
  bool is_sd()
  {
    return sd_ != nullptr;
  }

  bool exists(const std::string &);
  void remove(const std::string &);
  std::string load(const std::string &);
  void store(const char *, const std::string &);
  void force_factory_reset();
private:
  std::string getFilePath(const std::string &);
  int configFd_{-1};
  sdmmc_card_t *sd_{nullptr};
};

uint64_t string_to_uint64(std::string value);

#endif // CONFIG_MGR_H_

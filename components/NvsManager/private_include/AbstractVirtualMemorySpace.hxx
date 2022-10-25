/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2022 Mike Dunston

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

#ifndef ABSTRACT_VIRTUAL_MEMORY_SPACE_HXX_
#define ABSTRACT_VIRTUAL_MEMORY_SPACE_HXX_

namespace esp32cs
{

/// Abstract base class for virtual memory space implementations providing a
/// consistent API for tracking modifications that may require persistence.
class AbstractVirtualMemorySpace
{
public:
    /// @return true if the memory space has been modified, false otherwise.
    bool modified()
    {
        return modified_;
    }
protected:
    /// Helper method for implementations to flag that modifications have been
    /// made.
    /// @param value indicates modifications when true.
    void set_modified(bool value)
    {
        modified_ = value;
    }
private:
    /// Tracking variable for modifications.
    bool modified_{false};
};

} // namespace esp32cs

#endif // ABSTRACT_VIRTUAL_MEMORY_SPACE_HXX_
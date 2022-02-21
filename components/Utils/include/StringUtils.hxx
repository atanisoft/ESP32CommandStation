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

#ifndef STRINGUTILS_HXX_
#define STRINGUTILS_HXX_

#include <algorithm>
#include <string>
#include <utils/format_utils.hxx>

namespace esp32cs
{
    /// Utility function to inject a separator into a string at a specified
    /// interval.
    ///
    /// @param input is the string to be manipulated.
    /// @param num is the interval at which to insert the separator.
    /// @param separator is the character to insert.
    template <const unsigned num, const char separator>
    static inline void inject_seperator(std::string &input)
    {
        for (auto it = input.begin();
             (num + 1) <= std::distance(it, input.end());
             ++it)
        {
            std::advance(it, num);
            it = input.insert(it, separator);
        }
    }

    /// Converts a string to an unsigned 64bit integer removing "." characters.
    ///
    /// @param value String to convert.
    /// @return uint64_t version of @param value.
    static inline uint64_t string_to_uint64(std::string value)
    {
        // remove period characters if present
        value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
        // convert the string to a uint64_t value
        return std::stoull(value, nullptr, 16);
    }

    /// Converts an OpenLCB Node ID to a string format injecting a "." every
    /// two characters.
    ///
    /// @param id Node ID to convert.
    /// @return String representation of the Node ID.
    static inline std::string node_id_to_string(uint64_t id)
    {
        std::string result = uint64_to_string_hex(id, 12);
        std::replace(result.begin(), result.end(), ' ', '0');
        inject_seperator<2, '.'>(result);
        return result;
    }

    /// Converts an OpenLCB Event ID to a string format injecting a "." every
    /// two characters.
    ///
    /// @param id Event ID to convert.
    /// @return String representation of the Event ID.
    static inline std::string event_id_to_string(uint64_t id)
    {
        std::string result = uint64_to_string_hex(id, 16);
        std::replace(result.begin(), result.end(), ' ', '0');
        inject_seperator<2, '.'>(result);
        return result;
    }

    /// Modifies (in place) a string to remove null (\0), 0xFF and trailing
    /// spaces.
    ///
    /// @param value String to be modified.
    /// @param drop_eol When enabled `\r` and `\n` will be removed.
    static inline void remove_nulls_and_FF(
        std::string &value, bool drop_eol = false)
    {
        // replace null characters with spaces so the browser can parse the XML
        // successfully.
        std::replace(value.begin(), value.end(), '\0', ' ');

        // remove 0xff which is a default value for EEPROM data.
        std::replace(value.begin(), value.end(), (char)0xFF, ' ');

        if (drop_eol)
        {
            // replace newline characters with spaces.
            std::replace(value.begin(), value.end(), '\n', ' ');

            // replace carriage return characters with spaces.
            std::replace(value.begin(), value.end(), '\r', ' ');
        }
    }

    /// Utility function for replacement of one (or more) characters in the
    /// provided string.
    ///
    /// @param source string to be modified, this will be updated in-place.
    /// @param match one (or more) characters to be replaced in @param source.
    /// @param replacement one (or more) characters used for replacement.
    static inline void string_replace_all(std::string &source,
        const std::string& match, const std::string& replacement)
    {
        std::string update;
        std::string::size_type lastPos = 0;
        std::string::size_type findPos;

        // exit early if the string is not found
        if (source.find(match) == std::string::npos)
        {
            return;
        }

        // reserve space in the new string based on the length of the incoming
        // string.
        update.reserve(source.length());

        // scan the source string for occurrences of the matching string value
        while(std::string::npos != (findPos = source.find(match, lastPos)))
        {
            update.append(source, lastPos, findPos - lastPos);
            update.append(replacement);
            lastPos = findPos + match.length();
        }

        // append any remaining content from the source string
        update.append(source, lastPos, source.length() - lastPos);

        // relace the source string with the updated value.
        source.swap(update);
    }
}

#endif // STRINGUTILS_HXX_
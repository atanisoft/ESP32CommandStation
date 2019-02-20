/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file ConfigEntry.hxx
 *
 * Configuration option reader classes.
 *
 * @author Balazs Racz
 * @date 31 May 2015
 */

#ifndef _OPENLCB_CONFIGENTRY_HXX_
#define _OPENLCB_CONFIGENTRY_HXX_

#include <sys/types.h>
#include <stdint.h>
#include <endian.h>

#include <functional>

#include "openlcb/ConfigRenderer.hxx"

namespace openlcb
{

/// Class representing a particular location in the configuration space. All
/// typed configuration objects (atoms as well as groups) will be subclasses of
/// this.
class ConfigReference
{
public:
    /// Initializes the config reference from a configuration space offset.
    ///
    /// @param offset is the integer offset (0-based) in the address space for
    /// configuration.
    constexpr explicit ConfigReference(unsigned offset)
        : offset_(offset)
    {
    }

    /// Initializes the config reference from an existing config reference.
    ///
    /// @param ref is the existing (or saved) config reference.
    constexpr explicit ConfigReference(const ConfigReference &ref)
        : offset_(ref.offset())
    {
    }

    constexpr unsigned offset() const
    {
        return offset_;
    }

protected:
    /// zero-based offset from the beginning of the configuration file.
    unsigned offset_;
};

/// Function declaration that will be called with all event offsets that exist
/// in the configuration space.
typedef std::function<void(unsigned)> EventOffsetCallback;

///
/// Base class for individual configuration entries. Defines helper methods for
/// reading and writing.
///
class ConfigEntryBase : public ConfigReference
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(ConfigEntryBase, ConfigReference)

    template<typename... Args>
    static constexpr GroupConfigOptions group_opts(Args... args)
    {
        return GroupConfigOptions();
    }

    static void handle_events(const EventOffsetCallback& fn) {}

protected:
    /// Reads a given typed variable from the configuration file. DOes not do
    /// any binary conversion (only reads raw data).
    ///
    /// @param fd file to read data from.
    ///
    /// @return the raw value read from the configuration file.
    ///
    template <class T> T raw_read(int fd) const
    {
        T ret;
        repeated_read(fd, &ret, sizeof(T));
        return ret;
    }

    /// Writes a given typed variable to the configuration file. Does not do
    /// any binary conversion.
    ///
    /// @param fd file to write data to.
    ///
    /// @param value the raw value to write to the configuration file.
    ///
    template <class T> void raw_write(int fd, const T &value) const
    {
        repeated_write(fd, &value, sizeof(T));
    }

    /// Performs a reliable read from the given FD. Crashes if the read fails.
    ///
    /// @param fd the file to read data from
    /// @param buf the location to write data to
    /// @param size how many bytes to read
    ///
    void repeated_read(int fd, void *buf, size_t size) const;

    /// Performs a reliable write to the given FD. Crashes if the write fails.
    ///
    /// @param fd the file to write data to
    /// @param buf the location of the data to write
    /// @param size how many bytes to write
    ///
    void repeated_write(int fd, const void *buf, size_t size) const;
};

/// Implementation class for numeric configuration entries, templated by the
/// integer type.
template <class TR> class NumericConfigEntry : public ConfigEntryBase
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(NumericConfigEntry, ConfigEntryBase)

    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint8_t endian_convert(uint8_t d)
    {
        return d;
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint16_t endian_convert(uint16_t d)
    {
        return be16toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint32_t endian_convert(uint32_t d)
    {
        return be32toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint64_t endian_convert(uint64_t d)
    {
        return be64toh(d);
    }

    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static int8_t endian_convert(int8_t d)
    {
        return d;
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static int16_t endian_convert(int16_t d)
    {
        return be16toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static int32_t endian_convert(int32_t d)
    {
        return be32toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static int64_t endian_convert(int64_t d)
    {
        return be64toh(d);
    }

    /// Storage bytes occupied by the instance in the config file.
    ///
    /// @return number of bytes that the config parser offset will be
    /// incremented by this entry.
    ///
    static constexpr unsigned size()
    {
        return sizeof(TR);
    }

    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }

    static constexpr NumericConfigRenderer config_renderer()
    {
        return NumericConfigRenderer("int", size());
    }

    /// Reads the data from the configuration file.
    ///
    /// @param fd file descriptor of the config file.
    ///
    /// @return value of the configuration atom that *this represents.
    ///
    TR read(int fd) const
    {
        return endian_convert(raw_read<TR>(fd));
    }

    /// Reads data from configuration file obeying a specific trimming. If the
    /// value violates the trimming constraints, the configuration file will be
    /// overwritten with a trimmed value.
    ///
    /// @param fd the descriptor of the config file.
    /// @param min_value minimum acceptable value
    /// @param max_value maximum acceptable value
    /// @return current value after trimming.
    TR read_or_write_trimmed(int fd, TR min_value, TR max_value) {
        TR value = read(fd);
        if (value < min_value) {
            value = min_value;
            write(fd, value);
        }
        if (value > max_value) {
            value = max_value;
            write(fd, value);
        }
        return value;
    }
    
    /// Writes the data to the configuration file.
    ///
    /// @param fd file descriptor of the config file.
    ///
    /// @param value of the configuration atom that should be written.
    ///
    void write(int fd, TR d) const
    {
        raw_write(fd, endian_convert(d));
    }
};

/// Unsigned numeric config entry with 1 byte width.
using Uint8ConfigEntry = NumericConfigEntry<uint8_t>;
/// Unsigned numeric config entry with 2 bytes width.
using Uint16ConfigEntry = NumericConfigEntry<uint16_t>;
/// Unsigned numeric config entry with 4 bytes width.
using Uint32ConfigEntry = NumericConfigEntry<uint32_t>;
/// Unsigned numeric config entry with 8 bytes width.
using Uint64ConfigEntry = NumericConfigEntry<uint64_t>;

/// Signed numeric config entry with 1 byte width.
using Int8ConfigEntry = NumericConfigEntry<int8_t>;
/// Signed numeric config entry with 2 bytes width.
using Int16ConfigEntry = NumericConfigEntry<int16_t>;
/// Signed numeric config entry with 4 bytes width.
using Int32ConfigEntry = NumericConfigEntry<int32_t>;
/// Signed numeric config entry with 8 bytes width.
using Int64ConfigEntry = NumericConfigEntry<int64_t>;

/// Implementation class for event ID configuration entries.
class EventConfigEntry : public Uint64ConfigEntry
{
public:
    template <typename T>
    constexpr EventConfigEntry(T t)
        : Uint64ConfigEntry(t)
    {
    }

    static constexpr AtomConfigRenderer config_renderer()
    {
        return AtomConfigRenderer("eventid", AtomConfigRenderer::SKIP_SIZE);
    }

    void handle_events(const EventOffsetCallback& fn) {
        fn(offset());
    }
};

/// Implementation class for string configuration entries. The template
/// argument is the string size (total space in bytes used in the configuration
/// space including the NULL character).
template <unsigned SIZE> class StringConfigEntry : public ConfigEntryBase
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(StringConfigEntry, ConfigEntryBase)

    /// Storage bytes occupied by the instance in the config file.
    ///
    /// @return number of bytes that the config parser offset will be
    /// incremented by this entry.
    ///
    static constexpr unsigned size()
    {
        return SIZE;
    }

    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }

    static constexpr AtomConfigRenderer config_renderer()
    {
        return AtomConfigRenderer("string", size());
    }

    string read(int fd) const
    {
        string s(size(), '\0');
        repeated_read(fd, &s[0], size());
        size_t real_len = strlen(s.c_str());
        s.resize(real_len);
        return s;
    }

    void write(int fd, string data) const
    {
        if (data.size() > size() - 1) {
            data.resize(size() - 1);
        }
        repeated_write(fd, data.c_str(), data.size() + 1);
    }
};

/// Implementation class for internal data bytes configuration entries. These
/// are stored like a pascal string: the first byte is the length, the rest is
/// the payload. The template argument is the storage size. These data elements
/// are not rendered in the CDI in a user-visible form.
template <unsigned SIZE> class BytesConfigEntry : public ConfigEntryBase
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(BytesConfigEntry, ConfigEntryBase)

    static_assert(SIZE <= 256, "BytesConfigEntry cannot store longer than 255");
    
    /// Storage bytes occupied by the instance in the config file.
    ///
    /// @return number of bytes that the config parser offset will be
    /// incremented by this entry.
    ///
    static constexpr unsigned size()
    {
        return SIZE;
    }

    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }

    static constexpr EmptyGroupConfigRenderer config_renderer()
    {
        return EmptyGroupConfigRenderer(size());
    }

    string read(int fd) const
    {
        string s(size(), '\0');
        repeated_read(fd, &s[0], size());
        size_t real_len = (unsigned char)s[0];
        return s.substr(1, real_len);
    }

    void write(int fd, string data) const
    {
        HASSERT(data.size() < size());
        data.insert(0, 1, data.size());
        repeated_write(fd, data.data(), data.size());
    }
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGENTRY_HXX_

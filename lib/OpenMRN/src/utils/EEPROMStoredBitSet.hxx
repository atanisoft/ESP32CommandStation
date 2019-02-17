/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file EEPROMStoredBitSet.hxx
 *
 * Implementation for persistent bit set that uses EEPROM as backing storage.
 *
 * @author Balazs Racz
 * @date 5 June 2017
 */

#ifndef _UTILS_EEPROMSTOREDBITSET_HXX_
#define _UTILS_EEPROMSTOREDBITSET_HXX_

#include "utils/StoredBitSet.hxx"
#include "utils/logging.h"

/// Template of how the HW class should look like.
class EEPROMStoredBitSet_DefaultHW
{
protected:
    typedef uint32_t eeprom_t;

    /// @return how many user bits we store per physical cell.
    static unsigned bits_per_cell()
    {
        return 27;
    }

    /// Defines how many virtual cells we store. The total size of the
    /// storage will be VIRTUAL_CELL_COUNT * BITS_PER_CELL. The maximum is
    /// 2 ^ (sizeof(eeprom_t) * 8 - bits_per_cell() - 1) - 1.
    /// @return the number of cells in the address space
    static unsigned virtual_cell_count()
    {
        return 8;
    }

    /// Defines
    /// @return how many cells are there in the physical storage.
    /// 0..physical_cell_count() - 1 will be the indexes in
    static unsigned physical_cell_count()
    {
        return 64;
    }

    /// Writes to the physical storage.
    /// @param cell_offset the eeprom cell to write,
    /// 0..physical_cell_count() - 1.
    /// @param value is the data to write to that cell.
    static void write_cell(unsigned cell_offset, eeprom_t value);

    /// Reads from the physical storage.
    /// @param cell_offset the eeprom cell to read.
    /// @return the last written value of that cell.
    static eeprom_t read_cell(unsigned cell_offset);
};

template <class HW>
class EEPROMStoredBitSet : private HW, public ShadowedStoredBitSet
{
public:
    template <typename... Args>
    EEPROMStoredBitSet(Args... args)
        : HW(args...)
        , ShadowedStoredBitSet(HW::bits_per_cell() * HW::virtual_cell_count(),
              HW::bits_per_cell())
    {
        mount();
    }

    void flush() override
    {
        do
        {
            auto c = ShadowedStoredBitSet::next_dirty();
            if (c == NO_CELL)
                break;
            if (writeOffset_ >= HW::physical_cell_count())
            {
                // Wrap around.
                currentMarker_ = (~currentMarker_) & MARKER_MASK;
                write_header();
                clear_all_dirty();
                return;
            }
            if (writeOffset_ + 1 < HW::physical_cell_count())
            {
                auto v = HW::read_cell(writeOffset_ + 1);
                if ((v & MARKER_MASK) == currentMarker_)
                {
                    // This is a problem. If we write the second magic now, the
                    // additional entry will become a valid journal entry.
                    HW::write_cell(
                        writeOffset_ + 1, (~currentMarker_) & MARKER_MASK);
                }
            }
            HW::write_cell(writeOffset_++, get_vcell(c));
        } while (true);
    }

private:
    using eeprom_t = typename HW::eeprom_t;

    void mount()
    {
        unsigned vcell_ofs_bits =
            (sizeof(eeprom_t) * 8) - HW::bits_per_cell() - 1;
        HASSERT(vcell_ofs_bits < 32);
        HASSERT((1U << vcell_ofs_bits) - 2 >= HW::virtual_cell_count());
        HASSERT(
            HW::virtual_cell_count() + 2 <= (HW::physical_cell_count() / 2));
        eeprom_t magic = HW::read_cell(FIRST_MAGIC_OFS);
        if ((magic & ~MARKER_MASK) != MAGIC)
        {
            format();
            return;
        }
        for (unsigned i = 0; i < HW::virtual_cell_count(); ++i)
        {
            eeprom_t v = HW::read_cell(i + HEADER_OFS);
            read_entry(v);
        }
        eeprom_t magic2 = HW::read_cell(SECOND_MAGIC_OFS);
        if (magic2 == magic)
        {
            // Markers are consistent. Read further entries.
            currentMarker_ = magic2 & MARKER_MASK;
            writeOffset_ = JOURNAL_OFS;
            eeprom_t v;
            while (writeOffset_ < HW::physical_cell_count() &&
                ((v = HW::read_cell(writeOffset_)) & MARKER_MASK) ==
                    currentMarker_)
            {
                read_entry(v);
                ++writeOffset_;
            }
        }
        else
        {
            // There was a problem rewriting the header. Read the entire sector.
            for (unsigned i = JOURNAL_OFS; i < HW::physical_cell_count(); ++i)
            {
                eeprom_t v = HW::read_cell(i);
                if (v == ERASED)
                {
                    writeOffset_ = i;
                    break;
                }
                read_entry(v);
            }
            currentMarker_ = magic & MARKER_MASK;
            write_header();
        }
        clear_all_dirty();
    }

    void clear_all_dirty()
    {
        cell_offs_t c;
        while ((c = next_dirty()) != NO_CELL)
        {
            clear_dirty(c);
        }
    }

    void format()
    {
        for (unsigned j = JOURNAL_OFS; j < HW::physical_cell_count(); ++j)
        {
            eeprom_t v = HW::read_cell(j);
            if (v != 0)
            {
                HW::write_cell(j, 0);
            }
        }
        currentMarker_ = MARKER_MASK;
        write_header();
    }

    void write_header()
    {
        HW::write_cell(FIRST_MAGIC_OFS, MAGIC | currentMarker_);
        for (unsigned i = 0; i < HW::virtual_cell_count(); ++i)
        {
            eeprom_t ov = HW::read_cell(i + HEADER_OFS);
            eeprom_t nv = get_vcell(i);
            if ((ov & ~MARKER_MASK) == (nv & ~MARKER_MASK))
            {
                // Didn't change, skip write.
                continue;
            }
            HW::write_cell(i + HEADER_OFS, nv);
        }
        eeprom_t v = HW::read_cell(JOURNAL_OFS);
        if ((v & MARKER_MASK) == currentMarker_)
        {
            // This is a problem. If we write the second magic now, the
            // additional entry will become a valid journal entry.
            HW::write_cell(JOURNAL_OFS, (~currentMarker_) & MARKER_MASK);
        }
        HW::write_cell(SECOND_MAGIC_OFS, MAGIC | currentMarker_);
        writeOffset_ = JOURNAL_OFS;
    }

    void read_entry(eeprom_t v)
    {
        auto vcell = value_to_vcell_ofs(v);
        ShadowedStoredBitSet::set_multi(vcell * HW::bits_per_cell(),
            HW::bits_per_cell(), value_to_vcell_value(v));
    }

    unsigned value_to_vcell_ofs(eeprom_t value)
    {
        return (value & ~MARKER_MASK) >> HW::bits_per_cell();
    }

    unsigned value_to_vcell_value(eeprom_t value)
    {
        return (value & ((1U << HW::bits_per_cell()) - 1));
    }

    eeprom_t get_vcell(unsigned vcell)
    {
        eeprom_t v =
            ShadowedStoredBitSet::get_multi(vcell * HW::bits_per_cell(), HW::bits_per_cell());
        v |= (vcell << HW::bits_per_cell());
        v |= currentMarker_;
        return v;
    }

    /// The MSB of each eeprom value is the marker. This alternates between
    /// zero and one every time we wrap around.
    static constexpr eeprom_t MARKER_MASK = eeprom_t(1)
        << ((sizeof(eeprom_t) * 8) - 1);

    static constexpr eeprom_t EEPROM_MASK = eeprom_t(-1);

    /// Magic value in the zero offset certifies that we are correctly
    /// formatted.
    static constexpr eeprom_t MAGIC =
        0x02b26758U & EEPROM_MASK & (~MARKER_MASK);

    static constexpr eeprom_t ERASED = 0xFFFFFFFFU & EEPROM_MASK;

    static constexpr unsigned FIRST_MAGIC_OFS = 0;
    static constexpr unsigned SECOND_MAGIC_OFS = 1;
    static constexpr unsigned HEADER_OFS = 2;
    static constexpr unsigned JOURNAL_OFS =
        HW::virtual_cell_count() + HEADER_OFS;

    /// What is the current desired value of the marker bit.
    eeprom_t currentMarker_;
    unsigned writeOffset_;
};

#endif // _UTILS_EEPROMSTOREDBITSET_HXX_

#include "utils/test_main.hxx"

// We have to avoid pulling in freertos stuff. We redefine the base class to
// avoid dependency on hand-written fileio stuff.
#define _FREERTOS_DRIVERS_COMMON_EEPROM_HXX_

class EEPROM
{
public:
    /// Constructor.
    ///
    /// @param name The name of the device node in the filesystem, e.g.
    /// /dev/eeprom
    /// @param file_size how many bytes are in the eeprom (real or emulated).
    ///
    EEPROM(const char *name, size_t file_size)
        : fileSize(file_size)
    {
    }

    /// Override this function to write data to the eeprom. Has to function
    /// synchronously.
    ///
    /// @param index offset where to write data to inside the file.
    /// [0..file_size).
    /// @param buf data to write
    /// @param len how many bytes to write
    ///
    virtual void write(unsigned int index, const void *buf, size_t len) = 0;

    /// Override this function to read data from the eeprom. Has to function
    /// synchronously.
    ///
    /// @param index offset where to read data from inside the file.
    /// [0..file_size).
    /// @param buf where to read data to
    /// @param len how many bytes to read
    ///
    virtual void read(unsigned int index, void *buf, size_t len) = 0;

    /// @return the eeprom size.
    size_t file_size()
    {
        return fileSize;
    }

private:
    size_t fileSize; ///< size of the eeprom.
};

// Terrible hack to test internals of the eeprom emulation.
#define private public
#define protected public

#include "freertos_drivers/common/EEPROMEmulation.hxx"
#include "freertos_drivers/common/EEPROMEmulation.cxx"

static const char FILENAME[] = "/tmp/eeprom";

#define EELEN 32768

// We need to jump through some hoops to define a linker symbol
// "__eeprom_start" in a place that is not actually constant.
namespace foo {
extern "C" {
uint8_t __eeprom_start[EELEN];
uint8_t __eeprom_end;
}
}

#define EEBLOCKSIZE 4

const size_t EEPROMEmulation::SECTOR_SIZE = (4 * 1024);
const size_t EEPROMEmulation::BLOCK_SIZE = (EEBLOCKSIZE);
const size_t EEPROMEmulation::BYTES_PER_BLOCK = (EEBLOCKSIZE / 2);
static constexpr unsigned blocks_per_sector = EEPROMEmulation::SECTOR_SIZE / EEPROMEmulation::BLOCK_SIZE;

/// Test EEPROM emulation HAL implementation that writes to a block of
/// (RAM) memory. Used for unittesting the EEPROM Emulation code.
class MyEEPROM : public EEPROMEmulation
{
public:
    /// Contructor. @param file_size how many bytes @param clear if true, the
    /// EEPROM will be initialized with all 0xFF bytes.
    MyEEPROM(size_t file_size, bool clear = true)
        : EEPROMEmulation(FILENAME, file_size)
    {
        HASSERT(EELEN == &__eeprom_end - &__eeprom_start);
        if (clear) {
            memset(foo::__eeprom_start, 0xFF, EELEN);
        }
        mount();

        LOG(INFO, "sector count %d, active index %d, slot count %d, available count %d", sector_count(), activeSector_, slot_count(), avail());
    }

    /// @return how many blocks are available in the current sector.
    unsigned avail() {
        return availableSlots_;
    }

private:
    void flash_erase(unsigned sector) override {
        ASSERT_LE(0u, sector);
        ASSERT_GT(EELEN / SECTOR_SIZE, sector);
        void* address = &foo::__eeprom_start[sector * SECTOR_SIZE];
        memset(address, 0xff, SECTOR_SIZE);
    }

    void flash_program(unsigned sector, unsigned block, uint32_t *data, uint32_t byte_count) override {
        ASSERT_LE(0u, sector);
        ASSERT_GT(EELEN / SECTOR_SIZE, sector);
        ASSERT_LE(0u, block);
        ASSERT_GT(SECTOR_SIZE/BLOCK_SIZE, block);
        ASSERT_EQ(0u, byte_count % BLOCK_SIZE);
        uint8_t* address = &foo::__eeprom_start[sector * SECTOR_SIZE + block * BLOCK_SIZE];
        memcpy(address, data, byte_count);
    }

    const uint32_t* block(unsigned sector, unsigned index) override {
        EXPECT_GT(EELEN / SECTOR_SIZE, sector);
        EXPECT_GT(SECTOR_SIZE / BLOCK_SIZE, index);
        void* address = &foo::__eeprom_start[sector * SECTOR_SIZE + index * BLOCK_SIZE];
        return (uint32_t*) address;
    }
};

TEST(EepromStaticTest, assertions) {
    volatile size_t p1 = (volatile size_t)&__eeprom_start;
    volatile size_t p2 = (volatile size_t)&foo::__eeprom_start[0];

    volatile size_t e1 = (volatile size_t)&__eeprom_end;

    ASSERT_EQ(p1, p2);
    ASSERT_EQ(p1 + EELEN, e1);
    ASSERT_EQ(0u, p1 % 4); // alignment
}

/// Test fixture class for testing the EEPROM emulation.
class EepromTest : public ::testing::Test {
protected:
    /// Creates the eeprom under test. @param clear if true, eeprom starts up
    /// empty.
    void create(bool clear = true) {
        e.reset(new MyEEPROM(eeprom_size, clear));
    }

    /// Helper function to write to the test eeprom.
    ///
    /// @param ofs where to write
    /// @param payload what to write
    ///
    void write_to(unsigned ofs, const string &payload)
    {
        ee()->write(ofs, payload.data(), payload.size());
    }

    /// @return the eeprom implementation under test.
    EEPROM* ee() {
        return static_cast<EEPROM*>(e.operator->());
    }

    /** @return the data payload in a given block. @param block_number which
     * block's data to return */
    string block_data(unsigned block_number) {
        uint32_t* address = (uint32_t*)&foo::__eeprom_start[block_number * EEBLOCKSIZE];
        uint8_t data[EEBLOCKSIZE / 2];
        for (int i = 0; i < EEBLOCKSIZE / 4; ++i) {
            data[(i * 2) + 0] = (address[i] >> 0) & 0xFF;
            data[(i * 2) + 1] = (address[i] >> 8) & 0xFF;
        }
        return string((char*)data, EEBLOCKSIZE / 2);
    }

    /// Write enough much data to the eepromemu under test to overflow the
    /// current block.
    void overflow_block() {
        unsigned avail = e->avail();
        for (int i = 0; i < 27000; ++i) {
            char d[1] = {static_cast<char>(i & 0xff)};
            write_to(27, string(d, 1));
            if (e->avail() > avail) return;
            avail = e->avail();
        }
    }

    /** @return the address (that is, the file offset) of the payload stored in
     * a given block. @param block_number is the index of the block in question
     * (0.. block count - 1).*/
    uint32_t block_address(unsigned block_number) {
        uint32_t* address = (uint32_t*)&foo::__eeprom_start[block_number * EEBLOCKSIZE];
        return ((*address) >> 16) * (EEBLOCKSIZE / 2);
    }

#define EXPECT_AT(ofs, PAYLOAD) { string p(PAYLOAD); string ret(p.size(), 0); ee()->read(ofs, &ret[0], p.size()); EXPECT_EQ(p, ret); }

#define EXPECT_SLOT(block_number, address, payload) { EXPECT_EQ((unsigned)address, block_address(block_number)); EXPECT_EQ(string(payload), block_data(block_number)); }

    static constexpr unsigned eeprom_size = 1000; ///< test eeprom size
    std::unique_ptr<MyEEPROM> e; ///< EEPROM under test.
};


TEST_F(EepromTest, create) {
    create();
    EXPECT_EQ(0, e->activeSector_);
    EXPECT_EQ(8u, e->sector_count());
    EXPECT_EQ((uint32_t*)&__eeprom_start, e->block(0, 0));
    EXPECT_EQ((uint32_t*)&foo::__eeprom_start[4*1024], e->block(1, 0));
}

TEST_F(EepromTest, readwrite) {
    create();

    write_to(13, "abcd");
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    EXPECT_SLOT(6, 2*0xFFFF, "\xFF\xFF");

    EXPECT_AT(13, "abcd");
    EXPECT_AT(14, "bc");
    EXPECT_AT(15, "cd");

    write_to(12, "up");
    EXPECT_AT(12, "upb");
    EXPECT_AT(12, "upbcd");
    write_to(12, "kq");
    EXPECT_AT(12, "kqbcd");
    EXPECT_AT(12, "kqbcd\xFF");

    // Reboot MCU
    create(false);
    EXPECT_AT(12, "kqbcd\xFF");
}

TEST_F(EepromTest, readwrite_recreate) {
    create();
    // Reboot MCU after creating empty.
    create(false);

    write_to(13, "abcd");
    EXPECT_AT(13, "abcd");

    // Reboot MCU
    create(false);
    EXPECT_AT(13, "abcd");
}

TEST_F(EepromTest, smalloverflow) {
    create();
    write_to(13, "abcd");
    EXPECT_AT(13, "abcd");
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    overflow_block();
    EXPECT_EQ(1, e->activeSector_);
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    EXPECT_SLOT(blocks_per_sector + 3, 12, "\xFF""a");
    EXPECT_SLOT(blocks_per_sector + 4, 14, "bc");
    EXPECT_SLOT(blocks_per_sector + 5, 16, "d\xFF");

    EXPECT_AT(13, "abcd");
    overflow_block();
    EXPECT_AT(13, "abcd");
    overflow_block();
    EXPECT_AT(13, "abcd");
    EXPECT_EQ(3, e->activeSector_);
    create(false);
    EXPECT_AT(13, "abcd");
    EXPECT_EQ(3, e->activeSector_);
}

TEST_F(EepromTest, many_overflow) {
    create();
    write_to(13, "abcd");
    EXPECT_AT(13, "abcd");
    // A lot of writes will surely cause the data to be overflowed to a new
    // sector
    for (int i = 0; i < 20; ++i) {
        overflow_block();
    }
    EXPECT_AT(13, "abcd");
    unsigned s = e->activeSector_;
    create(false);
    EXPECT_AT(13, "abcd");
    EXPECT_EQ(s, e->activeSector_);
}

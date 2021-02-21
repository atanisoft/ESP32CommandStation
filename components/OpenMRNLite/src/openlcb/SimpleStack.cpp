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
 * \file SimpleStack.cxx
 *
 * A complete OpenLCB stack for use in straightforward OpenLCB nodes.
 *
 * @author Balazs Racz
 * @date 18 Mar 2015
 */

/// Overrides loglevel.
#ifndef __FreeRTOS__
#define LOGLEVEL INFO
#endif

#if defined(__linux__) || defined(__MACH__)
#include <net/if.h>
#include <termios.h> /* tc* functions */
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "openlcb/SimpleStack.hxx"

#include "openlcb/EventHandler.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "openlcb/SimpleNodeInfo.hxx"
#include "openmrn_features.h"
#include "utils/HubDeviceSelect.hxx"
#include "utils/SocketCan.hxx"

namespace openlcb
{

SimpleStackBase::SimpleStackBase(
    std::function<std::unique_ptr<SimpleStackBase::PhysicalIf>()>
        create_if_helper)
    : ifaceHolder_(create_if_helper())
{
}

SimpleCanStackBase::SimpleCanStackBase(const openlcb::NodeID node_id)
    : SimpleStackBase(std::bind(&SimpleCanStackBase::create_if, this, node_id))
{
}

std::unique_ptr<SimpleStackBase::PhysicalIf> SimpleCanStackBase::create_if(
    const openlcb::NodeID node_id)
{
    return std::unique_ptr<PhysicalIf>(new CanPhysicalIf(node_id, service()));
}

SimpleTcpStackBase::SimpleTcpStackBase(const openlcb::NodeID node_id)
    : SimpleStackBase(std::bind(&SimpleTcpStackBase::create_if, this, node_id))
{
}

std::unique_ptr<SimpleStackBase::PhysicalIf> SimpleTcpStackBase::create_if(
    const openlcb::NodeID node_id)
{
    return std::unique_ptr<PhysicalIf>(new TcpPhysicalIf(node_id, service()));
}

SimpleCanStack::SimpleCanStack(const openlcb::NodeID node_id)
    : SimpleCanStackBase(node_id)
    , node_(iface(), node_id)
{
}

SimpleTcpStack::SimpleTcpStack(const openlcb::NodeID node_id)
    : SimpleTcpStackBase(node_id)
    , node_(iface(), node_id)
{
}

void SimpleStackBase::start_stack(bool delay_start)
{
#if OPENMRN_HAVE_POSIX_FD
    // Opens the eeprom file and sends configuration update commands to all
    // listeners.
    configUpdateFlow_.open_file(CONFIG_FILENAME);
    configUpdateFlow_.init_flow();
#endif // have posix fd

    if (!delay_start)
    {
        start_iface(false);
    }

    // Adds memory spaces.
    if (config_enable_all_memory_space() == CONSTANT_TRUE)
    {
        auto *space = new ReadOnlyMemoryBlock(nullptr, 0xFFFFFFFFUL);
        memoryConfigHandler_.registry()->insert(
            nullptr, MemoryConfigDefs::SPACE_ALL_MEMORY, space);
        additionalComponents_.emplace_back(space);
    }

    // Calls node-specific startup hook.
    start_node();
}

void SimpleStackBase::default_start_node()
{
    {
        auto *space = new ReadOnlyMemoryBlock(
            reinterpret_cast<const uint8_t *>(&SNIP_STATIC_DATA),
            sizeof(SNIP_STATIC_DATA));
        memoryConfigHandler_.registry()->insert(
            node(), MemoryConfigDefs::SPACE_ACDI_SYS, space);
        additionalComponents_.emplace_back(space);
    }
#if OPENMRN_HAVE_POSIX_FD 
    {
        auto *space = new FileMemorySpace(
            SNIP_DYNAMIC_FILENAME, sizeof(SimpleNodeDynamicValues));
        memoryConfigHandler_.registry()->insert(
            node(), MemoryConfigDefs::SPACE_ACDI_USR, space);
        additionalComponents_.emplace_back(space);
    }
#endif // NOT ARDUINO, YES ESP32
    size_t cdi_size = strlen(CDI_DATA);
    if (cdi_size > 0)
    {
        auto *space = new ReadOnlyMemoryBlock(
            reinterpret_cast<const uint8_t *>(&CDI_DATA), cdi_size + 1);
        memoryConfigHandler_.registry()->insert(
            node(), MemoryConfigDefs::SPACE_CDI, space);
        additionalComponents_.emplace_back(space);
    }
#if OPENMRN_HAVE_POSIX_FD
    if (CONFIG_FILENAME != nullptr)
    {
        auto *space = new FileMemorySpace(
            configUpdateFlow_.open_file(CONFIG_FILENAME), CONFIG_FILE_SIZE);
        memory_config_handler()->registry()->insert(
            node(), openlcb::MemoryConfigDefs::SPACE_CONFIG, space);
        additionalComponents_.emplace_back(space);
    }
#endif // NOT ARDUINO, YES ESP32
}

SimpleTrainCanStack::SimpleTrainCanStack(
    openlcb::TrainImpl *train, const char *fdi_xml, NodeID node_id)
    // Note: this code tries to predict what the node id of the trainNode_ will
    // be. Unfortunately due to initialization order problems we cannot query
    // it in advance.
    : SimpleCanStackBase(node_id)
    , trainNode_(&tractionService_, train, node_id)
    , fdiBlock_(reinterpret_cast<const uint8_t *>(fdi_xml), strlen(fdi_xml))
{
}

void SimpleTrainCanStack::start_node()
{
    default_start_node();
    memoryConfigHandler_.registry()->insert(
        &trainNode_, MemoryConfigDefs::SPACE_FDI, &fdiBlock_);
}

void SimpleStackBase::start_after_delay()
{
    start_iface(false);
}

void SimpleTcpStackBase::start_iface(bool restart)
{
}

void SimpleCanStackBase::start_iface(bool restart)
{
    if (restart)
    {
        if_can()->alias_allocator()->reinit_seed();
        if_can()->local_aliases()->clear();
        if_can()->remote_aliases()->clear();
    }

    // Bootstraps the fresh alias allocation process.
    if_can()->alias_allocator()->send(if_can()->alias_allocator()->alloc());
}

void SimpleStackBase::restart_stack()
{
    node()->clear_initialized();
    start_iface(true);

    // Causes all nodes to grab a new alias and send out node initialization
    // done messages. This object owns itself and will do `delete this;` at the
    // end of the process.
    new ReinitAllNodes(iface());
}

int SimpleStackBase::create_config_file_if_needed(const InternalConfigData &cfg,
    uint16_t expected_version, unsigned file_size)
{
    HASSERT(CONFIG_FILENAME);
    struct stat statbuf;
    bool reset = false;
    bool extend = false;
    int fd = ::open(CONFIG_FILENAME, O_RDONLY);
    if (fd < 0)
    {
        // Create file.
        LOG(INFO, "Creating config file %s", CONFIG_FILENAME);
        reset = true;
        fd = ::open(
            CONFIG_FILENAME, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR);
        if (fd < 0)
        {
            printf("Failed to create config file: fd %d errno %d: %s\n", fd,
                errno, strerror(errno));
            DIE();
        }
        reset = true;
    }
    ::close(fd);
    fd = configUpdateFlow_.open_file(CONFIG_FILENAME);
    HASSERT(fstat(fd, &statbuf) == 0);
    if (statbuf.st_size < (ssize_t)file_size)
    {
        extend = true;
    }
    // Handle the case where the file exists but is too short for the verison
    // check. This was observed on the esp32 with SD storage which does not
    // automatically flush to disk on write.
    if ((long)statbuf.st_size < (long)cfg.version().end_offset())
    {
        LOG(VERBOSE, "%s is too short (%d vs %d), forcing reset.",
            CONFIG_FILENAME, (int)statbuf.st_size, cfg.version().end_offset());
        reset = true;
    }
    if (!reset && cfg.version().read(fd) != expected_version)
    {
        uint16_t current_version = cfg.version().read(fd);
        if (current_version != expected_version)
        {
            LOG(VERBOSE, "config version mismatch (%d vs %d), forcing reset.",
                current_version, expected_version);
            reset = true;
        }
    }
    if (!reset && !extend)
    {
        return fd;
    }

    // Clears the file, preserving the node name and desription if any.
    if (extend && !reset)
    {
        auto ret = lseek(fd, statbuf.st_size, SEEK_SET);
        HASSERT(ret == statbuf.st_size);
        file_size -= statbuf.st_size; // Clears nothing, just extends with 0xFF.
    }
    else if (statbuf.st_size >= 128)
    {
        auto ret = lseek(fd, 128, SEEK_SET);
        HASSERT(ret == 128);
        file_size -= 128; // Clears less.
    }
    else
    {
        lseek(fd, 0, SEEK_SET);
    }

    static const unsigned bufsize = 128;
    char *buf = (char *)malloc(bufsize);
    HASSERT(buf);
    memset(buf, 0xff, bufsize);
    unsigned len = file_size;
    while (len > 0)
    {
        ssize_t c = write(fd, buf, std::min(len, bufsize));
        HASSERT(c >= 0);
        len -= c;
    }
    free(buf);

    // Initializes basic structures in the file.
    cfg.version().write(fd, expected_version);
    cfg.next_event().write(fd, 0);
    // ACDI version byte. This is not very nice because we cannot be
    // certain that the EEPROM starts with the ACDI data. We'll check it
    // though.
    HASSERT(SNIP_DYNAMIC_FILENAME == CONFIG_FILENAME);
    Uint8ConfigEntry(0).write(fd, 2);
    factory_reset_all_events(cfg, node()->node_id(), fd);
    configUpdateFlow_.factory_reset();
    return fd;
}

int SimpleStackBase::check_version_and_factory_reset(
    const InternalConfigData &cfg, uint16_t expected_version, bool force)
{
    HASSERT(CONFIG_FILENAME);
    int fd = configUpdateFlow_.open_file(CONFIG_FILENAME);
    if (cfg.version().read(fd) != expected_version)
    {
        /// @todo (balazs.racz): We need to clear the eeprom. Best would be if
        /// there was an ioctl to return the eeprom to factory default state by
        /// just erasing the segments.
        cfg.next_event().write(fd, 0);
        // ACDI version byte. This is not very nice because we cannot be
        // certain that the EEPROM starts with the ACDI data. We'll check it
        // though.
        HASSERT(SNIP_DYNAMIC_FILENAME == CONFIG_FILENAME);
        Uint8ConfigEntry(0).write(fd, 2);
        force = true;
    }
    if (force)
    {
        factory_reset_all_events(cfg, node()->node_id(), fd);
        configUpdateFlow_.factory_reset();
        cfg.version().write(fd, expected_version);
    }
    return fd;
}

/// Contains an array describing each position in the Configuration space that
/// is occupied by an Event ID from a producer or consumer. These Event IDs
/// will be reset to increasing event numbers upon factory reset. The array is
/// exported by the cdi compilation mechanism (in CompileCdiMain.cxx) and
/// defined by cdi.o for the linker.
extern const uint16_t CDI_EVENT_OFFSETS[];
const uint16_t *cdi_event_offsets_ptr = CDI_EVENT_OFFSETS;

void SimpleStackBase::set_event_offsets(const vector<uint16_t> *offsets)
{
    cdi_event_offsets_ptr = &(*offsets)[0];
}

void SimpleStackBase::factory_reset_all_events(
    const InternalConfigData &cfg, uint64_t node_id, int fd)
{
    // First we find the event count.
    uint16_t new_next_event = cfg.next_event().read(fd);
    uint16_t next_event = new_next_event;
    for (unsigned i = 0; cdi_event_offsets_ptr[i]; ++i)
    {
        ++new_next_event;
    }
    // We block off the event IDs first.
    cfg.next_event().write(fd, new_next_event);
    // Then we write them to eeprom.
    for (unsigned i = 0; cdi_event_offsets_ptr[i]; ++i)
    {
        EventId id = node_id;
        id <<= 16;
        id |= next_event++;
        EventConfigEntry(cdi_event_offsets_ptr[i]).write(fd, id);
    }
}

void SimpleCanStackBase::add_gridconnect_port(
    const char *path, Notifiable *on_exit)
{
    int fd = ::open(path, O_RDWR);
    HASSERT(fd >= 0);
    LOG(INFO, "Adding device %s as fd %d", path, fd);
    create_gc_port_for_can_hub(can_hub(), fd, on_exit);
}

#if defined(__linux__) || defined(__MACH__)
void SimpleCanStackBase::add_gridconnect_tty(
    const char *device, Notifiable *on_exit)
{
    int fd = ::open(device, O_RDWR);
    HASSERT(fd >= 0);
    LOG(INFO, "Adding device %s as fd %d", device, fd);
    create_gc_port_for_can_hub(can_hub(), fd, on_exit);

    HASSERT(!tcflush(fd, TCIOFLUSH));
    struct termios settings;
    HASSERT(!tcgetattr(fd, &settings));
    cfmakeraw(&settings);
    cfsetspeed(&settings, B115200);
    HASSERT(!tcsetattr(fd, TCSANOW, &settings));
}
#endif
#if defined(__linux__)
void SimpleCanStackBase::add_socketcan_port_select(
    const char *device, int loopback)
{
    int s = socketcan_open(device, loopback);
    if (s >= 0)
    {
        auto *port = new HubDeviceSelect<CanHubFlow>(can_hub(), s);
        additionalComponents_.emplace_back(port);
    }
}
#endif
extern Pool *const __attribute__((__weak__)) g_incoming_datagram_allocator =
    init_main_buffer_pool();

} // namespace openlcb

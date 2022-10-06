/*
 * SPDX-FileCopyrightText: 2017-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#ifndef NODEID_MEMORY_CONFIG_SPACE_HXX_
#define NODEID_MEMORY_CONFIG_SPACE_HXX_

#include <NodeIdConfigurationGroup.hxx>
#include <NvsManager.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/Singleton.hxx>

#include <utils/StringUtils.hxx>
#include "sdkconfig.h"

namespace esp32cs
{

/// Node configuration holder
NodeIdConfig node_id_config(CONFIG_OLCB_NODEID_MEMORY_SPACE_OFFSET);

/// Virtual memory space that allows reconfiguration of the persistent node
/// identifier.
class NodeIdMemoryConfigSpace
    : public openlcb::VirtualMemorySpace,
      public Singleton<NodeIdMemoryConfigSpace>
{
public:
    /// Constructor.
    ///
    /// @param stack is the @ref SimpleStackBase that this memory space should
    /// be registered with.
    /// @param node_id is the current node identifier.
    NodeIdMemoryConfigSpace(openlcb::SimpleStackBase *stack, NvsManager *nvs)
      : nvs_(nvs), id_(utils::node_id_to_string(nvs->node_id())),
        nodeid_(nvs->node_id())
    {
        register_string(node_id_config.node_id(),
            [&](unsigned repeat, string *contents, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                LOG(VERBOSE, "[NodeIdMemCfg-READ] %s", id_.c_str());
                *contents = id_;
            },
            [&](unsigned repeat, string contents, BarrierNotifiable *done)
            {
                AutoNotify n(done);
                LOG(VERBOSE, "[NodeIdMemCfg-WRITE] %s", contents.c_str());
                uint64_t new_node_id = utils::string_to_uint64(contents);
                nvs->node_id(new_node_id);
                updated_ = true;
                nodeid_ = new_node_id;
                id_ = std::move(contents);
            }
        );
        LOG(INFO, "[NodeIdMemCfg:%02x] NodeID: %s", SPACE, id_.c_str());
        stack->memory_config_handler()->registry()->insert(
            stack->node(), SPACE, this);
    }

    /// Returns the currently configured node identifier.
    uint64_t node_id()
    {
        return nodeid_;
    }

    /// @return true if the node identifier has been changed via this virtual
    /// memory space, false otherwise.
    bool updated()
    {
        return updated_;
    }
private:
    static constexpr uint8_t SPACE = CONFIG_OLCB_NODEID_MEMORY_SPACE_ID;
    NvsManager *nvs_;
    /// temporary holder for the node id in a hex string format.
    /// NOTE: the value will be a dot expanded hex format,
    /// ie: 05.02.01.03.10.00.
    std::string id_;

    /// temporary holder for the currently assigned node id.
    uint64_t nodeid_{0};

    /// Flag indicating that the node-id has been changed via the memory space.
    bool updated_{false};
};

} // namespace esp32cs

#endif // NODEID_MEMORY_CONFIG_SPACE_HXX_

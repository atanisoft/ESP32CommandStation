/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#include "LocoManager.hxx"

#include <openlcb/If.hxx>
#include <openlcb/TractionTrain.hxx>

namespace locomgr
{

openlcb::TrainService *LocoManager::train_service()
{
    return trainService_;
}

bool LocoManager::is_known_train_node(openlcb::Node *node)
{
    return train_service()->is_known_train_node(node);
}

openlcb::If *LocoManager::iface()
{
    return train_service()->iface();
}

} // namespace locomgr

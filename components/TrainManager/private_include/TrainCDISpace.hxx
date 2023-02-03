/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef TRAINCDISPACE_HXX_
#define TRAINCDISPACE_HXX_

#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>

namespace trainmanager
{

class TrainManager;

class TrainCDISpace : public openlcb::MemorySpace
{
public:
    TrainCDISpace(TrainManager *parent);

    bool set_node(openlcb::Node* node) override;

    openlcb::MemorySpace::address_t max_address() override;

    size_t read(address_t source, uint8_t* dst, size_t len, errorcode_t* error,
                Notifiable* again) override;

private:
    TrainManager *parent_;
    openlcb::MemorySpace *proxySpace_;
};

} // namespace trainmanager

#endif // TRAINCDISPACE_HXX_
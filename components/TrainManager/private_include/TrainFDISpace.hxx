/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef TRAINFDISPACE_HXX_
#define TRAINFDISPACE_HXX_

#include <executor/Notifiable.hxx>
#include <FdiXmlGenerator.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>

namespace trainmanager
{
class TrainManager;

class TrainFDISpace : public openlcb::MemorySpace
{
public:
    TrainFDISpace(TrainManager *parent);
    bool set_node(openlcb::Node *node) override;
    openlcb::MemorySpace::address_t max_address() override;
    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                Notifiable *again) override;

private:
    FdiXmlGenerator gen_;
    TrainManager *parent_;
    openlcb::Node *node_{nullptr};
    void reset_file();
};

} // namespace trainmanager

#endif // TRAINFDISPACE_HXX_
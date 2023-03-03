/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2023 Mike Dunston (atanisoft)
 *
 */


#ifndef PERSISTENTTRAINCONFIGSPACE_HXX_
#define PERSISTENTTRAINCONFIGSPACE_HXX_

#include <functional>
#include <openlcb/Node.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <locodb/LocoDatabaseEntry.hxx>

namespace trainmanager
{

class TrainManager;

class PersistentTrainConfigSpace : public openlcb::VirtualMemorySpace
{
public:
    PersistentTrainConfigSpace(TrainManager *parent);
    bool set_node(openlcb::Node *node) override;

private:
    template <typename T>
    typename std::function<T(unsigned repeat, BarrierNotifiable *done)>
    typed_reader(int index);
    template <typename T>
    std::function<void(unsigned repeat, T contents, BarrierNotifiable *done)>
    typed_writer(int index);

    std::function<void(unsigned repeat, string *contents, BarrierNotifiable *done)> string_reader(int index);
    std::function<void(unsigned repeat, string contents, BarrierNotifiable *done)> string_writer(int index);

    TrainManager *parent_;
    LazyInitTrainNode *impl_{nullptr};
    std::shared_ptr<locodb::LocoDatabaseEntry> train_;
};

} // namespace trainmanager

#endif // PERSISTENTTRAINCONFIGSPACE_HXX_PERSISTENTTRAINCONFIGSPACE_HXX_
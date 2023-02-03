/*
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef LAZYINITTRAINNODE_HXX_
#define LAZYINITTRAINNODE_HXX_

#include <openlcb/MemoryConfig.hxx>
#include <openlcb/TractionTrain.hxx>
#include <locodb/Defs.hxx>

namespace openlcb
{
class DefaultTrainNode;
class TrainService;
}

namespace trainmanager
{

class LazyInitTrainNode : public openlcb::DefaultTrainNode
{
public:
  /// Constructor.
  /// @param service the traction service object that will own this node.
  /// @param offset the @ref TrainDb assigned identifier for this node.
  /// @param mode the @ref DriveMode that this node should use.
  /// @param address the address of this node.
  LazyInitTrainNode(openlcb::TrainService *service, ssize_t offset,
                    locodb::DriveMode mode,
                    uint16_t address);
  ~LazyInitTrainNode();
  openlcb::NodeID node_id() override;
  uint16_t address();
  locodb::DriveMode mode();
  ssize_t file_offset();
  bool is_allocated();
  openlcb::TrainImpl *train() override;
private:
  ssize_t offset_;
  locodb::DriveMode mode_;
  uint16_t addr_;
};

} // namespace trainmanager

#endif // LAZYINITTRAINNODE_HXX_
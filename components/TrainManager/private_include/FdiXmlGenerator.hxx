/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#ifndef FDIXMLGENERATOR_HXX_
#define FDIXMLGENERATOR_HXX_

#include "XmlGenerator.hxx"

#include <memory>
#include <locodb/LocoDatabaseEntry.hxx>

namespace trainmanager
{

class FdiXmlGenerator : public XmlGenerator
{
 public:
  /// Call this after the lokdb on entry was overwritten with the new loco's
  /// data.
  void reset(std::shared_ptr<locodb::LocoDatabaseEntry> entry);

 private:
  /// Generates XML field data for the next state.
  void generate_more() override;

  /// States used as part of generating the FDI XML data.
  enum State
  {
    STATE_START = 0,
    STATE_XMLHEAD = STATE_START,
    STATE_START_FN,
    STATE_FN_NAME,
    STATE_FN_NUMBER,
    STATE_FN_END,
    STATE_NO_MORE_FN,
    STATE_EOF
  };

  State state_;
  std::shared_ptr<locodb::LocoDatabaseEntry> entry_;
  int nextFunction_;
};

}  // namespace trainmanager

#endif // FDIXMLGENERATOR_HXX_
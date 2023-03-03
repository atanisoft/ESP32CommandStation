/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2023 Mike Dunston (atanisoft)
 *
 */

#include "FdiXmlGenerator.hxx"
#include <locodb/Defs.hxx>

namespace trainmanager
{

using locodb::LocoDatabaseEntry;
using locodb::function_to_string;

static const char kFdiXmlHead[] = R"(<?xml version='1.0' encoding='UTF-8'?>
<?xml-stylesheet type='text/xsl' href='xslt/fdi.xsl'?>
<fdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/trunk/prototypes/xml/schema/fdi.xsd'>
<segment space='249'><group><name/>
)";
static const char kFdiXmlTail[] = "</group></segment></fdi>";

static const char kFdiXmlBinaryFunction[] =
    R"(<function size='1' kind='binary'>
)";

static const char kFdiXmlMomentaryFunction[] =
    R"(<function size='1' kind='momentary'>
)";

void FdiXmlGenerator::reset(std::shared_ptr<LocoDatabaseEntry> lok)
{
  state_ = STATE_START;
  entry_ = lok;
  internal_reset();
}

void FdiXmlGenerator::generate_more()
{
  while (true)
  {
    switch (state_)
    {
      case STATE_XMLHEAD:
      {
        add_to_output(from_const_string(kFdiXmlHead));
        nextFunction_ = 0;
        state_ = STATE_START_FN;
        return;
      }
      case STATE_START_FN:
      {
        while (
            nextFunction_ <= entry_->get_max_fn() &&
            !entry_->is_function_valid(nextFunction_))
        {
          ++nextFunction_;
        }
        if (nextFunction_ > entry_->get_max_fn())
        {
          state_ = STATE_NO_MORE_FN;
          continue;
        }
        if (entry_->is_function_momentary(nextFunction_))
        {
          add_to_output(from_const_string(kFdiXmlMomentaryFunction));
        }
        else
        {
          add_to_output(from_const_string(kFdiXmlBinaryFunction));
        }
        state_ = STATE_FN_NAME;
        return;
      }
      case STATE_FN_NAME:
      {
        add_to_output(from_const_string("<name>"));
        const char* label =
          function_to_string(entry_->get_function_def(nextFunction_));
        if (label)
        {
          add_to_output(from_const_string(label));
        }
        else
        {
          add_to_output(from_const_string("F"));
          add_to_output(from_integer(nextFunction_));
        }
        add_to_output(from_const_string("</name>\n"));
        state_ = STATE_FN_NUMBER;
        return;
      }
      case STATE_FN_NUMBER:
      {
        add_to_output(from_const_string("<number>"));
        add_to_output(from_integer(nextFunction_));
        add_to_output(from_const_string("</number>\n</function>\n"));
        state_ = STATE_FN_END;
        return;
      }
      case STATE_FN_END:
      {
        ++nextFunction_;
        state_ = STATE_START_FN;
        continue;
      }
      case STATE_NO_MORE_FN:
      {
        add_to_output(from_const_string(kFdiXmlTail));
        state_ = STATE_EOF;
        return;
      }
      case STATE_EOF:
      {
        return;
      }
    }
  }
}

}  // namespace trainmanager
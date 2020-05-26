/*! \file */

#ifndef __NEONEXTION_INEXTIONCOLOURABLE
#define __NEONEXTION_INEXTIONCOLOURABLE

#include "NeoNextion.h"
#include "INextionWidget.h"
#include "NextionTypes.h"

/*!
 * \class INextionColourable
 * \brief Interface for widgets that can be coloured.
 */
class INextionColourable : public virtual INextionWidget
{
public:
  INextionColourable(Nextion &nex, uint8_t page, uint8_t component,
                     const std::string &name);

  bool setForegroundColour(uint32_t colour, bool refresh = true);
  uint32_t getForegroundColour();

  bool setEventForegroundColour(uint32_t colour, bool refresh = true);
  uint32_t getEventForegroundColour();

  bool setBackgroundColour(uint32_t colour, bool refresh = true);
  uint32_t getBackgroundColour();

  bool setEventBackgroundColour(uint32_t colour, bool refresh = true);
  uint32_t getEventBackgroundColour();

  bool setColour(const std::string &type, uint32_t colour, bool refresh);
  uint32_t getColour(const std::string &type);

  bool afterSet(bool result, bool refresh);
};

#endif

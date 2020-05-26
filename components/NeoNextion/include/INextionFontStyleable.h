/*! \file */

#ifndef __NEONEXTION_INEXTIONFONTSTYLEABLE
#define __NEONEXTION_INEXTIONFONTSTYLEABLE

#include "NeoNextion.h"
#include "INextionWidget.h"
#include "NextionTypes.h"

/*!
 * \class INextionFontStyleable
 * \brief Interface for widgets that can have their fonts styled.
 */
class INextionFontStyleable : public virtual INextionWidget
{
public:
  INextionFontStyleable(Nextion &nex, uint8_t page, uint8_t component,
                        const std::string &name);

  bool setFont(uint8_t id, bool refresh = true);
  uint8_t getFont();

  bool setHAlignment(NextionFontAlignment align, bool refresh = true);
  NextionFontAlignment getHAlignment();

  bool setVAlignment(NextionFontAlignment align, bool refresh = true);
  NextionFontAlignment getVAlignment();

  bool afterSet(bool result, bool refresh);
};

#endif

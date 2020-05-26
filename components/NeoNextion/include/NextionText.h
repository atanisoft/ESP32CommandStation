/*! \file */

#ifndef __NEONEXTION_NEXTIONTEXT
#define __NEONEXTION_NEXTIONTEXT

#include "NeoNextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionStringValued.h"
#include "INextionFontStyleable.h"

/*!
 * \class NextionText
 * \brief Represents a text widget.
 */
class NextionText : public INextionTouchable,
                    public INextionColourable,
                    public INextionStringValued,
                    public INextionFontStyleable
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionText(Nextion &nex, uint8_t page, uint8_t component, const std::string &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
      , INextionColourable(nex, page, component, name)
      , INextionStringValued(nex, page, component, name)
      , INextionFontStyleable(nex, page, component, name)
  {
  }
};

#endif

/*! \file */

#ifndef __NEONEXTION_NEXTIONBUTTON
#define __NEONEXTION_NEXTIONBUTTON

#include "Nextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionStringValued.h"
#include "INextionFontStyleable.h"

/*!
 * \class NextionButton
 * \brief Represents a basic button widget.
 */
class NextionButton : public INextionTouchable,
                      public INextionColourable,
                      public INextionStringValued,
                      public INextionFontStyleable
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionButton(Nextion &nex, uint8_t page, uint8_t component, const String &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
      , INextionColourable(nex, page, component, name)
      , INextionStringValued(nex, page, component, name)
      , INextionFontStyleable(nex, page, component, name)
  {
  }
};

#endif

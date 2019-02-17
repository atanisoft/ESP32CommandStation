/*! \file */

#ifndef __NEONEXTION_NEXTIONCHECKBOX
#define __NEONEXTION_NEXTIONCHECKBOX

#include "Nextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionBooleanValued.h"

/*!
 * \class NextionCheckbox
 * \brief Represents a checkbox.
 */
class NextionCheckbox : public INextionTouchable,
                        public INextionColourable,
                        public INextionBooleanValued
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionCheckbox(Nextion &nex, uint8_t page, uint8_t component,
                  const String &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
      , INextionColourable(nex, page, component, name)
      , INextionBooleanValued(nex, page, component, name)
  {
  }
};

#endif

/*! \file */

#ifndef __NEONEXTION_NEXTIONVARIABLENUMERIC
#define __NEONEXTION_NEXTIONVARIABLENUMERIC

#include "Nextion.h"
#include "INextionNumericalValued.h"

/*!
 * \class NextionVariableNumeric
 * \brief Represents a numeric variable.
 */
class NextionVariableNumeric : public INextionNumericalValued
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionVariableNumeric(Nextion &nex, uint8_t page, uint8_t component,
                         const String &name)
      : INextionWidget(nex, page, component, name)
      , INextionNumericalValued(nex, page, component, name)
  {
  }
};

#endif

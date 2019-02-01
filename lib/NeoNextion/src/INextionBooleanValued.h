/*! \file */

#ifndef __NEONEXTION_INEXTIONBOOLEANVALUED
#define __NEONEXTION_INEXTIONBOOLEANVALUED

#include "Nextion.h"
#include "INextionNumericalValued.h"
#include "NextionTypes.h"

/*!
 * \class INextionBooleanValued
 * \brief Interface for widgets that store a boolean value.
 *
 * Assumes that the boolean value is a property named "val".
 */
class INextionBooleanValued : private INextionNumericalValued
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  INextionBooleanValued(Nextion &nex, uint8_t page, uint8_t component,
                        const String &name)
      : INextionWidget(nex, page, component, name)
      , INextionNumericalValued(nex, page, component, name)
  {
  }

  /*!
   * \brief Gets the state of the boolean value of the control.
   * \return True if boolean state is active
   */
  bool isActive()
  {
    return getValue();
  }

  /*!
   * \brief Sets the state of the boolean value.
   * \param active State
   * \return True if successful
   */
  bool setActive(bool active)
  {
    return setValue((uint32_t)active);
  }
};

#endif

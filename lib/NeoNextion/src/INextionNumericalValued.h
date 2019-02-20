/*! \file */

#ifndef __NEONEXTION_INEXTIONNUMERICALVALUED
#define __NEONEXTION_INEXTIONNUMERICALVALUED

#include "Nextion.h"
#include "INextionWidget.h"
#include "NextionTypes.h"

/*!
 * \class INextionNumericalValued
 * \brief Interface for widgets that store a numerical value.
 *
 * Assumes that the numerical value is a property named "val".
 */
class INextionNumericalValued : public virtual INextionWidget
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  INextionNumericalValued(Nextion &nex, uint8_t page, uint8_t component,
                          const String &name)
      : INextionWidget(nex, page, component, name)
  {
  }

  /*!
   * \brief Gets the numerical value.
   * \return Value
   * \see INextionNumericalValued::setValue
   */
  uint32_t getValue()
  {
    return getNumberProperty("val");
  }

  /*!
   * \brief Sets the numerical value.
   * \param value Value
   * \return True if successful
   * \see INextionNumericalValued::getValue
   */
  bool setValue(uint32_t value)
  {
    return setNumberProperty("val", value);
  }
};

#endif

/*! \file */

#ifndef __NEONEXTION_INEXTIONSTRINGVALUED
#define __NEONEXTION_INEXTIONSTRINGVALUED

#include "NeoNextion.h"
#include "INextionWidget.h"
#include "NextionTypes.h"

/*!
 * \class INextionStringValued
 * \brief Interface for widgets that hold a string value.
 *
 * Assumes that the string value is a property named "txt".
 */
class INextionStringValued : public virtual INextionWidget
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  INextionStringValued(Nextion &nex, uint8_t page, uint8_t component,
                       const std::string &name)
      : INextionWidget(nex, page, component, name)
  {
  }

  /*!
   * \brief Gets the value of the string.
   * \param buffer Pointer to storage to strore string in
   * \param len Maximum length of string
   * \return Actual length of string
   * \see INextionStringValued::setText
   */
  size_t getText(std::string &buffer)
  {
    return getStringProperty("txt", buffer);
  }

  /*!
   * \brief Sets the value of the string.
   * \param buffer Value
   * \return True if successful
   * \see INextionStringValued::getText
   */
  bool setText(const std::string &buffer)
  {
    return setStringProperty("txt", buffer);
  }

  /*!
   * \brief Sets the text by a numercal value.
   * \param value Numerical value
   * \return True if successful
   * \see INextionStringValued::getTextAsNumber
   */
  bool setTextAsNumber(uint32_t value)
  {
    return setStringProperty("txt", std::to_string(value));
  }

  /*!
   * \brief Gets the text parsed as a number.
   * \return Numerical value
   * \see INextionStringValued::setTextAsNumber
   */
  uint32_t getTextAsNumber()
  {
    std::string buffer;
    if (getStringProperty("txt", buffer))
    {
      return std::stoi(buffer);
    }
    else
      return 0;
  }
};

#endif

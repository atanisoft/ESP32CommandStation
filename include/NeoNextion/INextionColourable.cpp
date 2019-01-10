/*! \file */

#include "INextionColourable.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
INextionColourable::INextionColourable(Nextion &nex, uint8_t page,
                                       uint8_t component, const String &name)
    : INextionWidget(nex, page, component, name)
{
}

/*!
 * \brief Sets the normal foreground colour.
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionColourable::getForegroundColour
 */
bool INextionColourable::setForegroundColour(uint32_t colour, bool refresh)
{
  return setColour("pco", colour, refresh);
}

/*!
 * \brief Gets the normal foreground colour.
 * \return Colour (may also return 0 in case of error)
 * \see INextionColourable::setForegroundColour
 */
uint32_t INextionColourable::getForegroundColour()
{
  return getColour("pco");
}

/*!
 * \brief Sets the foreground colour when a touch event is active.
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionColourable::getEventForegroundColour
 */
bool INextionColourable::setEventForegroundColour(uint32_t colour, bool refresh)
{
  return setColour("pco2", colour, refresh);
}

/*!
 * \brief Gets the foreground colour when a touch event is active.
 * \return Colour (may also return 0 in case of error)
 * \see INextionColourable::setEventForegroundColour
 */
uint32_t INextionColourable::getEventForegroundColour()
{
  return getColour("pco2");
}

/*!
 * \brief Sets the normal background colour.
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionColourable::getBackgroundColour
 */
bool INextionColourable::setBackgroundColour(uint32_t colour, bool refresh)
{
  return setColour("bco", colour, refresh);
}

/*!
 * \brief Gets the normal background colour.
 * \return Colour (may also return 0 in case of error)
 * \see INextionColourable::setBackgroundColour
 */
uint32_t INextionColourable::getBackgroundColour()
{
  return getColour("bco");
}

/*!
 * \brief Sets the background colour when a touch event is active.
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionColourable::getEventBackgroundColour
 */
bool INextionColourable::setEventBackgroundColour(uint32_t colour, bool refresh)
{
  return setColour("bco2", colour, refresh);
}

/*!
 * \brief Sets the background colour when a touch event is active.
 * \return Colour (may also return 0 in case of error)
 * \see INextionColourable::setEventBackgroundColour
 */
uint32_t INextionColourable::getEventBackgroundColour()
{
  return getColour("bco2");
}

/*!
 * \brief Sets a colour by its property name.
 * \param type Property name
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 */
bool INextionColourable::setColour(const String &type, uint32_t colour, bool refresh)
{
  return afterSet(setNumberProperty(type, colour), refresh);
}

/*!
 * \brief Gets a colour by its property name.
 * \param type Property name
 * \return Colour (may also return 0 in case of error)
 * \see INextionColourable::setColour
 */
uint32_t INextionColourable::getColour(const String &type)
{
  return getNumberProperty(type);
}

/*!
 * \brief Handles refreshing the page after a colour has been changed.
 * \param result Success of colour set
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionColourable::getColour
 */
bool INextionColourable::afterSet(bool result, bool refresh)
{
  if (result)
  {
    if (refresh)
    {
      m_nextion.refresh(m_name);
      return m_nextion.checkCommandComplete();
    }
    else
      return true;
  }
  else
    return false;
}

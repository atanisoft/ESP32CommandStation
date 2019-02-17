/*! \file */

#include "INextionWidget.h"

/*!
 * \brief Create a new widget adapter.
 * \param nex Reference to the Nextion driver
 * \param page ID of page this widget is on
 * \param component Component ID of this widget
 * \param name Name of this widget
 */
INextionWidget::INextionWidget(Nextion &nex, uint8_t page, uint8_t component,
                               const String &name)
    : m_nextion(nex)
    , m_pageID(page)
    , m_componentID(component)
    , m_name(name)
{
}

/*!
 * \brief Gets the ID of the page this widget resides on.
 * \return Page ID
 */
uint8_t INextionWidget::getPageID()
{
  return m_pageID;
}

/*!
 * \brief Gets the component ID of this widget.
 * \return Component ID
 */
uint8_t INextionWidget::getComponentID()
{
  return m_componentID;
}

/*!
 * \brief Sets the value of a numerical property of this widget.
 * \param propertyName Name of the property
 * \param value Value
 * \return True if successful
 */
bool INextionWidget::setNumberProperty(const String &propertyName, uint32_t value)
{
  return sendCommandWithWait("%s.%s=%d", m_name.c_str(), propertyName.c_str(), value);
}

/*!
 * \brief Gets the value of a numerical property of this widget.
 * \param propertyName Name of the property
 * \return Value (may also return 0 in case of error)
 */
uint32_t INextionWidget::getNumberProperty(const String &propertyName)
{
  sendCommand("get %s.%s", m_name.c_str(), propertyName.c_str());
  uint32_t id;
  if (m_nextion.receiveNumber(&id))
    return id;
  else
    return 0;
}

/*!
 * \brief Sets the value of a string property of this widget.
 * \param propertyName Name of the property
 * \param value Value
 * \return True if successful
 */
bool INextionWidget::setStringProperty(const String &propertyName, const String &value)
{
  return sendCommandWithWait("%s.%s=\"%s\"", m_name.c_str(), propertyName.c_str(), value.c_str());
}

/*!
 * \brief Gets the value of a string property of this widget.
 * \param propertyName Name of the property
 * \param value Pointer to char array to store result in
 * \param len Maximum length of value
 * \return Actual length of value
 */
size_t INextionWidget::getStringProperty(const String &propertyName, String &buffer)
{
  sendCommand("get %s.%s", m_name.c_str(), propertyName.c_str());
  return m_nextion.receiveString(buffer);
}

void INextionWidget::sendCommand(const String &format, ...)
{
  va_list args;
  va_start(args, format);
  m_nextion.sendCommand(format.c_str(), args);
  va_end(args);
}

bool INextionWidget::sendCommandWithWait(const String &format, ...)
{
  va_list args;
  va_start(args, format);
  m_nextion.sendCommand(format.c_str(), args);
  va_end(args);

  return m_nextion.checkCommandComplete();
}

bool INextionWidget::setPropertyCommand(const String &command, uint32_t value)
{
  m_nextion.sendCommand("%s %s,%ld", command.c_str(), m_name.c_str(), value);
  return m_nextion.checkCommandComplete();
}

bool INextionWidget::show()
{
  return setPropertyCommand("vis", 1);
}

bool INextionWidget::hide()
{
  return setPropertyCommand("vis", 0);
}

bool INextionWidget::enable()
{
  return setPropertyCommand("tsw", 1);
}

bool INextionWidget::disable()
{
  return setPropertyCommand("tsw", 0);
}

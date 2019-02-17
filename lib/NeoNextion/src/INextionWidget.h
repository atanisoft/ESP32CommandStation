/*! \file */

#ifndef __NEONEXTION_INEXTIONWIDGET
#define __NEONEXTION_INEXTIONWIDGET

#include "Nextion.h"

/*!
 * \class INextionWidget
 * \brief Abstract class for all UI widgets.
 *
 * Widget objects act as a adapter/API for the widgets defined in the Nextion
 * Editor software.
 */
class INextionWidget
{
public:
  INextionWidget(Nextion &nex, uint8_t page, uint8_t component,
                 const String &name);

  uint8_t getPageID();
  uint8_t getComponentID();

  bool setNumberProperty(const String &propertyName, uint32_t value);
  uint32_t getNumberProperty(const String &propertyName);
  bool setPropertyCommand(const String &command, uint32_t value);
  bool setStringProperty(const String &propertyName, const String &value);
  size_t getStringProperty(const String &propertyName, String &buffer);

  bool show();
  bool hide();
  bool enable();
  bool disable();

protected:
  void sendCommand(const String &format, ...);
  bool sendCommandWithWait(const String &format, ...);

protected:
  Nextion &m_nextion;    //!< Reference to the Nextion driver
  uint8_t m_pageID;      //!< ID of page this widget is on
  uint8_t m_componentID; //!< Component ID of this widget
  const String m_name;  //!< Name of this widget
};

#endif

/*! \file */

#ifndef __NEONEXTION_INEXTIONWIDGET
#define __NEONEXTION_INEXTIONWIDGET

#include "NeoNextion.h"

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
                 const std::string &name);

  uint8_t getPageID();
  uint8_t getComponentID();

  bool setNumberProperty(const std::string &propertyName, uint32_t value);
  uint32_t getNumberProperty(const std::string &propertyName);
  bool setPropertyCommand(const std::string &command, uint32_t value);
  bool setStringProperty(const std::string &propertyName, const std::string &value);
  size_t getStringProperty(const std::string &propertyName, std::string &buffer);

  bool show();
  bool hide();
  bool enable();
  bool disable();

protected:
  void sendCommand(const std::string &format, ...);
  bool sendCommandWithWait(const std::string &format, ...);

protected:
  Nextion &m_nextion;    //!< Reference to the Nextion driver
  uint8_t m_pageID;      //!< ID of page this widget is on
  uint8_t m_componentID; //!< Component ID of this widget
  const std::string m_name;  //!< Name of this widget
};

#endif

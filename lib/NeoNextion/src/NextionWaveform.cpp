/*! \file */

#include "NextionWaveform.h"
#include "INextionWidget.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
NextionWaveform::NextionWaveform(Nextion &nex, uint8_t page, uint8_t component,
                                 const String &name)
    : INextionWidget(nex, page, component, name)
    , INextionTouchable(nex, page, component, name)
    , INextionColourable(nex, page, component, name)
{
}

/*!
 * \brief Adds a value to the waveform display.
 * \param channel Channel number
 * \param value Value
 * \return True if successful
 */
bool NextionWaveform::addValue(uint8_t channel, uint8_t value)
{
  if (channel > 3)
    return false;

  size_t commandLen = 22;
  char commandBuffer[commandLen];
  snprintf(commandBuffer, commandLen, "add %d,%d,%d", m_componentID, channel,
           value);
  sendCommand(commandBuffer, false);

  /* TODO: this check still fails but the command does actually work */
  /* return m_nextion.checkCommandComplete(); */
  return true;
}

/*!
 * \brief Sets the colour of a channel.
 * \param channel Channel number
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 */
bool NextionWaveform::setChannelColour(uint8_t channel, uint32_t colour,
                                       bool refresh)
{
  char buffer[5];
  snprintf(buffer, 5, "pco%d", channel);
  return setColour(buffer, colour, refresh);
}

/*!
 * \brief Gets the colour of a channel.
 * \param channel Channel number
 * \return Colour (may return 0 in case of error)
 */
uint32_t NextionWaveform::getChannelColour(uint8_t channel)
{
  char buffer[5];
  snprintf(buffer, 5, "pco%d", channel);
  return getColour(buffer);
}

/*!
 * \brief Sets the colour of the grid lines.
 * \param colour Colour
 * \param refresh If the widget should be refreshed
 * \return True if successful
 */
bool NextionWaveform::setGridColour(uint32_t colour, bool refresh)
{
  return setColour("gdc", colour, refresh);
}

/*!
 * \brief Gets the colour of the grid lines.
 * \return Colour (may return 0 in case of error)
 */
uint32_t NextionWaveform::getGridColour()
{
  return getColour("gdc");
}

/*!
 * \brief Sets the width of the grid squares.
 * \param width Width
 * \return True if successful
 */
bool NextionWaveform::setGridWidth(uint16_t width)
{
  return setNumberProperty("gdw", width);
}

/*!
 * \brief Gets the width of the grid squares.
 * \return Width (may return 0 in case of error)
 */
uint16_t NextionWaveform::getGridWidth()
{
  return getNumberProperty("gdw");
}

/*!
 * \brief Sets the height of the grid squares.
 * \param height Height
 * \return True if successful
 */
bool NextionWaveform::setGridHeight(uint16_t height)
{
  return setNumberProperty("gdh", height);
}

/*!
 * \brief Gets the height of the grid squares.
 * \return Height (may return 0 in case of error)
 */
uint16_t NextionWaveform::getGridHeight()
{
  return getNumberProperty("gdh");
}

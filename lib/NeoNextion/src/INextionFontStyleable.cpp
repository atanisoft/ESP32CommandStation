/*! \file */

#include "INextionFontStyleable.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
INextionFontStyleable::INextionFontStyleable(Nextion &nex, uint8_t page,
                                             uint8_t component,
                                             const String &name)
    : INextionWidget(nex, page, component, name)
{
}

/*!
 * \brief Sets the active font for the text.
 * \param id Font ID
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionFontStyleable::getFont
 */
bool INextionFontStyleable::setFont(uint8_t id, bool refresh)
{
  return afterSet(setNumberProperty("font", id), refresh);
}

/*!
 * \brief Gets the active font for the text.
 * \return Font ID (may return 0 in event of error)
 * \see INextionFontStyleable::setFont
 */
uint8_t INextionFontStyleable::getFont()
{
  return getNumberProperty("font");
}

/*!
 * \brief Sets the horizontal alignment of the text.
 * \param align Alignment
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionFontStyleable::getHAlignment
 */
bool INextionFontStyleable::setHAlignment(NextionFontAlignment align,
                                          bool refresh)
{
  return afterSet(setNumberProperty("xcen", align), refresh);
}

/*!
 * \brief Gets the horizontal alignment of the text.
 * \return Alignment
 * \see INextionFontStyleable::setHAlignment
 */
NextionFontAlignment INextionFontStyleable::getHAlignment()
{
  return (NextionFontAlignment)getNumberProperty("xcen");
}

/*!
 * \brief Sets the vertical alignment of the text.
 * \param align Alignment
 * \param refresh If the widget should be refreshed
 * \return True if successful
 * \see INextionFontStyleable::getVAlignment
 */
bool INextionFontStyleable::setVAlignment(NextionFontAlignment align,
                                          bool refresh)
{
  return afterSet(setNumberProperty("ycen", align), refresh);
}

/*!
 * \brief Gets the vertical alignment of the text.
 * \return Alignment
 * \see INextionFontStyleable::setVAlignment
 */
NextionFontAlignment INextionFontStyleable::getVAlignment()
{
  return (NextionFontAlignment)getNumberProperty("ycen");
}

/*!
 * \brief Handles refreshing the page after a style has been changed.
 * \param result Success of style change
 * \param refresh If the widget should be refreshed
 * \return True if successful
 */
bool INextionFontStyleable::afterSet(bool result, bool refresh)
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

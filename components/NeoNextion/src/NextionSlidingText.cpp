/*! \file */

#include "NextionSlidingText.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
NextionSlidingText::NextionSlidingText(Nextion &nex, uint8_t page,
                                       uint8_t component, const std::string &name)
    : INextionWidget(nex, page, component, name)
    , INextionTouchable(nex, page, component, name)
    , INextionColourable(nex, page, component, name)
    , INextionStringValued(nex, page, component, name)
    , INextionFontStyleable(nex, page, component, name)
{
}

bool NextionSlidingText::setScrolling(bool scroll)
{
  return setNumberProperty("en", scroll);
}

bool NextionSlidingText::isScrolling()
{
  return (bool)getNumberProperty("en");
}

bool NextionSlidingText::setScrollDirection(NextionScrollDirection direction)
{
  return setNumberProperty("dir", direction);
}

NextionScrollDirection NextionSlidingText::getScrollDirection()
{
  return (NextionScrollDirection)getNumberProperty("dir");
}

bool NextionSlidingText::setScrollDistance(uint32_t distance)
{
  return setNumberProperty("dis", distance);
}

uint32_t NextionSlidingText::getScrollDistance()
{
  return getNumberProperty("dis");
}

bool NextionSlidingText::setScrollDelay(uint32_t delay)
{
  return setNumberProperty("tim", delay);
}

uint32_t NextionSlidingText::getScrollDelay()
{
  return getNumberProperty("tim");
}

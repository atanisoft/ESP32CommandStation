/*! \file */

#ifndef __NEONEXTION_NEXTIONSLIDINGTEXT
#define __NEONEXTION_NEXTIONSLIDINGTEXT

#include "Nextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionStringValued.h"
#include "INextionFontStyleable.h"

/*!
 * \class NextionSlidingText
 * \brief Represents a sliding text widget.
 */
class NextionSlidingText : public INextionTouchable,
                           public INextionColourable,
                           public INextionStringValued,
                           public INextionFontStyleable
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionSlidingText(Nextion &nex, uint8_t page, uint8_t component,
                     const String &name);

  bool setScrolling(bool scroll);
  bool isScrolling();

  bool setScrollDirection(NextionScrollDirection direction);
  NextionScrollDirection getScrollDirection();

  bool setScrollDistance(uint32_t distance);
  uint32_t getScrollDistance();

  bool setScrollDelay(uint32_t delay);
  uint32_t getScrollDelay();
};

#endif

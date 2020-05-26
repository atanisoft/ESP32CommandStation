/*! \file */

#ifndef __NEONEXTION_NEXTIONHOTSPOT
#define __NEONEXTION_NEXTIONHOTSPOT

#include "NeoNextion.h"
#include "INextionWidget.h"
#include "INextionTouchable.h"

/*!
 * \class NextionHotspot
 * \brief Represents a hotspot widget.
 */
class NextionHotspot : public INextionTouchable
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionHotspot(Nextion &nex, uint8_t page, uint8_t component,
                 const std::string &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
  {
  }
};

#endif

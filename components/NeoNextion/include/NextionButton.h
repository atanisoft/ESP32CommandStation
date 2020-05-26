/*! \file */

#ifndef __NEONEXTION_NEXTIONBUTTON
#define __NEONEXTION_NEXTIONBUTTON

#include "NeoNextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionStringValued.h"
#include "INextionFontStyleable.h"

/*!
 * \class NextionButton
 * \brief Represents a basic button widget.
 */
class NextionButton : public INextionTouchable,
                      public INextionColourable,
                      public INextionStringValued,
                      public INextionFontStyleable
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionButton(Nextion &nex, uint8_t page, uint8_t component, const std::string &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
      , INextionColourable(nex, page, component, name)
      , INextionStringValued(nex, page, component, name)
      , INextionFontStyleable(nex, page, component, name)
  {
  }

  /*!
  * \brief Gets the ID of the currently displayed picture.
  * \return PIcture ID (may also return 0 in case of error)
  */
  uint16_t getPictureID()
  {
    return getNumberProperty("pic");
  }

  /*!
  * \brief Sets the picture to be displayed.
  * \param id Picture ID
  * \return True if successful
  */
  bool setPictureID(uint16_t id)
  {
    return setNumberProperty("pic", id);
  }
};

#endif

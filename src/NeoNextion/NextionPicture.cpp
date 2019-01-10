/*! \file */

#include "NeoNextion/NextionPicture.h"
#include "NeoNextion/INextionWidget.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
NextionPicture::NextionPicture(Nextion &nex, uint8_t page, uint8_t component,
                               const String &name)
    : INextionWidget(nex, page, component, name)
    , INextionTouchable(nex, page, component, name)
{
}

/*!
 * \brief Gets the ID of the currently displayed picture.
 * \return PIcture ID (may also return 0 in case of error)
 */
uint16_t NextionPicture::getPictureID()
{
  return getNumberProperty("pic");
}

/*!
 * \brief Sets the picture to be displayed.
 * \param id Picture ID
 * \return True if successful
 */
bool NextionPicture::setPictureID(uint16_t id)
{
  return setNumberProperty("pic", id);
}

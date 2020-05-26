/*! \file */

#include "INextionTouchable.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
INextionTouchable::INextionTouchable(Nextion &nex, uint8_t page,
                                     uint8_t component, const std::string &name)
    : INextionWidget(nex, page, component, name)
    , m_callback(NULL)
{
  nex.registerTouchable(this);
}

/*!
 * \brief Processes a touch event.
 * \param pageID Page ID of touch event
 * \param componentID Component ID of touch event
 * \param eventType Type of touch event
 * \return True if the event effects this widget
 */
bool INextionTouchable::processEvent(uint8_t pageID, uint8_t componentID,
                                     uint8_t eventType)
{
  if (pageID != m_pageID)
    return false;

  if (componentID != m_componentID)
    return false;

  switch (eventType)
  {
  case NEX_EVENT_PUSH:
    if (m_callback)
      m_callback->handleNextionEvent((NextionEventType)eventType, this);
    return true;

  case NEX_EVENT_POP:
    if (m_callback)
      m_callback->handleNextionEvent((NextionEventType)eventType, this);
    return true;

  default:
    return false;
  }
}

/*!
 * \brief Attaches a callback function to this widget.
 * \param function Pointer to callback function
 * \return True if successful
 * \see INextionTouchable::detachCallback
 */
bool INextionTouchable::attachCallback(
    NextionCallbackFunctionHandler::NextionFunction function)
{
  if (!function)
    return false;

  if (m_callback != NULL)
    detachCallback();

  m_callback = new NextionCallbackFunctionHandler(function);
  return true;
}

/*!
 * \brief Attaches a callback handler to this widget.
 * \param handler Pointer to handler
 * \return True if successful
 * \see INextionTouchable::detachCallback
 */
bool INextionTouchable::attachCallback(INextionCallback *handler)
{
  if (!handler)
    return false;

  if (m_callback != NULL)
    detachCallback();

  m_callback = handler;
  return true;
}

/*!
 * \brief Removes the callback handler from this widget
 *
 * Memory is not freed.
 */
void INextionTouchable::detachCallback()
{
  m_callback = NULL;
}

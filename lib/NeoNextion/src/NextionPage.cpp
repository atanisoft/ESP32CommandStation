/*! \file */

#include "NextionPage.h"

/*!
 * \copydoc INextionWidget::INextionWidget
 */
NextionPage::NextionPage(Nextion &nex, uint8_t page, uint8_t component,
                         const String &name)
    : INextionWidget(nex, page, component, name)
{
}

/*!
 * \brief Sets this page as the currently displayed page.
 * \return True if successful
 */
bool NextionPage::show()
{
  if(!isShown())
  {
    return sendCommandWithWait("page %s", m_name.c_str());
  }
  return true;
}

/*!
 * \brief Determines if this page is currently displayed.
 * \return True if displayed
 */
bool NextionPage::isShown()
{
  return m_nextion.getCurrentPage() == m_pageID;
}

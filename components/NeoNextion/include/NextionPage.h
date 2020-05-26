/*! \file */

#ifndef __NEONEXTION_NEXTIONPAGE
#define __NEONEXTION_NEXTIONPAGE

#include "NeoNextion.h"
#include "INextionWidget.h"

/*!
 * \class NextionPage
 * \brief Represents a page of widgets.
 */
class NextionPage : public INextionWidget
{
public:
  NextionPage(Nextion &nex, uint8_t page, uint8_t component, const std::string &name);

  bool show();
  bool isShown();
};

#endif

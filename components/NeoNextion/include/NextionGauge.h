/*! \file */

#ifndef __NEONEXTION_NEXTIONGAUGE
#define __NEONEXTION_NEXTIONGAUGE

#include "NeoNextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"
#include "INextionNumericalValued.h"

/*!
 * \class NextionGauge
 * \brief Represents a gauge widget/
 */
class NextionGauge : public INextionTouchable,
                     public INextionColourable,
                     public INextionNumericalValued
{
public:
  /*!
   * \copydoc INextionWidget::INextionWidget
   */
  NextionGauge(Nextion &nex, uint8_t page, uint8_t component, const std::string &name)
      : INextionWidget(nex, page, component, name)
      , INextionTouchable(nex, page, component, name)
      , INextionColourable(nex, page, component, name)
      , INextionNumericalValued(nex, page, component, name)
  {
  }
};

#endif

/*! \file */

#ifndef __NEONEXTION_NEXTIONTIMER
#define __NEONEXTION_NEXTIONTIMER

#include "NeoNextion.h"
#include "INextionTouchable.h"

/*!
 * \class NextionTimer
 * \brief Represents a timer.
 */
class NextionTimer : public INextionTouchable
{
public:
  NextionTimer(Nextion &nex, uint8_t page, uint8_t component, const std::string &name);

  uint32_t getCycle();
  bool setCycle(uint32_t cycle);

  bool enable();
  bool disable();
};

#endif

/*! \file */

#ifndef __NEONEXTION_NEXTIONWAVEFORM
#define __NEONEXTION_NEXTIONWAVEFORM

#include "Nextion.h"
#include "INextionTouchable.h"
#include "INextionColourable.h"

/*!
 * \class NextionWaveform
 * \brief Represents a waveform widget.
 */
class NextionWaveform : public INextionTouchable, public INextionColourable
{
public:
  NextionWaveform(Nextion &nex, uint8_t page, uint8_t component,
                  const String &name);

  bool addValue(uint8_t channel, uint8_t value);

  bool setChannelColour(uint8_t channel, uint32_t colour, bool refresh = true);
  uint32_t getChannelColour(uint8_t channel);

  bool setGridColour(uint32_t colour, bool refresh = true);
  uint32_t getGridColour();

  bool setGridWidth(uint16_t width);
  uint16_t getGridWidth();

  bool setGridHeight(uint16_t height);
  uint16_t getGridHeight();
};

#endif

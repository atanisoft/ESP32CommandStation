/*! \file */

#ifndef __NEONEXTION_NEXTIONCROP
#define __NEONEXTION_NEXTIONCROP

#include "NeoNextion.h"
#include "INextionTouchable.h"

/*!
 * \class NextionCrop
 * \brief Represents a cropped picture widget.
 */
class NextionCrop : public INextionTouchable
{
public:
  NextionCrop(Nextion &nex, uint8_t page, uint8_t component, const std::string &name);

  uint16_t getPictureID();
  bool setPictureID(uint16_t id);
};

#endif

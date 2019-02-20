/*! \file */

#ifndef __NEONEXTION_NEXTION
#define __NEONEXTION_NEXTION

#if defined(SPARK) || defined(PLATFORM_ID)
#include "application.h"
extern char *itoa(int a, char *buffer, unsigned char radix);
#else
#include <Arduino.h>
#endif

#include <WString.h>

#include "NextionTypes.h"

class INextionTouchable;

/*!
 * \struct ITouchableListItem
 * \brief Linked list node for INextionTouchable objects.
 */
struct ITouchableListItem
{
  INextionTouchable *item;  //!< Pointer to stored INextionTouchable
  ITouchableListItem *next; //!< Pointer to next list node
};

/*!
 * \class Nextion
 * \brief Driver for a physical Nextion device.
 */
class Nextion
{
public:
  Nextion(Stream &stream, bool flushSerialBeforeTx = true);

  bool init();
  void poll();

  bool refresh();
  bool refresh(const String &objectName);

  bool sleep();
  bool wake();

  uint16_t getBrightness();
  bool setBrightness(uint16_t val, bool persist = false);

  uint8_t getCurrentPage();

  bool clear(uint32_t colour = NEX_COL_WHITE);
  bool drawPicture(uint16_t x, uint16_t y, uint8_t id);
  bool drawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t id);
  bool drawStr(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t fontID,
               const String &str, uint32_t bgColour = NEX_COL_BLACK,
               uint32_t fgColour = NEX_COL_WHITE,
               uint8_t bgType = NEX_BG_SOLIDCOLOUR,
               NextionFontAlignment xCentre = NEX_FA_CENTRE,
               NextionFontAlignment yCentre = NEX_FA_CENTRE);
  bool drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                uint32_t colour);
  bool drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool filled,
                uint32_t colour);
  bool drawCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t colour);

  void registerTouchable(INextionTouchable *touchable);
  void sendCommand(const String &command);
  void sendCommand(const char *format, ...);
  void sendCommand(const char *format, va_list args);
  bool checkCommandComplete();
  bool receiveNumber(uint32_t *number);
  size_t receiveString(String &buffer, bool stringHeader=true);

private:
  Stream &m_serialPort;       //!< Serial port device is attached to
  uint32_t m_timeout;         //!< Serial communication timeout in ms
  bool m_flushSerialBeforeTx; //!< Flush serial port before transmission
  ITouchableListItem *m_touchableList; //!< LInked list of INextionTouchable
};

#endif

/*! \file */

#include "NeoNextion.h"
#include "INextionTouchable.h"

/*!
 * \brief Creates a new device driver.
 * \param stream Stream (serial port) the device is connected to
 * \param flushSerialBeforeTx If the serial port should be flushed before
 *                            transmission
 */
Nextion::Nextion(uint8_t uartNum, long baud, uint8_t rx_pin, uint8_t tx_pin, bool flushSerialBeforeTx)
    : m_timeout(500)
    , m_flushSerialBeforeTx(flushSerialBeforeTx)
    , m_touchableList(NULL)
{
  m_serialPort = (uart_port_t)(UART_NUM_0 + uartNum);
  uart_config_t uart_config =
  {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .use_ref_tick = false
  };
  uart_param_config(m_serialPort, &uart_config);
  uart_set_pin(m_serialPort, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(m_serialPort, 2*1024, 0, 0, NULL, 0);
}

/*!
 * \brief Initialises the device.
 * \return True if initialisation was successful.
 */
bool Nextion::init()
{
  sendCommand("");

  sendCommand("bkcmd=1");
  bool result1 = checkCommandComplete();

  sendCommand("page 0");
  bool result2 = checkCommandComplete();

  return (result1 && result2);
}

/*!
 * \brief Polls for new messages and touch events.
 */
void Nextion::poll()
{
  size_t ready{0};
  uart_get_buffered_data_len(m_serialPort, &ready);
  while (ready >= 7)
  {
    uint8_t buffer[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (uart_read_bytes(m_serialPort, buffer, 7, 0) == 7)
    {
      if (buffer[0] == NEX_RET_EVENT_TOUCH_HEAD &&
          buffer[4] == 0xFF &&
          buffer[5] == 0xFF &&
          buffer[6] == 0xFF)
      {
        ITouchableListItem *item = m_touchableList;
        while (item != NULL &&
              !item->item->processEvent(buffer[1], buffer[2], buffer[3]))
        {
          item = item->next;
        }
      }
      else
      {
        printf("Nextion: %02x %02x %02x %02x %02x %02x %02x\n", buffer[0]
              , buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
      }
    }
    else
    {
      printf("Nextion: short read\n");
    }
  }
}

/*!
 * \brief Refreshes the entire page.
 * \return True if successful
 */
bool Nextion::refresh()
{
  sendCommand("ref 0");
  return checkCommandComplete();
}

/*!
 * \brief Refreshes a specific object.
 * \param objectName Name of the object to refresh
 * \return True if successful
 */
bool Nextion::refresh(const std::string &objectName)
{
  sendCommand("ref %s", objectName.c_str());
  return checkCommandComplete();
}

/*!
 * \brief Puts the device into sleep mode.
 * \return True if successful
 */
bool Nextion::sleep()
{
  sendCommand("sleep=1");
  return checkCommandComplete();
}

/*!
 * \brief Wakes the device from sleep mode.
 * \return True if successful
 */
bool Nextion::wake()
{
  sendCommand("sleep=0");
  return checkCommandComplete();
}

/*!
 * \brief Gets the current backlight brightness.
 * \return Brightness
 */
uint16_t Nextion::getBrightness()
{
  sendCommand("get dim");
  uint32_t val;
  if (receiveNumber(&val))
    return val;
  else
    return 0;
}

/*!
 * \brief Sets the backlight brightness.
 * \param val Brightness value (0-100)
 * \param persist If set to true value will be set as new power on default
 * \return True if successful
 */
bool Nextion::setBrightness(uint16_t val, bool persist)
{
  if (persist)
  {
    sendCommand("dims=%d", val);
  }
  else
  {
    sendCommand("dim=%d", val);
  }
  return checkCommandComplete();
}

/*!
 * \brief Gets the ID of the current displayed page.
 * \return Page ID
 */
uint8_t Nextion::getCurrentPage()
{
  sendCommand("sendme");

  uint8_t temp[5] = {0};

  if (sizeof(temp) != uart_read_bytes(m_serialPort, temp, sizeof(temp), pdMS_TO_TICKS(10)))
  {
    return 0;
  }

  if (temp[0] == NEX_RET_CURRENT_PAGE_ID_HEAD && temp[2] == 0xFF &&
      temp[3] == 0xFF && temp[4] == 0xFF)
  {
    return temp[1];
  }

  return 0;
}

/*!
 * \brief Clears the current display.
 * \param colour Colour to set display to
 * \return True if successful
 */
bool Nextion::clear(uint32_t colour)
{
  sendCommand("cls %d", colour);
  return checkCommandComplete();
}

/*!
 * \brief Draws a pre uploaded picture on the display.
 * \param x X position
 * \param y Y position
 * \param id ID of the picture to display
 * \return True if successful
 */
bool Nextion::drawPicture(uint16_t x, uint16_t y, uint8_t id)
{
  sendCommand("pic %d,%d,%d", x, y, id);
  return checkCommandComplete();
}

/*!
 * \brief Draws a cropped pre uplaoded picture on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param id ID of the picture to display
 * \return True if successful
 */
bool Nextion::drawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          uint8_t id)
{
  sendCommand("picq %d,%d,%d,%d,%d", x, y, w, h, id);
  return checkCommandComplete();
}

/*!
 * \brief Draws a string on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param fontID ID of the font to use
 * \param str String to draw
 * \param bgColour Colour of the background of the bounding box
 * \param fgColour Colour of the text
 * \param bgType Background type
 * \param xCentre X alignment
 * \param yCentre Y alignment
 * \return True if successful
 */
bool Nextion::drawStr(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint8_t fontID, const std::string &str, uint32_t bgColour,
                      uint32_t fgColour, uint8_t bgType,
                      NextionFontAlignment xCentre,
                      NextionFontAlignment yCentre)
{
  sendCommand("xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s", x, y, w, h, fontID, fgColour, bgColour, xCentre, yCentre, bgType, str.c_str());
  return checkCommandComplete();
}

/*!
 * \brief Draws a line on the display.
 * \param x1 X position of first vertex
 * \param y1 Y position of first vertex
 * \param x2 X position of second vertex
 * \param y2 Y position of second vertex
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint32_t colour)
{
  sendCommand("line %d,%d,%d,%d,%d", x1, y1, x2, y2, colour);
  return checkCommandComplete();
}

/*!
 * \brief Draws a rectangle on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param filled If the rectangle should be filled with a solid colour
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       bool filled, uint32_t colour)
{
  if (filled)
  {
    sendCommand("draw %d,%d,%d,%d,%d", x, y, x + w, y + h, colour);
  }
  else
  {
    sendCommand("fill %d,%d,%d,%d,%d", x, y, x + w, y + h, colour);
  }
  return checkCommandComplete();
}

/*!
 * \brief Draws a circle on the display.
 * \param x X position
 * \param y Y position
 * \param r Radius
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t colour)
{
  sendCommand("cir %d,%d,%d,%d", x, y, r, colour);
  return checkCommandComplete();
}

/*!
 * \brief Adds a INextionTouchable to the list of registered touchable
 *        elements.
 * \param touchable Pointer to the INextionTouchable
 *
 * Required for touch events from an INextionTouchable to be polled.
 *
 * Should be called automatically by INextionTouchable::INextionTouchable.
 */
void Nextion::registerTouchable(INextionTouchable *touchable)
{
  ITouchableListItem *newListItem = new ITouchableListItem;
  newListItem->item = touchable;
  newListItem->next = NULL;

  if (m_touchableList == NULL)
    m_touchableList = newListItem;
  else
  {
    ITouchableListItem *item = m_touchableList;
    while (item->next != NULL)
      item = item->next;
    item->next = newListItem;
  }
}

/*!
 * \brief Sends a command to the device.
 * \param command Command to send
 */
void Nextion::sendCommand(const std::string &command)
{
  char end_bytes[3] = {0xFF, 0xFF, 0xFF};
  if (m_flushSerialBeforeTx)
  {
    uart_flush_input(m_serialPort);
  }

#if NEXTION_DEBUG
  printf("Nextion: TX: %s\n", command.c_str());
#endif
  uart_write_bytes(m_serialPort, command.c_str(), command.length());
  uart_write_bytes(m_serialPort, end_bytes, 3);
}

void Nextion::sendCommand(const char *format, ...) {
  va_list args;
  va_start(args, format);
  sendCommand(format, args);
  va_end(args);
}

void Nextion::sendCommand(const char *format, va_list args) {
  char buf[512] = {0};
  vsnprintf(buf, sizeof(buf), format, args);
  sendCommand(buf);
}

/*!
 * \brief Checks if the last command was successful.
 * \return True if command was successful
 */
bool Nextion::checkCommandComplete()
{
  bool ret = false;
  uint8_t temp[4] = {0};
  size_t bytesRead = uart_read_bytes(m_serialPort, temp, sizeof(temp), pdMS_TO_TICKS(10));

  if (bytesRead != sizeof(temp))
  {
#if NEXTION_DEBUG
    printf("Nextion: checkCommandComplete short read: %d\n", bytesRead);
    printf("Nextion: %02x %02x %02x %02x, ret: %d\n", temp[0], temp[1], temp[2], temp[3], ret);
#endif // NEXTION_DEBUG
  }
  else if (temp[0] == NEX_RET_CMD_FINISHED && temp[1] == 0xFF && temp[2] == 0xFF && temp[3] == 0xFF)
  {
    ret = true;
  }

  return ret;
}

/*!
 * \brief Receive a number from the device.
 * \param number Pointer to the number to store received number in
 * \return True if receive was successful
 */
bool Nextion::receiveNumber(uint32_t *number)
{
  uint8_t temp[8] = {0};

  if (!number)
    return false;
  size_t bytesRead = uart_read_bytes(m_serialPort, temp, sizeof(temp), pdMS_TO_TICKS(10));

  if (bytesRead != sizeof(temp))
  {
    return false;
  }

  if (temp[0] == NEX_RET_NUMBER_HEAD && temp[5] == 0xFF && temp[6] == 0xFF &&
      temp[7] == 0xFF)
  {
    *number = (temp[4] << 24) | (temp[3] << 16) | (temp[2] << 8) | (temp[1]);
    return true;
  }

  return false;
}

/*!
 * \brief Receive a string from the device.
 * \param buffer Pointer to buffer to store string in
 * \return Actual length of string received
 */
size_t Nextion::receiveString(std::string &buffer, bool stringHeader) {
  bool have_header_flag = !stringHeader;
  uint8_t flag_count = 0;
  uint32_t start = esp_timer_get_time() / 1000ULL;
  buffer.reserve(128);
  while (esp_timer_get_time() - start <= m_timeout)
  {
    size_t ready = 0;
    uart_get_buffered_data_len(m_serialPort, &ready);
    while (ready > 0)
    {
      uint8_t ch;
      uart_read_bytes(m_serialPort, &ch, 1, pdMS_TO_TICKS(10));
      if (!have_header_flag && ch == NEX_RET_STRING_HEAD)
      {
        have_header_flag = true;
      }
      else if (have_header_flag)
      {
        if (ch == NEX_RET_CMD_FINISHED)
        {
          // it appears that we received a "previous command completed successfully"
          // response. Discard the next three bytes which will be 0xFF so we can
          // advance toh the actual response we are wanting.
          uint8_t temp[3];
          uart_read_bytes(m_serialPort, temp, sizeof(temp), pdMS_TO_TICKS(10));
        }
        else if (ch == 0xFF)
        {
          flag_count++;
        }
        else if (ch == 0x05 && !stringHeader)
        {
          // this is a special case for the "connect" command
          flag_count = 3;
        }
        else if (ch < 0x20 || ch > 0x7F)
        {
          // discard non-printable character
        }
        else
        {
          buffer += (char)ch;
        }
      }
      uart_get_buffered_data_len(m_serialPort, &ready);
    }

    if (flag_count >= 3)
    {
      break;
    }
  }
  return buffer.length();
}

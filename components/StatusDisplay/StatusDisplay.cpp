/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#include "sdkconfig.h"
#include "StatusDisplay.h"

#include <AllTrainNodes.hxx>
#include <driver/i2c.h>
#include <esp_ota_ops.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <LCCWiFiManager.h>

using commandstation::AllTrainNodes;

static constexpr uint8_t STATUS_DISPLAY_LINE_COUNT = 5;

static constexpr TickType_t DISPLAY_I2C_TIMEOUT =
  pdMS_TO_TICKS(CONFIG_DISPLAY_I2C_TIMEOUT_MSEC);

#if CONFIG_DISPLAY_TYPE_OLED

#if CONFIG_DISPLAY_OLED_FONT_THIN
#include "ThinOLEDFont.h"
#elif CONFIG_DISPLAY_OLED_FONT_BOLD
#include "BoldOLEDFont.h"
#else
#error Unknown OLED Font type!
#endif

// Control command bytes for the OLED display
static constexpr uint8_t OLED_COMMAND_STREAM           = 0x00;
static constexpr uint8_t OLED_COMMAND_SINGLE           = 0x80;
static constexpr uint8_t OLED_DATA_STREAM              = 0x40;

// Basic display behavior commands
static constexpr uint8_t OLED_DISPLAY_OFF              = 0xAE;
static constexpr uint8_t OLED_DISPLAY_RAM              = 0xA4;
static constexpr uint8_t OLED_DISPLAY_NORMAL           = 0xA6;
static constexpr uint8_t OLED_DISPLAY_ON               = 0xAF;
static constexpr uint8_t OLED_SET_CONTRAST             = 0x81;

// OLED hardware configuration commands 
static constexpr uint8_t OLED_DISPLAY_START_LINE       = 0x40;
static constexpr uint8_t OLED_SET_SEGMENT_MAP_NORMAL   = 0xA0;
static constexpr uint8_t OLED_SET_SEGMENT_MAP_INVERTED = 0xA1;
static constexpr uint8_t OLED_DISPLAY_MUX              = 0xA8;
static constexpr uint8_t OLED_SET_SCAN_MODE_NORMAL     = 0xC0;
static constexpr uint8_t OLED_SET_SCAN_MODE_INVERTED   = 0xC8;
static constexpr uint8_t OLED_DISPLAY_OFFSET           = 0xD3;
static constexpr uint8_t OLED_COM_PIN_MAP              = 0xDA;

// OLED memory addressing control
static constexpr uint8_t OLED_MEMORY_MODE              = 0x20;
static constexpr uint8_t OLED_MEMORY_MODE_HORIZONTAL   = 0x00;
static constexpr uint8_t OLED_MEMORY_MODE_VERTICAL     = 0x01;
static constexpr uint8_t OLED_MEMORY_MODE_PAGE         = 0x02;
static constexpr uint8_t OLED_MEMORY_COLUMN_RANGE      = 0x21;
static constexpr uint8_t OLED_MEMORY_PAGE_RANGE        = 0x22;
static constexpr uint8_t OLED_SET_PAGE                 = 0xB0;

// OLED Timing/Driving control
static constexpr uint8_t OLED_CLOCK_DIVIDER            = 0xD5;
static constexpr uint8_t OLED_SET_CHARGEPUMP           = 0x8D;
static constexpr uint8_t OLED_CHARGEPUMP_ON            = 0x14;
static constexpr uint8_t OLED_SET_PRECHARGE            = 0xD9;
static constexpr uint8_t OLED_SET_VCOMH                = 0xDB;

// OLED scroll control
static constexpr uint8_t OLED_DISABLE_SCROLL           = 0x2E;

#elif CONFIG_DISPLAY_TYPE_LCD

// LCD control commands
static constexpr uint8_t LCD_DISPLAY_SHIFT             = 0x10;
static constexpr uint8_t LCD_FUNCTION_SET              = 0x20;
static constexpr uint8_t LCD_ADDRESS_SET               = 0x80;

// LCD shift flags
static constexpr uint8_t LCD_SHIFT_LEFT                = 0x08;
static constexpr uint8_t LCD_SHIFT_RIGHT               = 0x0C;

// LCD commands
static constexpr uint8_t LCD_CMD_CLEAR_SCREEN          = 0x01;
static constexpr uint8_t LCD_CMD_RETURN_HOME           = 0x02;
static constexpr uint8_t LCD_CMD_ENTRY_MODE            = 0x04;
static constexpr uint8_t LCD_CMD_DISPLAY_CONTROL       = 0x08;

static constexpr uint8_t LCD_ENTRY_AUTO_SCROLL         = 0x01;
static constexpr uint8_t LCD_ENTRY_LEFT_TO_RIGHT       = 0x02;

// display control flags
static constexpr uint8_t LCD_CURSOR_BLINK_ON           = 0x01;
static constexpr uint8_t LCD_CURSOR_ON                 = 0x02;
static constexpr uint8_t LCD_DISPLAY_ON                = 0x04;

static constexpr uint8_t LCD_8BIT_MODE                 = 0x10;
static constexpr uint8_t LCD_4BIT_MODE                 = 0x00;
static constexpr uint8_t LCD_TWO_LINE_MODE             = 0x08;
static constexpr uint8_t LCD_ONE_LINE_MODE             = 0x00;

static constexpr int LCD_LINE_OFFSETS[] = { 0x00, 0x40, 0x14, 0x54 };

static inline bool send_to_lcd(uint8_t addr, uint8_t value)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_NUM_0, cmd
                                                     , DISPLAY_I2C_TIMEOUT));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return false;
  }
  return true;
}

static inline bool send_lcd_nibble(uint8_t addr, uint8_t value, bool data)
{
  uint8_t nibble = (value << 4);
  
  // always send the data with the backlight flag enabled
  nibble |= CONFIG_DISPLAY_LCD_BACKLIGHT_BITMASK;

  // if this is a data byte set the register select flag
  if (data)
  {
    nibble |= CONFIG_DISPLAY_LCD_REGISTER_SELECT_BITMASK;
  }

  // send with ENABLE flag set
  if(!send_to_lcd(addr, nibble | CONFIG_DISPLAY_LCD_ENABLE_BITMASK))
  {
    return false;
  }

  ets_delay_us(1);
  // send without the ENABLE flag set
  if(!send_to_lcd(addr, nibble & ~CONFIG_DISPLAY_LCD_ENABLE_BITMASK))
  {
    return false;
  }
  ets_delay_us(37);
  return true;
}

static inline bool send_lcd_byte(uint8_t addr, uint8_t value, bool data)
{
  return send_lcd_nibble(addr, (value >> 4) & 0x0F, data) &&
         send_lcd_nibble(addr, value & 0x0F, data);
}

#endif

#define I2C_READ_REG(address, reg, data, data_size, status)               \
  {                                                                       \
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                         \
    i2c_master_start(cmd);                                                \
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);  \
    i2c_master_write_byte(cmd, reg, true);                                \
    i2c_master_start(cmd);                                                \
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);   \
    if (data_size > 1)                                                    \
    {                                                                     \
      i2c_master_read(cmd, &data, data_size - 1, I2C_MASTER_ACK);         \
    }                                                                     \
    i2c_master_read_byte(cmd, &data + data_size - 1, I2C_MASTER_NACK);    \
    i2c_master_stop(cmd);                                                 \
    status = i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT);   \
    i2c_cmd_link_delete(cmd);                                             \
  }

StatusDisplay::StatusDisplay(openlcb::SimpleStackBase *stack, Service *service)
  : StateFlowBase(service), stack_(stack)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  lccNodeBrowser_.emplace(stack->node()
                        , std::bind(&StatusDisplay::node_pong, this
                                  , std::placeholders::_1));
  clear();
  info("ESP32-CS: v%s", CONFIG_ESP32CS_SW_VERSION);
  wifi("IP:Pending");
  Singleton<Esp32WiFiManager>::instance()->register_network_up_callback(
  [&](esp_interface_t interface, uint32_t ip)
  {
    if (interface == ESP_IF_WIFI_STA)
    {
#if CONFIG_DISPLAY_COLUMN_COUNT > 16 || CONFIG_DISPLAY_TYPE_OLED
      wifi("IP: %s", ipv4_to_string(ip).c_str());
#else
      wifi(ipv4_to_string(ip).c_str());
#endif
    }
    else if (interface == ESP_IF_WIFI_AP)
    {
      wifi("SSID: %s"
         , Singleton<esp32cs::LCCWiFiManager>::instance()->get_ssid().c_str());
    }
  });
  Singleton<Esp32WiFiManager>::instance()->register_network_down_callback(
  [&](esp_interface_t interface)
  {
    wifi("Disconnected");
  });
  start_flow(STATE(init));
#endif
}

void StatusDisplay::clear()
{
  LOG(VERBOSE, "[StatusDisplay] clear screen");
  for(int line = 0; line < CONFIG_DISPLAY_LINE_COUNT; line++)
  {
    lines_[line] = "";
    lineChanged_[line] = true;
  }
}

void StatusDisplay::info(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[0] = buf;
  lineChanged_[0] = true;
#endif
}

void StatusDisplay::status(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[CONFIG_DISPLAY_LINE_COUNT - 1] = buf;
  lineChanged_[CONFIG_DISPLAY_LINE_COUNT - 1] = true;
#endif
}

void StatusDisplay::wifi(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
#if CONFIG_DISPLAY_LINE_COUNT > 2
  lines_[1] = buf;
  lineChanged_[1] = true;
#else
  lines_[0] = buf;
  lineChanged_[0] = true;
#endif
#endif
}

void StatusDisplay::track_power(const std::string &format, ...)
{
#if CONFIG_DISPLAY_LINE_COUNT > 2
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[2] = buf;
  lineChanged_[2] = true;
#endif
}

// NOTE: this code uses ets_printf() instead of LOG(VERBOSE, ...) due to the
// calling context.
void StatusDisplay::node_pong(openlcb::NodeID id)
{
  AtomicHolder h(this);
  // If this is the first response, reset the counts
  if (lccNodeRefreshPending_)
  {
    lccNodeRefreshPending_ = false;
    lccLocalNodeCount_ = 0;
    lccRemoteNodeCount_ = 0;
    lccSeenNodes_.clear();
  }
  else if (lccSeenNodes_.find(id) != lccSeenNodes_.end())
  {
    // duplicate node
    return;
  }
  lccSeenNodes_.emplace(id);
#if CONFIG_DISPLAY_LCC_LOGGING_VERBOSE
  ets_printf("[LCC-Node] PONG: %s,", uint64_to_string_hex(id).c_str());
#endif
  // If it is a train node and we recognize it as one we manage then it is
  // considered a local node.
  if (Singleton<AllTrainNodes>::instance()->is_valid_train_node(id, false))
  {
#if CONFIG_DISPLAY_LCC_LOGGING_VERBOSE
    ets_printf("Train");
#endif
    lccLocalNodeCount_++;
  }
  else if (id == stack_->node()->node_id())
  {
#if CONFIG_DISPLAY_LCC_LOGGING_VERBOSE
    ets_printf("CS");
#endif
    // If the node id is our local node id it is considered local
    lccLocalNodeCount_++;
  }
  else
  {
#if CONFIG_DISPLAY_LCC_LOGGING_VERBOSE
    ets_printf("Remote");
#endif
    // Anything else is remote
    lccRemoteNodeCount_++;
  }
#if CONFIG_DISPLAY_LCC_LOGGING_VERBOSE
  ets_printf("\n");
#endif
}

StateFlowBase::Action StatusDisplay::init()
{
#if !CONFIG_DISPLAY_TYPE_NONE
  LOG(INFO, "[StatusDisplay] Initializing I2C driver...");
  i2c_config_t i2c_config =
  {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = (gpio_num_t)CONFIG_DISPLAY_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = (gpio_num_t)CONFIG_DISPLAY_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master =
    {
      .clk_speed = CONFIG_DISPLAY_I2C_BUS_SPEED
    }
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

  // scan a handful of known I2C address ranges for LCD or OLED devices
  esp_err_t ret;
#if CONFIG_DISPLAY_TYPE_OLED
  for(uint8_t addr = 0x3C; addr <= 0x3C; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for OLED on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initOLED));
    }
  }
#elif CONFIG_DISPLAY_TYPE_LCD
  // PCF8574 or MCP23008
  for(uint8_t addr = 0x20; addr <= 0x27; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for LCD on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initLCD));
    }
  }
  // PCF8574A
  for(uint8_t addr = 0x38; addr <= 0x3F; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for LCD on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initLCD));
    }
  }
#endif
  LOG(WARNING, "[StatusDisplay] no display detected");

  // Scan the I2C bus and dump the output of devices that respond
  std::string scanresults =
    "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n"
    "00:         ";
  scanresults.reserve(256);
  for (uint8_t addr=3; addr < 0x78; addr++)
  {
    if (addr % 16 == 0)
    {
      scanresults += "\n" + int64_to_string_hex(addr) + ":";
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
      scanresults += int64_to_string_hex(addr);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
      scanresults += " ??";
    }
    else
    {
      scanresults += " --";
    }
  }
  LOG(WARNING, "[StatusDisplay] A supported display was not detected, below "
      "are the detected I2C devices\n%s", scanresults.c_str());
#endif // !CONFIG_DISPLAY_TYPE_NONE
  return exit();
}

StateFlowBase::Action StatusDisplay::initOLED()
{
#if CONFIG_DISPLAY_TYPE_OLED
  LOG(INFO, "[StatusDisplay] OLED display detected on address 0x%02x"
    , i2cAddr_);
#if CONFIG_DISPLAY_OLED_RESET_PIN != -1
  static bool resetCalled = false;
  if(!resetCalled)
  {
    LOG(INFO, "[StatusDisplay] Resetting OLED display");
    gpio_pad_select_gpio((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN
                      , GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN, 0);
    resetCalled = true;
    return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(initOLED));
  }
  else
  {
    gpio_set_level((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN, 1);
  }
#endif // CONFIG_DISPLAY_OLED_RESET_PIN
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_COMMAND_STREAM, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_OFF, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_MUX, true);
  i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_WIDTH -1, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_OFFSET, true);
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_START_LINE, true);
  i2c_master_write_byte(cmd, OLED_SET_CHARGEPUMP, true);
  i2c_master_write_byte(cmd, OLED_CHARGEPUMP_ON, true);

  // Test the register zero data with power/state masked to identify the
  // connected chipset since SSD1306 and SH1106 require slightly different
  // initialization parameters.
  if (((regZero_ & 0x0F) == 0x03) || ((regZero_ & 0x0F) == 0x06))
  {
    LOG(INFO, "[StatusDisplay] OLED driver IC: SSD1306");
    i2c_master_write_byte(cmd, OLED_CLOCK_DIVIDER, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE_PAGE, true);
    i2c_master_write_byte(cmd, OLED_SET_PRECHARGE, true);
    i2c_master_write_byte(cmd, 0xF1, true);
    i2c_master_write_byte(cmd, OLED_SET_VCOMH, true);
    i2c_master_write_byte(cmd, 0x40, true);
  }
  else if (((regZero_ & 0x0F) == 0x08))
  {
    LOG(INFO, "[StatusDisplay] OLED driver IC: SH1106");
    sh1106_ = true;
    i2c_master_write_byte(cmd, OLED_CLOCK_DIVIDER, true);
    i2c_master_write_byte(cmd, 0xF0, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE_HORIZONTAL, true);
    i2c_master_write_byte(cmd, OLED_SET_PRECHARGE, true);
    i2c_master_write_byte(cmd, 0x22, true);
    i2c_master_write_byte(cmd, OLED_SET_VCOMH, true);
    i2c_master_write_byte(cmd, 0x30, true);
  }
  else
  {
    LOG(WARNING, "[StatusDisplay] Unrecognized OLED Register zero value: %x"
      , regZero_ & 0x0F);
    // cleanup and abort init process
    i2c_cmd_link_delete(cmd);
    return exit();
  }
  LOG(INFO, "[StatusDisplay] OLED display size: %dx%d (%d lines)"
    , CONFIG_DISPLAY_OLED_WIDTH, CONFIG_DISPLAY_OLED_HEIGHT
    , CONFIG_DISPLAY_LINE_COUNT);

#if CONFIG_DISPLAY_OLED_VFLIP
  i2c_master_write_byte(cmd, OLED_SET_SEGMENT_MAP_INVERTED, true);
  i2c_master_write_byte(cmd, OLED_SET_SCAN_MODE_INVERTED, true);
#else
  i2c_master_write_byte(cmd, OLED_SET_SEGMENT_MAP_NORMAL, true);
  i2c_master_write_byte(cmd, OLED_SET_SCAN_MODE_NORMAL, true);
#endif

  i2c_master_write_byte(cmd, OLED_COM_PIN_MAP, true);
#if CONFIG_DISPLAY_OLED_128x64
  i2c_master_write_byte(cmd, 0x12, true);
#elif CONFIG_DISPLAY_OLED_128x32 || CONFIG_DISPLAY_OLED_96x16
  i2c_master_write_byte(cmd, 0x02, true);
#endif

  i2c_master_write_byte(cmd, OLED_SET_CONTRAST, true);
  i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_CONTRAST, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_RAM, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_NORMAL, true);
  i2c_master_write_byte(cmd, OLED_DISABLE_SCROLL, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_ON, true);
	i2c_master_stop(cmd);

  LOG(INFO, "[StatusDisplay] Initializing OLED display");
  esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_NUM_0, cmd
                                                     , DISPLAY_I2C_TIMEOUT));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    LOG_ERROR("[StatusDisplay] Failed to initialize the OLED display");
    return exit();
  }
  LOG(INFO, "[StatusDisplay] OLED successfully initialized");

  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_OLED
}

StateFlowBase::Action StatusDisplay::initLCD()
{
#if CONFIG_DISPLAY_TYPE_LCD
  LOG(INFO,
      "[StatusDisplay] Detected LCD on address 0x%02x, initializing %dx%x "
      "display..."
    , i2cAddr_, CONFIG_DISPLAY_COLUMN_COUNT, CONFIG_DISPLAY_LINE_COUNT);

  // wake up the LCD and init to known state
  send_to_lcd(i2cAddr_, 0x00);
  vTaskDelay(pdMS_TO_TICKS(10));

  // send init data to reset to 8 bit mode
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(4537);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(237);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(237);

  // switch to four bit mode
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, LCD_FUNCTION_SET);
  ets_delay_us(37);

  // send rest of init commands as 4 bit mode
  send_lcd_byte(i2cAddr_, LCD_FUNCTION_SET | LCD_TWO_LINE_MODE, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_CLEAR_SCREEN, false);
  ets_delay_us(1600); // clear takes 1.5ms
  send_lcd_byte(i2cAddr_, LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT_TO_RIGHT, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_RETURN_HOME, false);
  ets_delay_us(1600); // home takes 1.5ms
  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_LCD
}

StateFlowBase::Action StatusDisplay::update()
{
#if CONFIG_LOCONET
  static uint8_t rotatingLineCount = 7;
#else
  static uint8_t rotatingLineCount = 5;
#endif
  // switch to next status line detail set after 10 iterations
  if(++updateCount_ > 10)
  {
    updateCount_ = 0;
    ++rotatingIndex_ %= rotatingLineCount;
  }
  // update the status line details every other iteration
  if(updateCount_ % 2)
  {
    if(rotatingIndex_ == 0)
    {
      status("Free Heap:%d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
    else if (rotatingIndex_ == 1)
    {
      uint64_t seconds = USEC_TO_SEC(esp_timer_get_time());
      status("Uptime: %02d:%02d:%02d"
           , (uint32_t)(seconds / 3600), (uint32_t)(seconds % 3600) / 60
           , (uint32_t)(seconds % 60)
      );
    }
    else if (rotatingIndex_ == 2)
    {
      status("Active Locos:%3d"
           , Singleton<commandstation::AllTrainNodes>::instance()->size()
      );
    }
    else if (rotatingIndex_ == 3)
    {
      AtomicHolder h(this);
      static uint8_t _lccStatusIndex = 0;
      ++_lccStatusIndex %= 5;
      if(_lccStatusIndex == 0)
      {
        status("LCC Rmt Node:%02d", lccRemoteNodeCount_);
      }
      else if (_lccStatusIndex == 1)
      {
        status("LCC Lcl Node:%02d", lccLocalNodeCount_);
      }
      else if (_lccStatusIndex == 2)
      {
        status("LCC Dg: %d"
             , stack_->dg_service()->client_allocator()->pending());
      }
      else if (_lccStatusIndex == 3)
      {
        uint32_t currentCount = stack_->service()->executor()->sequence();
        status("LCC Ex: %d", currentCount - lccLastExecCount_);
        lccLastExecCount_ = currentCount;
      }
      else if (_lccStatusIndex == 4)
      {
        status("LCC Pool: %.2fkB", mainBufferPool->total_size() / 1024.0f);

        // if enough time has passed since our last refresh trigger a new one.
        if (esp_timer_get_time() >= nextLccNodeCountRefreshTime_)
        {
          nextLccNodeCountRefreshTime_ =
            esp_timer_get_time() + LCC_NODE_REFRESH_INTERVAL;
          lccNodeRefreshPending_ = true;
          lccNodeBrowser_->refresh();
        }
      }
#if CONFIG_LOCONET
    }
    else if (rotatingIndex_ == 5)
    {
      status("LN-RX: %d/%d", locoNet.getRxStats()->rxPackets
           , locoNet.getRxStats()->rxErrors
      );
    }
    else if (rotatingIndex_ == 6)
    {
      status("LN-TX: %d/%d/%d", locoNet.getTxStats()->txPackets
           , locoNet.getTxStats()->txErrors
           , locoNet.getTxStats()->collisions
      );
#endif
    }
  }

  for (uint8_t line = 0; line < CONFIG_DISPLAY_LINE_COUNT; line++)
  {
    // if the line has not changed skip it
    if (!lineChanged_[line])
    {
      continue;
    }

#if CONFIG_DISPLAY_TYPE_OLED
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_COMMAND_STREAM, true);
    i2c_master_write_byte(cmd, OLED_SET_PAGE | line, true);
    if (sh1106_)
    {
      i2c_master_write_byte(cmd, 0x02, true);
      i2c_master_write_byte(cmd, 0x10, true);
    }
    else
    {
      i2c_master_write_byte(cmd, OLED_MEMORY_COLUMN_RANGE, true);
      i2c_master_write_byte(cmd, 0x00, true);
      i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_WIDTH - 1, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_DATA_STREAM, true);
    uint8_t col = 0;
    for (auto ch : lines_[line])
    {
      // Check that the character is a renderable character.
      if (ch <= 0x7f)
      {
        i2c_master_write(cmd, (uint8_t *)oled_font[(uint8_t)ch]
                        , OLED_FONT_WIDTH, true);
      }
      else
      {
        // since it is not a renderable character, send the 50% shaded block to
        // the display instead so the rendering stays consistent
        i2c_master_write(cmd, (uint8_t *)oled_font[1], OLED_FONT_WIDTH, true);
      }
      col++;
      // make sure we haven't rendered past the end of the display
      if (col >= CONFIG_DISPLAY_COLUMN_COUNT)
      {
        break;
      }
    }
    while(col++ < CONFIG_DISPLAY_COLUMN_COUNT)
    {
      i2c_master_write(cmd, (uint8_t *)oled_font[0], OLED_FONT_WIDTH, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
#elif CONFIG_DISPLAY_TYPE_LCD
    send_lcd_byte(i2cAddr_, LCD_ADDRESS_SET | LCD_LINE_OFFSETS[line], false);
    uint8_t col = 0;
    for (auto ch : lines_[line])
    {
      send_lcd_byte(i2cAddr_, ch, true);
      col++;
      if (col >= CONFIG_DISPLAY_COLUMN_COUNT)
      {
        break;
      }
    }
    // space pad to the width of the LCD
    while(col++ < CONFIG_DISPLAY_COLUMN_COUNT)
    {
      send_lcd_byte(i2cAddr_, ' ', true);
    }
#endif
    lineChanged_[line] = false;
  }
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}
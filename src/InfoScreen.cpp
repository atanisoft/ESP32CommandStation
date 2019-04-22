/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2017-2019 Mike Dunston

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

#include "DCCppESP32.h"

#ifndef INFO_SCREEN_SDA_PIN
#define INFO_SCREEN_SDA_PIN SDA
#endif
#ifndef INFO_SCREEN_SCL_PIN
#define INFO_SCREEN_SCL_PIN SCL
#endif
#ifndef INFO_SCREEN_OLED
#define INFO_SCREEN_OLED false
#endif

#ifndef INFO_SCREEN_LCD
#define INFO_SCREEN_LCD false
#endif

#include <Wire.h>
#if INFO_SCREEN_OLED
#include "InfoScreen_OLED_font.h"
#define INFO_SCREEN_I2C_TEST_ADDRESS INFO_SCREEN_OLED_I2C_ADDRESS
#if OLED_CHIPSET == SH1106
#include <SH1106Wire.h>
SH1106Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#elif OLED_CHIPSET == SSD1306
#include <SSD1306Wire.h>
SSD1306Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#endif
#elif INFO_SCREEN_LCD
#define INFO_SCREEN_I2C_TEST_ADDRESS INFO_SCREEN_LCD_I2C_ADDRESS
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcdDisplay(INFO_SCREEN_LCD_I2C_ADDRESS);
#endif

bool InfoScreen::_enabled;

#if INFO_SCREEN_OLED
String infoScreenLines[INFO_SCREEN_OLED_LINES];
void redrawOLED() {
  oledDisplay.clear();
  for(int line = 0; line < INFO_SCREEN_OLED_LINES; line++) {
    oledDisplay.drawString(0, line * Monospaced_plain_10[1], infoScreenLines[line]);
  }
  oledDisplay.display();
}
#endif

void InfoScreen::init() {
  _enabled = false;
#if INFO_SCREEN_ENABLED
  bool scanI2C = false;
  Wire.begin(INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);

  // if we have a reset pin defined, attempt to reset the I2C screen
#if defined(INFO_SCREEN_RESET_PIN)
  pinMode(INFO_SCREEN_RESET_PIN, OUTPUT);
  digitalWrite(INFO_SCREEN_RESET_PIN, LOW);
  delay(50);
  digitalWrite(INFO_SCREEN_RESET_PIN, HIGH);
#endif

  // Check that we can find the screen by its address before attempting to
  // use/configure it.
  Wire.beginTransmission(INFO_SCREEN_I2C_TEST_ADDRESS);
  if(Wire.endTransmission() == 0) {
    _enabled = true;
  } else {
    LOG(WARNING, "OLED/LCD screen not found at 0x%x\n", INFO_SCREEN_I2C_TEST_ADDRESS);
    scanI2C = true;
  }

#if INFO_SCREEN_OLED
  for(int i = 0; i < INFO_SCREEN_OLED_LINES; i++) {
    infoScreenLines[i] = "";
  }
  if(_enabled) {
    oledDisplay.init();
    oledDisplay.setContrast(255);
  	if(INFO_SCREEN_OLED_VERTICAL_FLIP == true) {
  		oledDisplay.flipScreenVertically();
  	}

    // NOTE: If the InfoScreen_OLED_font.h file is modified with a new font
    // definition, the name of the font needs to be declared on the next line.
  	oledDisplay.setFont(Monospaced_plain_10);
  }
#elif INFO_SCREEN_LCD
  // Check that we can find the LCD by its address before attempting to use it.
  if(_enabled) {
    lcdDisplay.begin(INFO_SCREEN_LCD_COLUMNS, INFO_SCREEN_LCD_LINES);
    lcdDisplay.setBacklight(255);
    lcdDisplay.clear();
    _enabled = true;
  }
#endif
  if(!_enabled) {
    LOG(WARNING, "Unable to initialize InfoScreen");
    if(scanI2C) {
      LOG(INFO, "Scanning for I2C devices...\n");
      LOG(INFO, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
      LOG(INFO, "00:         ");
      for (uint8_t addr=3; addr < 0x78; addr++) {
        if (addr % 16 == 0) {
          printf("\n%.2x:", addr);
        }
        Wire.beginTransmission(addr);
        if(Wire.endTransmission() == 0) {
          printf(" %.2x", addr);
        } else {
          printf(" --");
        }
      }
    }
  }
#endif // INFO_SCREEN_ENABLED
}

void InfoScreen::clear() {
  if(_enabled) {
#if INFO_SCREEN_OLED
    oledDisplay.clear();
#elif INFO_SCREEN_LCD
    lcdDisplay.clear();
#endif
  }
}

void InfoScreen::print(int col, int row, const __FlashStringHelper *format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf_P(buf, sizeof(buf), (const char *)format, args);
  va_end(args);
  if(_enabled) {
#if INFO_SCREEN_OLED
    infoScreenLines[row] = infoScreenLines[row].substring(0, col) + buf + infoScreenLines[row].substring(col + strlen(buf));
    redrawOLED();
#elif INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(col, row);
      lcdDisplay.print(buf);
    }
#endif
  }
}

void InfoScreen::print(int col, int row, const String &format, ...) {
  if(_enabled) {
    char buf[512] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format.c_str(), args);
    va_end(args);
#if INFO_SCREEN_OLED
    infoScreenLines[row] = infoScreenLines[row].substring(0, col) + buf + infoScreenLines[row].substring(col + strlen(buf));
    redrawOLED();
#elif INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(col, row);
      lcdDisplay.print(buf);
    }
#endif
  }
}

void InfoScreen::replaceLine(int row, const __FlashStringHelper *format, ...) {
  if(_enabled) {
    char buf[512] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf_P(buf, sizeof(buf), (const char *)format, args);
    va_end(args);
#if INFO_SCREEN_OLED
    infoScreenLines[row] = buf;
    redrawOLED();
#elif INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(0, row);
      lcdDisplay.print(buf);
      if(strlen(buf) < INFO_SCREEN_LCD_COLUMNS) {
        for(int space = strlen(buf); space < INFO_SCREEN_LCD_COLUMNS; space++) {
          lcdDisplay.print(" ");
        }
      }
    }
#endif
  }
}

void InfoScreen::replaceLine(int row, const String &format, ...) {
  if(_enabled) {
    char buf[512] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format.c_str(), args);
    va_end(args);
#if INFO_SCREEN_OLED
    infoScreenLines[row] = buf;
    redrawOLED();
#elif INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(0, row);
      lcdDisplay.print(buf);
      if(strlen(buf) < INFO_SCREEN_LCD_COLUMNS) {
        for(int space = strlen(buf); space < INFO_SCREEN_LCD_COLUMNS; space++) {
          lcdDisplay.print(" ");
        }
      }
    }
#endif
  }
}

#if LCC_ENABLED
extern OpenMRN openmrn;
#endif

void InfoScreen::update() {
  static uint8_t _rotatingStatusIndex = 0;
  static uint8_t _rotatingStatusLineCount = 3;
  static uint8_t _motorboardIndex = 0;
  static uint32_t _lastRotation = millis();
  static uint32_t _lastUpdate = millis();
#if LOCONET_ENABLED
  static uint8_t _firstLocoNetIndex = 0;
  if(!_firstLocoNetIndex) {
    _firstLocoNetIndex = _rotatingStatusLineCount;
    _rotatingStatusLineCount += 2;
  }
#endif
#if LCC_ENABLED
  static uint8_t _firstLCCIndex = 0;
  if(!_firstLCCIndex) {
    _firstLCCIndex = _rotatingStatusLineCount;
    _rotatingStatusLineCount++;
  }
#endif
  // switch to next status line detail set every five seconds
  if(millis() - _lastRotation >= 5000) {
    _lastRotation = millis();
    ++_rotatingStatusIndex %= _rotatingStatusLineCount;
  }
  // update the status line details every second
  if(millis() - _lastUpdate >= 950) {
    _lastUpdate = millis();
    if(_enabled) {
      if(_rotatingStatusIndex == 0) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Free Heap:%d"),
          ESP.getFreeHeap());
      } else if (_rotatingStatusIndex == 1) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Active Locos:%3d"),
          LocomotiveManager::getActiveLocoCount());
      } else if (_rotatingStatusIndex == 2) {
        ++_motorboardIndex %= MotorBoardManager::getMotorBoardCount();
        auto board = MotorBoardManager::getBoardByName(MotorBoardManager::getBoardNames()[_motorboardIndex]);
        if(board != nullptr && (board->isOn() || board->isOverCurrent())) {
          if(board->isOverCurrent()) {
#if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
            replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:%2.2fA"),
              board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
#else
            replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:Fault (%2.2f A)"),
              board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
#endif
          } else if(board->isOn()) {
#if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
            replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:%2.2fA"),
              board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
#else
            replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:Normal (%2.2f A)"),
              board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
#endif
          }
        } else if(board != nullptr) {
          replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:Off"),
            board->getName().c_str());
        }
#if LOCONET_ENABLED
      } else if (_rotatingStatusIndex == _firstLocoNetIndex) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("LN-RX: %d/%d"),
          locoNet.getRxStats()->rxPackets, locoNet.getRxStats()->rxErrors);
      } else if (_rotatingStatusIndex == _firstLocoNetIndex + 1) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("LN-TX: %d/%d/%d"),
          locoNet.getTxStats()->txPackets, locoNet.getTxStats()->txErrors, locoNet.getTxStats()->collisions);
#endif
#if LCC_ENABLED
      } else if (_rotatingStatusIndex == _firstLCCIndex) {
        // placeholder until LCC stats can be exported and displayed
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("LCC: %d"),
          openmrn.stack()->can_hub()->size());
#endif
      }
    }
  }
}

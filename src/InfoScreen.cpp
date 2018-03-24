/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

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
#include "Locomotive.h"

#ifndef INFO_SCREEN_SDA_PIN
#define INFO_SCREEN_SDA_PIN SDA
#endif
#ifndef INFO_SCREEN_SCL_PIN
#define INFO_SCREEN_SCL_PIN SCL
#endif

#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
#include <Wire.h>
#include "InfoScreen_OLED_font.h"
#if OLED_CHIPSET == SH1106
#include <SH1106Wire.h>
SH1106Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#elif OLED_CHIPSET == SH1306
#include <SSD1306Wire.h>
SSD1306Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#endif
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcdDisplay(INFO_SCREEN_LCD_I2C_ADDRESS);
#endif

bool InfoScreen::_enabled;

#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
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
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
  for(int i = 0; i < INFO_SCREEN_OLED_LINES; i++) {
    infoScreenLines[i] = "";
  }
  Wire.begin(INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);

#if defined(INFO_SCREEN_RESET_PIN)
  pinMode(INFO_SCREEN_RESET_PIN, OUTPUT);
  digitalWrite(INFO_SCREEN_RESET_PIN, LOW);
  delay(50);
  digitalWrite(INFO_SCREEN_RESET_PIN, HIGH);
#endif

  // Check that we can find the OLED screen by its address before attempting
  // to use/configure it.
  Wire.beginTransmission(INFO_SCREEN_OLED_I2C_ADDRESS);
  if(Wire.endTransmission() == 0) {
    oledDisplay.init();
    oledDisplay.setContrast(255);
  	if(INFO_SCREEN_OLED_VERTICAL_FLIP == true) {
  		oledDisplay.flipScreenVertically();
  	}

    // NOTE: If the InfoScreen_OLED_font.h file is modified with a new font
    // definition, the name of the font needs to be declared on the next line.
  	oledDisplay.setFont(Monospaced_plain_10);
    _enabled = true;
  } else {
    Serial.print("OLED screen not found at 0x");
    Serial.println(INFO_SCREEN_OLED_I2C_ADDRESS, HEX);
  }
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
  Wire.begin();
  // Check that we can find the LCD by its address before attempting to use it.
  Wire.beginTransmission(INFO_SCREEN_LCD_I2C_ADDRESS);
  if(Wire.endTransmission() == 0) {
    lcdDisplay.begin(INFO_SCREEN_LCD_COLUMNS, INFO_SCREEN_LCD_LINES);
    lcdDisplay.setBacklight(255);
    lcdDisplay.clear();
    _enabled = true;
  }
#endif
  if(!_enabled) {
    Serial.println("Unable to initialize InfoScreen, switching to Serial");
  }
}

void InfoScreen::clear() {
  if(_enabled) {
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    oledDisplay.clear();
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
    lcdDisplay.clear();
#endif
  }
}

void InfoScreen::printf(int col, int row, const __FlashStringHelper *format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf_P(buf, sizeof(buf), (const char *)format, args);
  va_end(args);
  if(_enabled) {
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    infoScreenLines[row] = infoScreenLines[row].substring(0, col) + buf + infoScreenLines[row].substring(col + strlen(buf));
    redrawOLED();
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(col, row);
      lcdDisplay.print(buf);
    }
#endif
  } else {
    Serial.println(buf);
  }
}

void InfoScreen::printf(int col, int row, const char *format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), (const char *)format, args);
  va_end(args);
  if(_enabled) {
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    infoScreenLines[row] = infoScreenLines[row].substring(0, col) + buf + infoScreenLines[row].substring(col + strlen(buf));
    redrawOLED();
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
    if(row <= INFO_SCREEN_LCD_LINES) {
      lcdDisplay.setCursor(col, row);
      lcdDisplay.print(buf);
    }
#endif
  } else {
    Serial.println(buf);
  }
}

void InfoScreen::replaceLine(int row, const __FlashStringHelper *format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf_P(buf, sizeof(buf), (const char *)format, args);
  va_end(args);
  if(_enabled) {
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    infoScreenLines[row] = buf;
    redrawOLED();
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
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
  } else {
    Serial.println(buf);
  }
}

void InfoScreen::replaceLine(int row, const char *format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), (const char *)format, args);
  va_end(args);
  if(_enabled) {
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED
    infoScreenLines[row] = buf;
    redrawOLED();
#elif defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
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
  } else {
    Serial.println(buf);
  }
}

void InfoScreen::update() {
  static uint8_t _rotatingStatusIndex = 0;
  static uint8_t _motorboardIndex = 0;
  static uint8_t _dccIndex = 0;
  static uint32_t _lastRotation = millis();
  static uint32_t _lastUpdate = millis();
  if(_enabled) {
    // switch to next status line detail set every five seconds
    if(millis() - _lastRotation >= 5000) {
      _lastRotation = millis();
      ++_rotatingStatusIndex %= 3;
    }
    // update the status line details every half second
    if(millis() - _lastUpdate >= 450) {
      _lastUpdate = millis();
      switch(_rotatingStatusIndex) {
        case 0: // free heap
          replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Free Heap: %d"),
            ESP.getFreeHeap());
          break;
        case 1: // locomotive count
          replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Active Locos: %03d"),
            LocomotiveManager::getActiveLocoCount());
          break;
        case 2: // motor shield
          ++_motorboardIndex %= MotorBoardManager::getMotorBoardCount();
          auto board = MotorBoardManager::getBoardByName(MotorBoardManager::getBoardNames()[_motorboardIndex]);
          if(board != nullptr && (board->isOn() || board->isOverCurrent())) {
            if(board->isOverCurrent()) {
              replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:Fault (%2.2f A)"),
                board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
            } else if(board->isOn()) {
              replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s:Normal (%2.2f A)"),
                board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
            }
          } else if(board != nullptr) {
            replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s: Off"),
              board->getName().c_str());
          }
          break;
        }
    }
  }
}

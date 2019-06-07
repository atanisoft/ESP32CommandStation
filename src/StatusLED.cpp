/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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

#include "ESP32CommandStation.h"

#if STATUS_LED_ENABLED

#include <NeoPixelBus.h>

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> statusLEDDriver(3, STATUS_LED_DATA_PIN);
static RgbColor RGB_RED = RgbColor(128, 0, 0);
static RgbColor RGB_GREEN = RgbColor(0, 128, 0);
static RgbColor RGB_YELLOW = RgbColor(128, 128, 0);
static RgbColor RGB_OFF = RgbColor(0);

STATUS_LED_COLOR statusLEDs[STATUS_LED::MAX_STATUS_LED] = {STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF};
bool statusLEDOn[STATUS_LED::MAX_STATUS_LED] = {false, false, false};

void updateStatusLEDs(void *arg) {
    statusLEDDriver.Begin();
    while(true) {
        for(int led = 0; led < STATUS_LED::MAX_STATUS_LED; led++) {
            LOG(VERBOSE, "[STATUS] %d : %d / %d", led, statusLEDs[led], statusLEDOn[led]);
            if(statusLEDs[led] == LED_RED_BLINK || statusLEDs[led] == LED_GREEN_BLINK || statusLEDs[led] == LED_YELLOW_BLINK) {
                if(statusLEDOn[led]) {
                    statusLEDDriver.SetPixelColor(led, RGB_OFF);
                } else if(statusLEDs[led] == LED_RED_BLINK) {
                    statusLEDDriver.SetPixelColor(led, RGB_RED);
                } else if(statusLEDs[led] == LED_GREEN_BLINK) {
                    statusLEDDriver.SetPixelColor(led, RGB_GREEN);
                } else if(statusLEDs[led] == LED_YELLOW_BLINK) {
                    statusLEDDriver.SetPixelColor(led, RGB_YELLOW);
                }
                statusLEDOn[led] = !statusLEDOn[led];
            }
        }
        if(statusLEDDriver.IsDirty()) {
            statusLEDDriver.Show();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setStatusLED(const STATUS_LED led, const STATUS_LED_COLOR color) {
    statusLEDs[led] = color;
    statusLEDOn[led] = true;
    if(statusLEDs[led] == LED_RED || statusLEDs[led] == LED_RED_BLINK) {
        statusLEDDriver.SetPixelColor(led, RGB_RED);
    } else if(statusLEDs[led] == LED_GREEN || statusLEDs[led] == LED_GREEN_BLINK) {
        statusLEDDriver.SetPixelColor(led, RGB_GREEN);
    } else if(statusLEDs[led] == LED_YELLOW || statusLEDs[led] == LED_YELLOW_BLINK) {
        statusLEDDriver.SetPixelColor(led, RGB_YELLOW);
    } else if(statusLEDs[led] == LED_OFF) {
        statusLEDDriver.SetPixelColor(led, RGB_OFF);
        statusLEDOn[led] = false;
    }
}

void initStatusLEDs() {
   xTaskCreatePinnedToCore(updateStatusLEDs, "LED", 2048, nullptr, 2, nullptr, 1);
}

#endif

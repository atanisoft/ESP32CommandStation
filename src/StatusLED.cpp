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

#include <NeoPixelBrightnessBus.h>

#if STATUS_LED_COLOR_ORDER == RGB
#define NEO_COLOR_MODE NeoRgbFeature
#define NEO_COLOR_TYPE RgbColor
#elif STATUS_LED_COLOR_ORDER == GRB
#define NEO_COLOR_MODE NeoGrbFeature
#define NEO_COLOR_TYPE RgbColor
#elif STATUS_LED_COLOR_ORDER == RGBW
#define NEO_COLOR_MODE NeoRgbwFeature
#define NEO_COLOR_TYPE RgbwColor
#elif STATUS_LED_COLOR_ORDER == GRBW
#define NEO_COLOR_MODE NeoGrbwFeature
#define NEO_COLOR_TYPE RgbwColor
#elif STATUS_LED_COLOR_ORDER == BRG
#define NEO_COLOR_MODE NeoBrgFeature
#define NEO_COLOR_TYPE RgbColor
#elif STATUS_LED_COLOR_ORDER == RBG
#define NEO_COLOR_MODE NeoRbgFeature
#define NEO_COLOR_TYPE RgbColor
#endif

#if STATUS_LED_TYPE == WS281X_800
#define NEO_METHOD Neo800KbpsMethod
#elif STATUS_LED_TYPE == WS281X_400
#define NEO_METHOD Neo400KbpsMethod
#elif STATUS_LED_TYPE == SK6812
#define NEO_METHOD NeoSk6812Method
#elif STATUS_LED_TYPE == LC6812
#define NEO_METHOD NeoLc8812Method
#endif

NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD> statusLED(STATUS_LED::MAX_STATUS_LED, STATUS_LED_DATA_PIN);

static NEO_COLOR_TYPE RGB_RED = NEO_COLOR_TYPE(255, 0, 0);
static NEO_COLOR_TYPE RGB_GREEN = NEO_COLOR_TYPE(0, 255, 0);
static NEO_COLOR_TYPE RGB_YELLOW = NEO_COLOR_TYPE(255, 255, 0);
static NEO_COLOR_TYPE RGB_OFF = NEO_COLOR_TYPE(0);

STATUS_LED_COLOR statusLEDColors[STATUS_LED::MAX_STATUS_LED] = {STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF};
bool statusLEDState[STATUS_LED::MAX_STATUS_LED] = {false, false, false};
int64_t statusLEDStateUpdate[STATUS_LED::MAX_STATUS_LED] = {0, 0, 0};
TaskHandle_t statusTaskHandle;

void updateStatusLEDs(void *arg) {
    esp_task_wdt_add(NULL);
    statusLED.Begin();
    statusLED.SetBrightness(STATUS_LED_BRIGHTNESS);
    statusLED.ClearTo(RGB_OFF);
    statusLED.Show();
    while(true) {
        esp_task_wdt_reset();
        for(int led = 0; led < STATUS_LED::MAX_STATUS_LED; led++) {
            // if the LED is set to blink and it has been at least 450ms since we updated it, update it
            if((statusLEDColors[led] == LED_RED_BLINK || statusLEDColors[led] == LED_GREEN_BLINK || statusLEDColors[led] == LED_YELLOW_BLINK) &&
               (esp_timer_get_time() - 450 > statusLEDStateUpdate[led])) {
                if(statusLEDState[led]) {
                    statusLED.SetPixelColor(led, RGB_OFF);
                } else if(statusLEDColors[led] == LED_RED_BLINK) {
                    statusLED.SetPixelColor(led, RGB_RED);
                } else if(statusLEDColors[led] == LED_GREEN_BLINK) {
                    statusLED.SetPixelColor(led, RGB_GREEN);
                } else if(statusLEDColors[led] == LED_YELLOW_BLINK) {
                    statusLED.SetPixelColor(led, RGB_YELLOW);
                }
                statusLEDState[led] = !statusLEDState[led];
                statusLEDStateUpdate[led] = esp_timer_get_time();
            }
        }
        statusLED.Show();
        // go to sleep for up to ~500ms
        ulTaskNotifyTake(true, pdMS_TO_TICKS(500));
    }
}

void setStatusLED(const STATUS_LED led, const STATUS_LED_COLOR color) {
    statusLEDColors[led] = color;
    statusLEDState[led] = false;
    statusLEDStateUpdate[led] = 0;
    // BLINK state will be handled in the task exclusively
    if(statusLEDColors[led] == LED_RED) {
        statusLED.SetPixelColor(led, RGB_RED);
    } else if(statusLEDColors[led] == LED_GREEN) {
        statusLED.SetPixelColor(led, RGB_GREEN);
    } else if(statusLEDColors[led] == LED_YELLOW) {
        statusLED.SetPixelColor(led, RGB_YELLOW);
    } else if(statusLEDColors[led] == LED_OFF) {
        statusLED.SetPixelColor(led, RGB_OFF);
    }
    // wake up task to update the LEDs
    xTaskNotifyGive(statusTaskHandle);
}

void initStatusLEDs() {
   xTaskCreatePinnedToCore(updateStatusLEDs, "LED", 2048, nullptr, 2, &statusTaskHandle, 1);
}

#endif

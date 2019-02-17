// LocoNet Packet Monitor
// Demonstrates the use of the:
//
//   LocoNet.processSwitchSensorMessage(LnPacket)
//
//   Requires the ESP8266 and ESP32 SSD1306 OLED
//   library by Daniel Eichhorn (@squix78) and
//   Fabrice Weinberg (@FWeinb)
//   https://github.com/squix78/esp8266-oled-ssd1306
//
// function and examples of each of the notifyXXXXXXX user call-back functions
#include <stdio.h>
#include <Arduino.h>
#include <LocoNetESP32UART.h>
#include <SSD1306.h>

LocoNetESP32Uart locoNet;
SSD1306 display(0x3c, 5, 4);
#define BUF_SIZE (1024)

void setup()
{

    // Configure the serial port for 57600 baud
    Serial.begin(115200);

    Serial.println("LocoNet Monitor");

    // First initialize the LocoNet interface

    locoNet.begin();
    locoNet.onPacket(CALLBACK_FOR_ALL_OPCODES, [](lnMsg *rxPacket) {
        Serial.print("rx'd ");
        for(uint8_t x = 0; x < 4; x++) {
            uint8_t val = rxPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
            if(val < 16) {
                Serial.print('0');
            }

            Serial.print(val, HEX);
            Serial.print(' ');
        }
        Serial.print("\r\n");
    });
    locoNet.onSwitchRequest([](uint16_t address, bool output, bool direction) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Switch Request: ");
        display.drawString(0, 20, String(address) + direction ? ":Closed" : ":Thrown");
        display.drawString(0, 40, output ? "On" : "Off");
        display.display();

        Serial.print("Switch Request: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
    });
    locoNet.onSwitchReport([](uint16_t address, bool state, bool sensor) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Switch/Sensor Report: ");
        display.drawString(0, 20, String(address) + sensor ? ":Switch" : ":Aux");
        display.drawString(0, 40, state ? "Active" : "Inactive");
        display.display();

        Serial.print("Switch/Sensor Report: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(sensor ? "Switch" : "Aux");
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });
    locoNet.onSensorChange([](uint16_t address, bool state) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Sensor: ");
        display.drawString(0, 20, String(address));
        display.drawString(0, 40, state ? "Active" : "Inactive");
        display.display();
        Serial.print("Sensor: ");
        Serial.print(address, DEC);
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });

    Serial.println("Display Initialising");
    // Then initialise the display interface */
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Loconet Monitor");
    display.display();
}

void loop()
{
    delay(1000);
}
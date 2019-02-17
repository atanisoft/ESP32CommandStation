/* /****************************************************************************
 * 	Copyright (C) 2018 Jeffrey Marten Gillmor
 *
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 *
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 *
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 You will need the following libraries

 AsyncTCP:
 https://github.com/me-no-dev/AsyncTCP

ESPAsyncWebServer:
 https://github.com/me-no-dev/ESPAsyncWebServer


The following should be installed from the library manager
 SPIFFS
 ESPmDNS
 ArduinoJson - Use version 5.13.x not 6.x.Beta


You will need the following plugins:

ESP32 Sketch Data Uploader:
https://github.com/me-no-dev/arduino-esp32fs-plugin
Copyright (c) 2015 Hristo Gochkov

ESP Exception Decoder
https://github.com/me-no-dev/EspExceptionDecoder
Copyright (c) 2015 Hristo Gochkov

The sketch data uploader is used to add the webpage info
to the SPIFFS filesystem on the ESP32


The layout of your sketchfolder should be

esp-locoio-web-|
               |-esp-locoio-web.ino
               |-data-|
                      |-config-|
                      |         |-config.json
                      |
                      |-css-|
                      |     |-loco.css
                      |     |-normalize.css
                      |     |-skeleton.css
                      |
                      |-images-|
                      |        |-favicon.png
                      |
                      |-js-|
                      |    |-loco.js
                      |    |-zepto.min.js
                      |
                      |-index.html

You'll need to upload the sketch data first using the
ESP32 Sketch Data Uploader plugin.

Also don't forget to change the ssid and password for
your wifi network below.


At the moment you have to edit config.json to add your pin mapping
this is on the todo list ot fix in a future release , copy and paste
this in  for each new pin you want to add, then change the "pin" number

{
      "pin": 15,
      "address": 201,
      "type": "Input",
      "contact": "1",
      "activeHigh": 1,
      "switchOffDelay": 0,
      "switchOffDelayLen": 0,
      "turnout": 0,
      "inputPulse": 0,
      "inputPulseLen": 0,
      "lowOnStart": 0,
      "outputPulse": 0,
      "outputPulseLen": 0,
      "flash": 0,
      "flashOnLen": 0,
      "flashOffLen": 0,
      "blockDetctor": 0
    },

You will then also need to update NUM_PINS as well.
*/


#include <LocoNetESP32UART.h>

#include <WiFi.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>         /* Use version 5.13.x not 6.xBeta  */

#define ssid      "MySSID"       // Add your wifi SSID here
#define password  "MyPassword "  // Add your wifi password here.

#define NUM_PINS 8
#define DEBOUNCE_TIME 20

AsyncWebServer server ( 80 );

typedef enum PIN_STATE
{
    PIN_STATE_IDLE,
    PULSE_ON_SENT,
    STATE_CHANGING,
    PULSE_OFF,
    INPUT_ON,
    INPUT_OFF
}PinStateEnum;

typedef struct INPUT_CONFIG_STRUCT
{
    bool activeHigh;
    bool switchOffDelay;
    uint16_t switchOffDelayLen;
    bool turnout;
    bool inputPulse;
    uint16_t inputPulseLen;
    bool switchOnDelay;
    uint16_t switchOnDelayLen;
}InputConfigStruct;


typedef struct OUTPUT_CONFIG_STRUCT
{
    bool lowOnStart;
    bool outputPulse;
    uint16_t outputPulseLen;
    bool flash;
    uint16_t flashOnLen;
    uint16_t flashOffLen;
    bool blockDetctor;
}OutputConfigStruct;


typedef struct PIN_CONFIG_STRUCT
{
    uint8_t pin;
    uint16_t address;
    bool typeInput;
    bool contact1;
    OutputConfigStruct OutputCfg;
    InputConfigStruct InputCfg;
    int pinState;
    uint32_t lastChangeTime;
    uint8_t sendState;
    uint32_t pulseEndAtTime;
}PinConfigStruct;


PinConfigStruct PinConfig[NUM_PINS];

File configFile;
LocoNetESP32Uart locoNet;

void ConfigureWifi(void);
void ConfigureSPIFFS(void);
void ConfigureLocoGPIO(void);
void ConfigureWebserver(void);

void ProcessData(uint8_t *data, size_t len, size_t index, size_t total);
void SpinOnException(void);

void setup()
{
    Serial.begin(115200);

    ConfigureWifi();

    ConfigureSPIFFS();

    ConfigureLocoGPIO();

    ConfigureWebserver();

    locoNet.begin();

    locoNet.onSwitchRequest([](uint16_t address, bool output, bool direction)
    {
#ifdef VERBOSE
        Serial.print("Switch Request: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
#endif
        for(int Idx = 0; Idx < NUM_PINS; Idx++)
        {
            if(PinConfig[Idx].address == address && !PinConfig[Idx].typeInput)
            {
                /* Pulsed outputs only look at the "output"*/
                if(PinConfig[Idx].OutputCfg.outputPulse)
                {
                    /* check if the direction matches what is expected for the contact number */
                    if(PinConfig[Idx].contact1 == direction && output)
                    {
                        digitalWrite(PinConfig[Idx].pin, HIGH);
                        delay(PinConfig[Idx].OutputCfg.outputPulseLen);
                        digitalWrite(PinConfig[Idx].pin, LOW);
                    }
                }
                else if(PinConfig[Idx].OutputCfg.blockDetctor)
                {
                    digitalWrite(PinConfig[Idx].pin, output);
                }
                else
                {
                    digitalWrite(PinConfig[Idx].pin, direction);
                }
            }
        }
    });


#ifdef VERBOSE
    locoNet.onSwitchReport([](uint16_t address, bool state, bool sensor)
    {
        Serial.print("Switch/Sensor Report: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(sensor ? "Switch" : "Aux");
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });
    locoNet.onSwitchState([](uint16_t address, bool output, bool direction)
    {
        Serial.print("Switch State: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
    });
#endif
}

/* This does the main input and output functions accoarding
 *  to the loconet library, it is currently not particularly
 *  tested.
 */
void loop()
{
  /* delay so as to give the other tasks time to do something*/
    delay(2);
    for(int Idx = 0; Idx < NUM_PINS; Idx++)
    {
        if(PinConfig[Idx].typeInput == true)
        {
            int currPinState = digitalRead(PinConfig[Idx].pin);
            int Active = PinConfig[Idx].InputCfg.activeHigh ? HIGH : LOW;

            if(currPinState != PinConfig[Idx].pinState)
            {
                PinConfig[Idx].pinState = currPinState;
                PinConfig[Idx].lastChangeTime = millis();
                PinConfig[Idx].sendState = STATE_CHANGING;
            }
            else if (PinConfig[Idx].sendState = STATE_CHANGING)
            {
                if(PinConfig[Idx].InputCfg.switchOffDelay && (currPinState != Active))
                {
                    if ((millis() - PinConfig[Idx].lastChangeTime) > PinConfig[Idx].InputCfg.switchOffDelayLen)
                    {
                        PinConfig[Idx].sendState = INPUT_OFF;
                    }
                }
                else if(PinConfig[Idx].InputCfg.switchOnDelay && (currPinState == Active))
                {
                    if ((millis() - PinConfig[Idx].lastChangeTime) >= PinConfig[Idx].InputCfg.switchOnDelayLen)
                    {
                        PinConfig[Idx].sendState = INPUT_ON;
                        if(PinConfig[Idx].InputCfg.inputPulse)
                        {
                            PinConfig[Idx].pulseEndAtTime = millis() + PinConfig[Idx].InputCfg.inputPulseLen;
                        }
                    }
                }
                else
                {
                    if((millis() - PinConfig[Idx].lastChangeTime) >= DEBOUNCE_TIME)
                    {
                        if(currPinState == LOW)
                        {
                            PinConfig[Idx].sendState = INPUT_OFF;
                        }
                        else
                        {
                            PinConfig[Idx].sendState = INPUT_ON;
                            if(PinConfig[Idx].InputCfg.inputPulse)
                            {
                                PinConfig[Idx].pulseEndAtTime = millis() + PinConfig[Idx].InputCfg.inputPulseLen;
                            }
                        }
                    }
                }
            }
            else
            {
                /* do nothing */
            }

            /* Work out if it is now time to turn a pulsed input off */
            if(PinConfig[Idx].InputCfg.inputPulse)
            {
                if(PinConfig[Idx].sendState == PULSE_ON_SENT && millis() >= PinConfig[Idx].pulseEndAtTime)
                {
                    PinConfig[Idx].sendState = PULSE_OFF;
                }
            }

            if(PinConfig[Idx].sendState > STATE_CHANGING)
            {
                lnMsg msg;

                if(PinConfig[Idx].InputCfg.turnout)
                {
                    msg.srp.command = OPC_SW_REP;
                    msg.srp.sn1 = PinConfig[Idx].address & LOWER_4K_ADDR_MASK;
                    msg.srp.sn2 = (PinConfig[Idx].address >> UPPER_4K_ADDR_SHIFT) & UPPER_4K_ADDR_MASK;
                    msg.srp.sn2 |= OPC_INPUT_REP_CB | OPC_INPUT_REP_SW;
                    if(PinConfig[Idx].sendState == INPUT_ON)
                    {
                        msg.srp.sn2 |= OPC_SW_REP_THROWN;
                    }
                }
                else
                {
                    msg.ir.command = OPC_INPUT_REP;
                    msg.ir.in1 = PinConfig[Idx].address & LOWER_4K_ADDR_MASK;
                    msg.ir.in2 = (PinConfig[Idx].address >> UPPER_4K_ADDR_SHIFT) & UPPER_4K_ADDR_MASK;
                    msg.ir.in2 |= OPC_INPUT_REP_CB | OPC_INPUT_REP_SW;
                    if(PinConfig[Idx].sendState == INPUT_ON)
                    {
                        msg.ir.in2 |= OPC_INPUT_REP_HI;
                    }
                }
                /* Send the input response. */
                locoNet.send(&msg, 8);

                if(PinConfig[Idx].InputCfg.inputPulse && PinConfig[Idx].sendState == INPUT_ON)
                {
                  PinConfig[Idx].sendState = PULSE_ON_SENT;
                }
                else
                {
                  PinConfig[Idx].sendState = PIN_STATE_IDLE;
                }
            }
        }
    }
}

/* Configure Wifi */
void ConfigureWifi(void)
{
    WiFi.begin ( ssid, password );

    while ( WiFi.status() != WL_CONNECTED )
    {
        delay ( 500 );
        Serial.print ( "." );
    }

    Serial.println ( "" );
    Serial.print ( "Connected to " ); Serial.println ( ssid );
    Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );
}


/* Configure SPIFFS */
void ConfigureSPIFFS(void)
{
    if (!SPIFFS.begin())
    {
        // Serious problem
        Serial.println("SPIFFS Mount failed");
    }
    else
    {
        Serial.println("SPIFFS Mount succesfull");
    }
}

/* Load Config.json */
void ConfigureLocoGPIO(void)
{
    size_t size = 0;
    configFile = SPIFFS.open("/config/config.json", "r");
    if (!configFile)
    {
        Serial.println("File open failed");
        Serial.println("Did you remember to upload the data flash via ESP32 Sketch Data Upload?");
        SpinOnException();
    }
    size = configFile.size();
    if (size > 20000)
    {
        Serial.println("Config file size is too large");
        SpinOnException();
    }

    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size]);

    // We don't use String here because ArduinoJson library requires the input
    // buffer to be mutable. If you don't use ArduinoJson, you may as well
    // use configFile.readString instead.
    configFile.readBytes(buf.get(), size);

    DynamicJsonBuffer jsonBuffer;
    JsonObject& root   = jsonBuffer.parseObject(buf.get());

    if (!root.success())
    {
        Serial.println("Failed to parse config file");
        SpinOnException();
    }

    for(int Idx = 0; Idx < NUM_PINS; Idx++)
    {
        JsonObject& v1 = root["pinconfigs"][Idx];
        String type = v1["type"];
        PinConfig[Idx].pin = v1["pin"];
        if(type.equals("Input"))
        {
            PinConfig[Idx].typeInput = true;
            pinMode(PinConfig[Idx].pin, INPUT_PULLUP);
            Serial.print("Setting pin ");
            Serial.print(PinConfig[Idx].pin);
            Serial.println(" to input.");
        }
        else
        {
            PinConfig[Idx].typeInput = false;
            Serial.print("Setting pin ");
            Serial.print(PinConfig[Idx].pin);
            pinMode(PinConfig[Idx].pin, OUTPUT);
            if(v1["lowOnStart"] == 0)
            {
                digitalWrite(PinConfig[Idx].pin, HIGH);
                PinConfig[Idx].OutputCfg.lowOnStart = false;
                Serial.println(" to output, and setting HIGH.");
            }
            else
            {
                PinConfig[Idx].OutputCfg.lowOnStart = true;
                digitalWrite(PinConfig[Idx].pin, LOW);
                Serial.println(" to output, and setting LOW.");
            }
        }
        PinConfig[Idx].address = v1["address"];
        PinConfig[Idx].contact1 = (v1["contact"]==1);
        PinConfig[Idx].InputCfg.activeHigh = (v1["activeHigh"]==1);
        PinConfig[Idx].InputCfg.switchOffDelay = (v1["switchOffDelay"]==1);
        PinConfig[Idx].InputCfg.switchOffDelayLen = v1["switchOffDelayLen"];
        PinConfig[Idx].InputCfg.turnout = (v1["turnout"]==1);
        PinConfig[Idx].InputCfg.inputPulse = (v1["inputPulse"]==1);
        PinConfig[Idx].InputCfg.inputPulseLen = v1["inputPulseLen"];
        PinConfig[Idx].OutputCfg.lowOnStart = (v1["lowOnStart"]==1);
        PinConfig[Idx].OutputCfg.outputPulse = (v1["outputPulse"]==1);
        PinConfig[Idx].OutputCfg.outputPulseLen = v1["outputPulseLen"];
        PinConfig[Idx].OutputCfg.flash = (v1["flash"]==1);
        PinConfig[Idx].OutputCfg.flashOnLen = v1["flashOnLen"];
        PinConfig[Idx].OutputCfg.flashOffLen = v1["flashOffLen"];
        PinConfig[Idx].OutputCfg.blockDetctor = (v1["blockDetctor"]==1);
    }


    configFile.close();
}

/* Configure Webserver */
void ConfigureWebserver(void)
{
    //max age is in seconds
    server.serveStatic("/", SPIFFS, "/index.html/");
    server.serveStatic("/js/loco.js", SPIFFS, "/js/loco.js");
    server.serveStatic("/js/zepto.min.js", SPIFFS, "/js/zepto.min.js").setCacheControl("max-age=600");
    server.serveStatic("/css/loco.css", SPIFFS, "/css/loco.css").setCacheControl("max-age=1");
    server.serveStatic("/css/skeleton.css", SPIFFS, "/css/skeleton.css").setCacheControl("max-age=600");
    server.serveStatic("/css/normalize.css", SPIFFS, "/css/normalize.css").setCacheControl("max-age=600");
    server.serveStatic("/config/config.json", SPIFFS, "/config/config.json");
    server.serveStatic("/images/favicon.png", SPIFFS, "/images/favicon.png").setCacheControl("max-age=1");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/index.html", HTTP_POST,
          [](AsyncWebServerRequest *request)
          {
              /* do nothing */
          },
          [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
          {
              /* do nothing */
          },
          [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
          {
              ProcessData( data,  len,  index,  total);
          });

    server.begin();
    Serial.println ( "HTTP server started" );
}

void ProcessData(uint8_t *data, size_t len, size_t index, size_t total)
{
    if (!configFile)
    {
        configFile = SPIFFS.open("/config/config.json", "w");

        if (!configFile)
        {
            Serial.println("Failed to open config file for writing");
            return;
        }
    }


    for(int idx = 0; idx< len; idx++, data++)
    {
        configFile.write(*data);
        Serial.write(*data);
    }

    if((index+len) >= total)
    {
        int waitCount = 0;
        configFile.close();
        Serial.println(" ");
        Serial.println("Applying new config");
        while(waitCount < 5)
        {
            waitCount++;
            delay(500);
            Serial.print(".");
        }

        ConfigureLocoGPIO();
    }
}

void SpinOnException(void)
{
    while(1)
    {
        delay(1);
    }
}

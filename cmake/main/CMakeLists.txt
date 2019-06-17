#######################################################################
# DCC COMMAND STATION FOR ESP32
#
# COPYRIGHT (c) 2017-2019 Mike Dunston
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses
#######################################################################


set(ESP32_CS_BASE_DIR ${COMPONENT_PATH}/../..)
set(ESP32_SYSLIB_DIR ${COMPONENT_PATH}/../syslib)

set(SYSLIB_SRCS
    ${ESP32_SYSLIB_DIR}/AsyncTCP/src/AsyncTCP.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/AsyncEventSource.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/AsyncWebSocket.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/SPIFFSEditor.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/WebAuthentication.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/WebHandlers.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/WebRequest.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/WebResponses.cpp
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src/WebServer.cpp
    ${ESP32_SYSLIB_DIR}/esp8266-oled-ssd1306/src/OLEDDisplay.cpp
    ${ESP32_SYSLIB_DIR}/esp8266-oled-ssd1306/src/OLEDDisplayUi.cpp
    ${ESP32_SYSLIB_DIR}/LiquidCrystal_PCF8574/src/LiquidCrystal_PCF8574.cpp
)

set(SYSLIB_INCS
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src
    ${ESP32_SYSLIB_DIR}/AsyncTCP/src
    ${ESP32_SYSLIB_DIR}/esp8266-oled-ssd1306/src
    ${ESP32_SYSLIB_DIR}/LiquidCrystal_PCF8574/src
    ${ESP32_SYSLIB_DIR}/ESPAsyncWebServer/src
    ${ESP32_SYSLIB_DIR}/ArduinoJson
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Data
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Deserialization
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Polyfills
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Serialization
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/StringTraits
    ${ESP32_SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/TypeTraits
)





set(ESP32_CS_LIB_DIR ${ESP32_CS_BASE_DIR}/lib)
set(ESP32_CS_LIB_INCS
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/os
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/dcc
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/freertos_drivers/esp32
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/freertos_drivers/arduino
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/sys
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src
    ${ESP32_CS_LIB_DIR}/NeoNextion/src
    ${ESP32_CS_LIB_DIR}/LocoNet2/src
)
set(ESP32_CS_LIB_SRCS
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNet.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetAvrICP.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetCV.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetESP32.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetESP32UART.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetFastClock.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetMessageBuffer.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetSystemVariable.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetThrottle.cpp
    ${ESP32_CS_LIB_DIR}/LocoNet2/src/LocoNetUART.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/INextionColourable.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/INextionFontStyleable.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/INextionTouchable.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/INextionWidget.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/Nextion.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionCrop.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionPage.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionPicture.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionSlidingText.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionTimer.cpp
    ${ESP32_CS_LIB_DIR}/NeoNextion/src/NextionWaveform.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/OpenMRNLite.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/dcc/Packet.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor/Executor.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor/Notifiable.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor/Service.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor/StateFlow.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/executor/Timer.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/AliasAllocator.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/AliasCache.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/CanDefs.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/ConfigEntry.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/ConfigUpdateFlow.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/Datagram.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/DatagramCan.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/DccAccyProducer.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/DefaultCdi.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/DefaultNode.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/EventHandler.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/EventHandlerContainer.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/EventHandlerTemplates.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/EventService.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/If.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/IfCan.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/IfImpl.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/IfTcp.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/MemoryConfig.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/nmranet_constants.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/NodeBrowser.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/NodeInitializeFlow.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/NonAuthoritativeEventProducer.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/PIPClient.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/RoutingLogic.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/SimpleNodeInfo.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/SimpleNodeInfoMockUserFile.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/SimpleStack.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TcpDefs.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionCvSpace.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionDefs.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionProxy.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionTestTrain.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionThrottle.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/TractionTrain.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/Velocity.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/openlcb/WriteHelper.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/os/os.c
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/os/stack_malloc.c
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/os/watchdog.c
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/Buffer.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/CanIf.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/ConfigUpdateListener.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/constants.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/Crc.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/errno_exit.c
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/FileUtils.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/format_utils.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/GcStreamParser.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/GcTcpHub.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/gc_format.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/GridConnect.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/GridConnectHub.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/HubDevice.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/HubDeviceSelect.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/ieeehalfprecision.c
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/JSHubPort.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/logging.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/Queue.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/SimpleQueue.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/SocketClient.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/socket_listener.cpp
    ${ESP32_CS_LIB_DIR}/OpenMRNLite/src/utils/StringPrintf.cpp
)



set(ESP32_CS_SRCS
  ${ESP32_CS_BASE_DIR}/src/ConfigurationManager.cpp
  ${ESP32_CS_BASE_DIR}/src/ESP32CommandStation.cpp
  ${ESP32_CS_BASE_DIR}/src/InfoScreen.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/DCCProgrammer.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/DCCSignalGenerator.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/DCCSignalGenerator_RMT.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/DCCSignalGenerator_Timer.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/MotorBoard.cpp
  ${ESP32_CS_BASE_DIR}/src/DCC/Turnouts.cpp
  ${ESP32_CS_BASE_DIR}/src/Interfaces/DCCppProtocol.cpp
  ${ESP32_CS_BASE_DIR}/src/Interfaces/HC12Interface.cpp
  ${ESP32_CS_BASE_DIR}/src/Interfaces/WebServer.cpp
  ${ESP32_CS_BASE_DIR}/src/Interfaces/WiFiInterface.cpp
  ${ESP32_CS_BASE_DIR}/src/IO/Outputs.cpp
  ${ESP32_CS_BASE_DIR}/src/IO/RemoteSensors.cpp
  ${ESP32_CS_BASE_DIR}/src/IO/S88Sensors.cpp
  ${ESP32_CS_BASE_DIR}/src/IO/Sensors.cpp
  ${ESP32_CS_BASE_DIR}/src/LCC/LCCCVMemorySpace.cpp
  ${ESP32_CS_BASE_DIR}/src/LCC/LCCInterface.cpp
  ${ESP32_CS_BASE_DIR}/src/Locomotives/Locomotive.cpp
  ${ESP32_CS_BASE_DIR}/src/Locomotives/LocomotiveConsist.cpp
  ${ESP32_CS_BASE_DIR}/src/Locomotives/LocomotiveManager.cpp
  ${ESP32_CS_BASE_DIR}/src/Nextion/NextionInterface.cpp
  ${ESP32_CS_BASE_DIR}/src/Nextion/NextionInterfaceAddressPage.cpp
  ${ESP32_CS_BASE_DIR}/src/Nextion/NextionInterfaceBasePage.cpp
  ${ESP32_CS_BASE_DIR}/src/Nextion/NextionInterfaceThrottlePage.cpp
  ${ESP32_CS_BASE_DIR}/src/Nextion/NextionInterfaceTurnoutPage.cpp
)

set(INCLUDE_SRCS
  ${ESP32_CS_BASE_DIR}/include/Config.h
  ${ESP32_CS_BASE_DIR}/include/ConfigurationManager.h
  ${ESP32_CS_BASE_DIR}/include/Config_HC12.h
  ${ESP32_CS_BASE_DIR}/include/Config_LCC.h
  ${ESP32_CS_BASE_DIR}/include/Config_LCD.h
  ${ESP32_CS_BASE_DIR}/include/Config_LocoNet.h
  ${ESP32_CS_BASE_DIR}/include/Config_MotorBoard.h
  ${ESP32_CS_BASE_DIR}/include/Config_Nextion.h
  ${ESP32_CS_BASE_DIR}/include/Config_OLED.h
  ${ESP32_CS_BASE_DIR}/include/Config_S88.h
  ${ESP32_CS_BASE_DIR}/include/Config_Status.h
  ${ESP32_CS_BASE_DIR}/include/Config_WiFi.h
  ${ESP32_CS_BASE_DIR}/include/DCCppProtocol.h
  ${ESP32_CS_BASE_DIR}/include/DCCProgrammer.h
  ${ESP32_CS_BASE_DIR}/include/DCCSignalGenerator.h
  ${ESP32_CS_BASE_DIR}/include/DCCSignalGenerator_RMT.h
  ${ESP32_CS_BASE_DIR}/include/DCCSignalGenerator_Timer.h
  ${ESP32_CS_BASE_DIR}/include/ESP32CommandStation.h
  ${ESP32_CS_BASE_DIR}/include/HC12Interface.h
  ${ESP32_CS_BASE_DIR}/include/InfoScreen.h
  ${ESP32_CS_BASE_DIR}/include/InfoScreen_OLED_font.h
  ${ESP32_CS_BASE_DIR}/include/JsonConstants.h
  ${ESP32_CS_BASE_DIR}/include/LCCCDI.h
  ${ESP32_CS_BASE_DIR}/include/LCCInterface.h
  ${ESP32_CS_BASE_DIR}/include/Locomotive.h
  ${ESP32_CS_BASE_DIR}/include/MotorBoard.h
  ${ESP32_CS_BASE_DIR}/include/NextionInterface.h
  ${ESP32_CS_BASE_DIR}/include/Outputs.h
  ${ESP32_CS_BASE_DIR}/include/RemoteSensors.h
  ${ESP32_CS_BASE_DIR}/include/S88Sensors.h
  ${ESP32_CS_BASE_DIR}/include/Sensors.h
  ${ESP32_CS_BASE_DIR}/include/Turnouts.h
  ${ESP32_CS_BASE_DIR}/include/WebServer.h
  ${ESP32_CS_BASE_DIR}/include/WiFiInterface.h
)

set(COMPONENT_SRCS
  ${INCLUDE_SRCS}
  ${SYSLIB_SRCS}
  ${ESP32_CS_LIB_SRCS}
  ${ESP32_CS_SRCS}
)
set(COMPONENT_ADD_INCLUDEDIRS 
  ${ESP32_CS_BASE_DIR}/include
  ${ESP32_CS_LIB_INCS}
  ${SYSLIB_INCS}
)

# the arduino github repo for esp32
set(EXTRA_COMPONENT_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/syslib/arduino-esp32
)

register_component()


#-------------------------------------------------------------
# compress index.html into   into 'include/index_html.h'
#-------------------------------------------------------------

set(HTML_SRC ${COMPONENT_PATH}/../../data/index.html)
set(INDEX_HTML_H ${COMPONENT_PATH}/../../include/index_html.h)
add_custom_command(OUTPUT "${INDEX_HTML_H}"
    COMMAND ${PYTHON} ${COMPONENT_PATH}/build_index_header_cmake.py -i ${HTML_SRC} -o ${INDEX_HTML_H}
    DEPENDS "${HTML_SRC}"
    VERBATIM)
add_custom_target(create_index_html_h ALL DEPENDS ${INDEX_HTML_H})
add_dependencies(${COMPONENT_TARGET} create_index_html_h)         

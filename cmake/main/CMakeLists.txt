set(ESP32_CS_BASEDIR ${COMPONENT_PATH}/../..)
set(SYSLIB_DIR ${COMPONENT_PATH}/../syslib)

set(SYSLIB_SRCS
    ${SYSLIB_DIR}/AsyncTCP/src/AsyncTCP.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/AsyncEventSource.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/AsyncWebSocket.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/SPIFFSEditor.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/WebAuthentication.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/WebHandlers.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/WebRequest.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/WebResponses.cpp
    ${SYSLIB_DIR}/ESPAsyncWebServer/src/WebServer.cpp
    ${SYSLIB_DIR}/esp8266-oled-ssd1306/src/OLEDDisplay.cpp
    ${SYSLIB_DIR}/esp8266-oled-ssd1306/src/OLEDDisplayUi.cpp
    ${SYSLIB_DIR}/LiquidCrystal_PCF8574/src/LiquidCrystal_PCF8574.cpp
)

set(SYSLIB_INCS
    ${SYSLIB_DIR}/ArduinoJson/src
    ${SYSLIB_DIR}/AsyncTCP/src
    ${SYSLIB_DIR}/esp8266-oled-ssd1306/src
    ${SYSLIB_DIR}/LiquidCrystal_PCF8574/src
    ${SYSLIB_DIR}/ESPAsyncWebServer/src
    ${SYSLIB_DIR}/ArduinoJson
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Data
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Deserialization
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Polyfills
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/Serialization
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/StringTraits
    ${SYSLIB_DIR}/ArduinoJson/src/ArduinoJson/TypeTraits
)





set(ESP32_CS_LIBDIR ${ESP32_CS_BASEDIR}/lib)
set(DCCPPLIB_INCS
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/os
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/dcc
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/freertos_drivers/esp32
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/freertos_drivers/arduino
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/sys
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src
    ${ESP32_CS_LIBDIR}/NeoNextion/src
    ${ESP32_CS_LIBDIR}/LocoNet2/src
)
set(DCCPPLIB_SRCS
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNet.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetAvrICP.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetCV.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetESP32.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetESP32UART.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetFastClock.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetMessageBuffer.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetSystemVariable.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetThrottle.cpp
    ${ESP32_CS_LIBDIR}/LocoNet2/src/LocoNetUART.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/INextionColourable.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/INextionFontStyleable.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/INextionTouchable.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/INextionWidget.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/Nextion.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionCrop.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionPage.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionPicture.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionSlidingText.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionTimer.cpp
    ${ESP32_CS_LIBDIR}/NeoNextion/src/NextionWaveform.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/OpenMRNLite.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/dcc/Packet.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor/Executor.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor/Notifiable.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor/Service.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor/StateFlow.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/executor/Timer.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/AliasAllocator.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/AliasCache.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/CanDefs.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/ConfigEntry.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/ConfigUpdateFlow.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/Datagram.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/DatagramCan.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/DccAccyProducer.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/DefaultCdi.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/DefaultNode.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/EventHandler.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/EventHandlerContainer.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/EventHandlerTemplates.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/EventService.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/If.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/IfCan.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/IfImpl.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/IfTcp.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/MemoryConfig.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/nmranet_constants.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/NodeBrowser.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/NodeInitializeFlow.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/NonAuthoritativeEventProducer.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/PIPClient.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/RoutingLogic.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/SimpleNodeInfo.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/SimpleNodeInfoMockUserFile.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/SimpleStack.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TcpDefs.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionCvSpace.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionDefs.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionProxy.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionTestTrain.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionThrottle.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/TractionTrain.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/Velocity.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/openlcb/WriteHelper.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/os/os.c
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/os/stack_malloc.c
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/os/watchdog.c
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/Buffer.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/CanIf.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/ConfigUpdateListener.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/constants.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/Crc.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/errno_exit.c
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/FileUtils.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/format_utils.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/GcStreamParser.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/GcTcpHub.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/gc_format.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/GridConnect.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/GridConnectHub.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/HubDevice.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/HubDeviceSelect.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/ieeehalfprecision.c
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/JSHubPort.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/logging.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/Queue.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/SimpleQueue.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/SocketClient.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/socket_listener.cpp
    ${ESP32_CS_LIBDIR}/OpenMRNLite/src/utils/StringPrintf.cpp
)



set(DCCPP_SRCS
  ${ESP32_CS_BASEDIR}/src/ConfigurationManager.cpp
  ${ESP32_CS_BASEDIR}/src/ESP32CommandStation.cpp
  ${ESP32_CS_BASEDIR}/src/InfoScreen.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/DCCProgrammer.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/DCCSignalGenerator.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/DCCSignalGenerator_RMT.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/DCCSignalGenerator_Timer.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/MotorBoard.cpp
  ${ESP32_CS_BASEDIR}/src/DCC/Turnouts.cpp
  ${ESP32_CS_BASEDIR}/src/Interfaces/DCCppProtocol.cpp
  ${ESP32_CS_BASEDIR}/src/Interfaces/HC12Interface.cpp
  ${ESP32_CS_BASEDIR}/src/Interfaces/WebServer.cpp
  ${ESP32_CS_BASEDIR}/src/Interfaces/WiFiInterface.cpp
  ${ESP32_CS_BASEDIR}/src/IO/Outputs.cpp
  ${ESP32_CS_BASEDIR}/src/IO/RemoteSensors.cpp
  ${ESP32_CS_BASEDIR}/src/IO/S88Sensors.cpp
  ${ESP32_CS_BASEDIR}/src/IO/Sensors.cpp
  ${ESP32_CS_BASEDIR}/src/LCC/LCCCVMemorySpace.cpp
  ${ESP32_CS_BASEDIR}/src/LCC/LCCInterface.cpp
  ${ESP32_CS_BASEDIR}/src/Locomotives/Locomotive.cpp
  ${ESP32_CS_BASEDIR}/src/Locomotives/LocomotiveConsist.cpp
  ${ESP32_CS_BASEDIR}/src/Locomotives/LocomotiveManager.cpp
  ${ESP32_CS_BASEDIR}/src/Nextion/NextionInterface.cpp
  ${ESP32_CS_BASEDIR}/src/Nextion/NextionInterfaceAddressPage.cpp
  ${ESP32_CS_BASEDIR}/src/Nextion/NextionInterfaceBasePage.cpp
  ${ESP32_CS_BASEDIR}/src/Nextion/NextionInterfaceThrottlePage.cpp
  ${ESP32_CS_BASEDIR}/src/Nextion/NextionInterfaceTurnoutPage.cpp
)

set(INCLUDE_SRCS
  ${ESP32_CS_BASEDIR}/include/Config.h
  ${ESP32_CS_BASEDIR}/include/ConfigurationManager.h
  ${ESP32_CS_BASEDIR}/include/Config_HC12.h
  ${ESP32_CS_BASEDIR}/include/Config_LCC.h
  ${ESP32_CS_BASEDIR}/include/Config_LCD.h
  ${ESP32_CS_BASEDIR}/include/Config_LocoNet.h
  ${ESP32_CS_BASEDIR}/include/Config_MotorBoard.h
  ${ESP32_CS_BASEDIR}/include/Config_Nextion.h
  ${ESP32_CS_BASEDIR}/include/Config_OLED.h
  ${ESP32_CS_BASEDIR}/include/Config_S88.h
  ${ESP32_CS_BASEDIR}/include/Config_Status.h
  ${ESP32_CS_BASEDIR}/include/Config_WiFi.h
  ${ESP32_CS_BASEDIR}/include/DCCppProtocol.h
  ${ESP32_CS_BASEDIR}/include/DCCProgrammer.h
  ${ESP32_CS_BASEDIR}/include/DCCSignalGenerator.h
  ${ESP32_CS_BASEDIR}/include/DCCSignalGenerator_RMT.h
  ${ESP32_CS_BASEDIR}/include/DCCSignalGenerator_Timer.h
  ${ESP32_CS_BASEDIR}/include/ESP32CommandStation.h
  ${ESP32_CS_BASEDIR}/include/HC12Interface.h
  ${ESP32_CS_BASEDIR}/include/InfoScreen.h
  ${ESP32_CS_BASEDIR}/include/InfoScreen_OLED_font.h
  ${ESP32_CS_BASEDIR}/include/JsonConstants.h
  ${ESP32_CS_BASEDIR}/include/LCCCDI.h
  ${ESP32_CS_BASEDIR}/include/LCCInterface.h
  ${ESP32_CS_BASEDIR}/include/Locomotive.h
  ${ESP32_CS_BASEDIR}/include/MotorBoard.h
  ${ESP32_CS_BASEDIR}/include/NextionInterface.h
  ${ESP32_CS_BASEDIR}/include/Outputs.h
  ${ESP32_CS_BASEDIR}/include/RemoteSensors.h
  ${ESP32_CS_BASEDIR}/include/S88Sensors.h
  ${ESP32_CS_BASEDIR}/include/Sensors.h
  ${ESP32_CS_BASEDIR}/include/Turnouts.h
  ${ESP32_CS_BASEDIR}/include/WebServer.h
  ${ESP32_CS_BASEDIR}/include/WiFiInterface.h
)

set(COMPONENT_SRCS
  ${INCLUDE_SRCS}
  ${SYSLIB_SRCS}
  ${DCCPPLIB_SRCS}
  ${DCCPP_SRCS}
)
set(COMPONENT_ADD_INCLUDEDIRS 
  ${ESP32_CS_BASEDIR}/include
  ${DCCPPLIB_INCS}
  ${SYSLIB_INCS}
)

# the arduino github repo for esp32:
# git submodule add https://github.com/espressif/arduino-esp32.git; cd arduino-esp32; git submodule update --init --recursive
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

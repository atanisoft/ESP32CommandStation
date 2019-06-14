set(ATANI_DCC_BASEDIR ${COMPONENT_PATH}/../..)
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





set(DCCPPLIB_BASE_DIR ${ATANI_DCC_BASEDIR}/lib)
set(DCCPPLIB_INCS
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/os
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/dcc
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/freertos_drivers/esp32
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/freertos_drivers/arduino
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/sys
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src
)
set(DCCPPLIB_SRCS
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNet.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetAvrICP.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetCV.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetESP32.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetESP32UART.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetFastClock.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetMessageBuffer.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetSystemVariable.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetThrottle.cpp
    ${DCCPPLIB_BASE_DIR}/LocoNet2/src/LocoNetUART.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/INextionColourable.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/INextionFontStyleable.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/INextionTouchable.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/INextionWidget.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/Nextion.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionCrop.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionPage.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionPicture.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionSlidingText.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionTimer.cpp
    ${DCCPPLIB_BASE_DIR}/NeoNextion/src/NextionWaveform.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/OpenMRNLite.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/dcc/Packet.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor/Executor.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor/Notifiable.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor/Service.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor/StateFlow.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/executor/Timer.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/AliasAllocator.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/AliasCache.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/CanDefs.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/ConfigEntry.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/ConfigUpdateFlow.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/Datagram.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/DatagramCan.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/DccAccyProducer.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/DefaultCdi.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/DefaultNode.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/EventHandler.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/EventHandlerContainer.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/EventHandlerTemplates.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/EventService.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/If.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/IfCan.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/IfImpl.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/IfTcp.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/MemoryConfig.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/nmranet_constants.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/NodeBrowser.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/NodeInitializeFlow.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/NonAuthoritativeEventProducer.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/PIPClient.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/RoutingLogic.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/SimpleNodeInfo.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/SimpleNodeInfoMockUserFile.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/SimpleStack.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TcpDefs.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionCvSpace.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionDefs.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionProxy.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionTestTrain.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionThrottle.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/TractionTrain.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/Velocity.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/openlcb/WriteHelper.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/os/os.c
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/os/stack_malloc.c
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/os/watchdog.c
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/Buffer.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/CanIf.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/ConfigUpdateListener.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/constants.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/Crc.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/errno_exit.c
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/FileUtils.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/format_utils.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/GcStreamParser.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/GcTcpHub.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/gc_format.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/GridConnect.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/GridConnectHub.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/HubDevice.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/HubDeviceSelect.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/ieeehalfprecision.c
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/JSHubPort.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/logging.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/Queue.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/SimpleQueue.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/SocketClient.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/socket_listener.cpp
    ${DCCPPLIB_BASE_DIR}/OpenMRNLite/src/utils/StringPrintf.cpp
)



set(DCCPP_SRCS
  ${ATANI_DCC_BASEDIR}/src/ConfigurationManager.cpp
  ${ATANI_DCC_BASEDIR}/src/DCCppESP32.cpp
  ${ATANI_DCC_BASEDIR}/src/InfoScreen.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/DCCProgrammer.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/DCCSignalGenerator.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/DCCSignalGenerator_RMT.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/DCCSignalGenerator_Timer.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/MotorBoard.cpp
  ${ATANI_DCC_BASEDIR}/src/DCC/Turnouts.cpp
  ${ATANI_DCC_BASEDIR}/src/Interfaces/DCCppProtocol.cpp
  ${ATANI_DCC_BASEDIR}/src/Interfaces/HC12Interface.cpp
  ${ATANI_DCC_BASEDIR}/src/Interfaces/WebServer.cpp
  ${ATANI_DCC_BASEDIR}/src/Interfaces/WiFiInterface.cpp
  ${ATANI_DCC_BASEDIR}/src/IO/Outputs.cpp
  ${ATANI_DCC_BASEDIR}/src/IO/RemoteSensors.cpp
  ${ATANI_DCC_BASEDIR}/src/IO/S88Sensors.cpp
  ${ATANI_DCC_BASEDIR}/src/IO/Sensors.cpp
  ${ATANI_DCC_BASEDIR}/src/LCC/LCCCVMemorySpace.cpp
  ${ATANI_DCC_BASEDIR}/src/LCC/LCCInterface.cpp
  ${ATANI_DCC_BASEDIR}/src/Locomotives/Locomotive.cpp
  ${ATANI_DCC_BASEDIR}/src/Locomotives/LocomotiveConsist.cpp
  ${ATANI_DCC_BASEDIR}/src/Locomotives/LocomotiveManager.cpp
  ${ATANI_DCC_BASEDIR}/src/Nextion/NextionInterface.cpp
  ${ATANI_DCC_BASEDIR}/src/Nextion/NextionInterfaceAddressPage.cpp
  ${ATANI_DCC_BASEDIR}/src/Nextion/NextionInterfaceBasePage.cpp
  ${ATANI_DCC_BASEDIR}/src/Nextion/NextionInterfaceThrottlePage.cpp
  ${ATANI_DCC_BASEDIR}/src/Nextion/NextionInterfaceTurnoutPage.cpp
)

set(INCLUDE_SRCS
  ${ATANI_DCC_BASEDIR}/include/Config.h                                                              
  ${ATANI_DCC_BASEDIR}/include/ConfigurationManager.h                                                
  ${ATANI_DCC_BASEDIR}/include/Config_HC12.h                                                         
  ${ATANI_DCC_BASEDIR}/include/Config_LCC.h                                                          
  ${ATANI_DCC_BASEDIR}/include/Config_LCD.h                                                          
  ${ATANI_DCC_BASEDIR}/include/Config_LocoNet.h                                                      
  ${ATANI_DCC_BASEDIR}/include/Config_MotorBoard.h                                                   
  ${ATANI_DCC_BASEDIR}/include/Config_Nextion.h                                                      
  ${ATANI_DCC_BASEDIR}/include/Config_OLED.h                                                         
  ${ATANI_DCC_BASEDIR}/include/Config_S88.h                                                          
  ${ATANI_DCC_BASEDIR}/include/Config_WiFi.h                                                         
  ${ATANI_DCC_BASEDIR}/include/DCCppESP32.h                                                          
  ${ATANI_DCC_BASEDIR}/include/DCCppProtocol.h                                                       
  ${ATANI_DCC_BASEDIR}/include/DCCProgrammer.h                                                       
  ${ATANI_DCC_BASEDIR}/include/DCCSignalGenerator.h                                                  
  ${ATANI_DCC_BASEDIR}/include/DCCSignalGenerator_RMT.h                                              
  ${ATANI_DCC_BASEDIR}/include/DCCSignalGenerator_Timer.h                                            
  ${ATANI_DCC_BASEDIR}/include/HC12Interface.h                                                       
  ${ATANI_DCC_BASEDIR}/include/index_html.h                                                          
  ${ATANI_DCC_BASEDIR}/include/InfoScreen.h                                                          
  ${ATANI_DCC_BASEDIR}/include/InfoScreen_OLED_font.h                                                
  ${ATANI_DCC_BASEDIR}/include/JsonConstants.h                                                       
  ${ATANI_DCC_BASEDIR}/include/LCCCDI.h                                                              
  ${ATANI_DCC_BASEDIR}/include/LCCInterface.h                                                        
  ${ATANI_DCC_BASEDIR}/include/Locomotive.h                                                          
  ${ATANI_DCC_BASEDIR}/include/MotorBoard.h                                                          
  ${ATANI_DCC_BASEDIR}/include/NextionInterface.h                                                    
  ${ATANI_DCC_BASEDIR}/include/Outputs.h                                                             
  ${ATANI_DCC_BASEDIR}/include/RemoteSensors.h                                                       
  ${ATANI_DCC_BASEDIR}/include/S88Sensors.h                                                          
  ${ATANI_DCC_BASEDIR}/include/Sensors.h                                                             
  ${ATANI_DCC_BASEDIR}/include/Turnouts.h                                                            
  ${ATANI_DCC_BASEDIR}/include/WebServer.h                                                           
  ${ATANI_DCC_BASEDIR}/include/WiFiInterface.h                                                       
)

set(COMPONENT_SRCS
  ${INCLUDE_SRCS}
  ${SYSLIB_SRCS}
  ${DCCPPLIB_SRCS}
  ${DCCPP_SRCS}
)
set(COMPONENT_ADD_INCLUDEDIRS 
  ${ATANI_DCC_BASEDIR}/include
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

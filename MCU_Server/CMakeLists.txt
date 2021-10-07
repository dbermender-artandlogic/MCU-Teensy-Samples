file(GLOB   LOG_SRC         ./Log.cpp)

add_library(LogIf INTERFACE)

add_library(Log STATIC EXCLUDE_FROM_ALL ${LOG_SRC})

target_link_libraries(Log PUBLIC LogIf)
                      
createArduinoCMock(MockCRC ./CRC.h)
testIncludeDirectories(MockCRC .)

createArduinoCMock(MockUARTDriver ./UARTDriver.h)
testIncludeDirectories(MockUARTDriver .)

createArduinoCMock(MockRTC ./RTC.h)
testIncludeDirectories(MockRTC .)


cmake_minimum_required(VERSION 3.5)
project(dogbotapi LANGUAGES CXX)
#VERSION 0.1 
FIND_PACKAGE ( Threads REQUIRED )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  pkg_check_modules(ZMQ libzmq)
  pkg_check_modules(LIBUSB libusb-1.0)
endif()

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

include_directories(${ZMQ_INCLUDE_DIRS})
link_directories(${ZMQ_LIBRARY_DIRS})

include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

add_definitions(-std=c++11)

ADD_LIBRARY (DogBotAPI
        Coms.cc 
        ComsZMQServer.cc 
        ComsZMQClient.cc
        ComsSerial.cc 
        ComsProxy.cc 
        ComsUSB.cc 
        DogBotAPI.cc 
        Servo.cc 
        LegKinematics.cc 
        Joint.cc 
        JointRelative.cc 
        Joint4BarLinkage.cc 
        ZMQContext.cc
)

target_link_libraries (DogBotAPI LINK_PUBLIC ${JSONCPP_LIBRARIES} ${ZMQ_LIBRARIES} ${LIBUSB_LIBRARIES})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(DogBotAPI PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include
    PRIVATE src)

add_executable (exampleClient exampleClient.cc)

target_link_libraries (exampleClient LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT} ${LIBUSB_LIBRARIES})

add_executable (dumpServoState dumpServoState.cc)

target_link_libraries (dumpServoState LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

add_executable (testKinematics testKinematics.cc)

target_link_libraries (testKinematics LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

add_executable (dogBotServer dogBotServer.cc)

target_link_libraries (dogBotServer LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

#add_executable (xusb xusb.cc)
#target_link_libraries (xusb LINK_PUBLIC ${LIBUSB_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


# 'make install' to the correct locations (provided by GNUInstallDirs).

#install(TARGETS DogBotAPI EXPORT DogBotAPIConfig
#    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

#install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
#install(EXPORT DogBotAPIConfig DESTINATION share/MyLibrary/cmake)

# This makes the project importable from the build directory
export(TARGETS DogBotAPI FILE DogBotAPIConfig.cmake)

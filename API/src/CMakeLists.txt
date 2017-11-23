
cmake_minimum_required(VERSION 3.5)
project(dogbotapi LANGUAGES CXX)
#VERSION 0.1 
FIND_PACKAGE ( Threads REQUIRED )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
endif()

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

add_definitions(-std=c++11)

ADD_LIBRARY (DogBotAPI SerialComs.cc DogBotAPI.cc Servo.cc LegKinematics.cc DogBotKinematics.cc )

target_link_libraries (DogBotAPI LINK_PUBLIC ${JSONCPP_LIBRARIES} )

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(DogBotAPI PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include
    PRIVATE src)

add_executable (dtalk dogtalk.cc)

target_link_libraries (dtalk LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

add_executable (dtservo dtservo.cc)

target_link_libraries (dtservo LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

add_executable (testKin testKinematics.cc)

target_link_libraries (testKin LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})


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

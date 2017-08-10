

FIND_PACKAGE ( Threads REQUIRED )

add_definitions(-std=c++11)

ADD_LIBRARY (DogBotAPI coms.cc)

target_include_directories (DogBotAPI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../inc )

add_executable (dtalk dogtalk.cc)

target_link_libraries (dtalk LINK_PUBLIC DogBotAPI ${CMAKE_THREAD_LIBS_INIT})

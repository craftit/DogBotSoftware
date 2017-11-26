#-------------------------------------------------
#
# Project created by QtCreator 2017-08-11T13:38:59
#
#-------------------------------------------------

QT       += core gui

CONFIG += c++11
CONFIG += link_pkgconfig

QMAKE_CXXFLAGS += -std=c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

PKGCONFIG += jsoncpp spdlog

TARGET = Bark
TEMPLATE = app

SOURCES +=\
    main.cc \
    mainwindow.cc \
    ServoTable.cc

HEADERS  += \
    mainwindow.hh \
    ServoTable.hh

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../API/build/src/release/ -lDogBotAPI
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../API/build/src/debug/ -lDogBotAPI
else:unix: LIBS += -L$$PWD/../API/build/ -lDogBotAPI

INCLUDEPATH += $$PWD/../API/include /usr/local/include
DEPENDPATH += $$PWD/../API/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../API/build/src/release/libDogBotAPI.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../API/build/src/debug/libDogBotAPI.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../API/build/src/release/DogBotAPI.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../API/build/src/debug/DogBotAPI.lib
#else:unix: PRE_TARGETDEPS += $$PWD/../API/build/libDogBotAPI.a
else:unix: PRE_TARGETDEPS += $$PWD/../API/build/libDogBotAPI.a

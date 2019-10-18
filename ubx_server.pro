QT -= gui
QT += serialport
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

SOURCES += main.cpp \
    nmeaserver.cpp \
    ublox.cpp \
    rtcm3_simple.c \
    tcpbroadcast.cpp \
    basestation.cpp \
    utility.cpp \
    rover.cpp

HEADERS += \
    nmeaserver.h \
    rtcm3_simple.h \
    ublox.h \
    datatypes.h \
    tcpbroadcast.h \
    basestation.h \
    utility.h \
    rover.h


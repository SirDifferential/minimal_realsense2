TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
SOURCES += \
    main.c

INCLUDEPATH += "C:\SDL2-2.0.7\include"
LIBS += -L"C:\SDL2-2.0.7_msvc2017_64\Release" -lsdl2

INCLUDEPATH += "C:\Program Files (x86)\Intel RealSense SDK 2.0\include"
LIBS += -L"C:\Program Files (x86)\Intel RealSense SDK 2.0\lib\x64" -lrealsense2

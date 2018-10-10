TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

LIBS += -L/home/mikkel/Documents/fuzzylite-6.0/fuzzylite/release/bin -lfuzzylite-static
INCLUDEPATH += /home/mikkel/Documents/fuzzylite-6.0/fuzzylite
DEPENDPATH += /home/mikkel/Documents/fuzzylite-6.0/fuzzylite

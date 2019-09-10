#-------------------------------------------------
#
# Project created by QtCreator 2019-08-30T04:36:16
#
#-------------------------------------------------

QT       += core gui

CONFIG += qwt

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 30_08_Kod
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    mainwindow.h

FORMS += \
    mainwindow.ui

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

LIBS += -L/usr/include/opencv

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/local/qwt-6.0.1/lib/release/ -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/local/qwt-6.0.1/lib/debug/ -lqwt
else:unix: LIBS += -L$$PWD/../../../usr/local/qwt-6.0.1/lib/ -lqwt

INCLUDEPATH += $$PWD/../../../usr/local/qwt-6.0.1/include
DEPENDPATH += $$PWD/../../../usr/local/qwt-6.0.1/include


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lueye_api
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lueye_api
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lueye_api

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

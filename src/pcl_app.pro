#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET = pcl_app
TEMPLATE = app


SOURCES += main.cpp\
        pclapp.cpp \
        kinectgrabber.cpp \
        utils.cpp

HEADERS  += pclapp.h\
        kinectgrabber.h \
        utils.h

FORMS    += pclapp.ui

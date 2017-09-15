#-------------------------------------------------
#
# Project created by QtCreator 2017-09-14T22:15:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DStar
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp \
    planner.cpp \
    xmap.cpp \
    xmath.cpp

HEADERS  += dialog.h \
    planner.h \
    xmap.h \
    xmath.h

FORMS    += dialog.ui

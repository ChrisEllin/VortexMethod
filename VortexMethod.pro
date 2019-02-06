#-------------------------------------------------
#
# Project created by QtCreator 2018-04-28T15:43:42
#
#-------------------------------------------------

QT       += core gui concurrent printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vortexMethod
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    mainfield.cpp \
    vector3d.cpp \
    vorton.cpp \
    multiframe.cpp \
    fourframe.cpp \
    framecalculations.cpp \
    bodyfragmentation.cpp \
    solver.cpp \
    solversettings.cpp \
    logger.cpp \
    vector2d.cpp \
    variatesettings.cpp \
    preprocessorsettings.cpp \
    simpsonintegration.cpp

HEADERS += \
        mainwindow.h \
    mainfield.h \
    vector3d.h \
    vorton.h \
    multiframe.h \
    fourframe.h \
    framecalculations.h \
    bodyfragmentation.h \
    solver.h \
    solversettings.h \
    logger.h \
    vector2d.h \
    variatesettings.h \
    preprocessorsettings.h \
    simpsonintegration.h

FORMS += \
        mainwindow.ui \
    solversettings.ui \
    variatesettings.ui \
    preprocessorsettings.ui

RESOURCES += \
    resources.qrc


win32 {
    COPY_FROM_PATH=$$shell_path($$PWD/dll)
    COPY_TO_PATH=$$shell_path($$DESTDIR)
}
copydata.commands = $(COPY_DIR) $$COPY_FROM_PATH $$COPY_TO_PATH
first.depends = $(first) copydata

export(first.depends)
export(copydata.commands)

QMAKE_EXTRA_TARGETS += first copydata





unix|win32: LIBS += -L$$PWD/dll/ -llibdrawstuff.dll

INCLUDEPATH += $$PWD/dll
DEPENDPATH += $$PWD/dll

unix|win32: LIBS += -L$$PWD/dll/ -llibode_single.dll

INCLUDEPATH += $$PWD/dll
DEPENDPATH += $$PWD/dll


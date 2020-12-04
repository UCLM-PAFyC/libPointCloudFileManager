# Cambio desde QtCreator DGS instalacion GIT
#-------------------------------------------------
#
# Project created by QtCreator 2020-03-25T09:25:54
#
#-------------------------------------------------
debug_and_release {
    CONFIG -= debug_and_release
    CONFIG += debug_and_release
}

# ensure one "debug" or "release" in CONFIG so they can be used as
# conditionals instead of writing "CONFIG(debug, debug|release)"...
CONFIG(debug, debug|release) {
    CONFIG -= debug release
    CONFIG += debug
    }
CONFIG(release, debug|release) {
        CONFIG -= debug release
        CONFIG += release
}

QT       += widgets sql xml concurrent

TARGET = libPointCloudFileManager
TEMPLATE = lib

DEFINES += LIBPOINTCLOUDFILEMANAGER_LIBRARY

DESTDIR_RELEASE=C:\dev\release
DESTDIR_DEBUG=C:\dev\debug
#OSGEO4W_PATH="C:\Program Files\QGIS 2.18"
OSGEO4W_PATH="C:\Program Files\QGIS 3.4"
LASTOOLS_PATH = E:\Qt5CreatorPrograms\TIDOP_TOOLS_USAL\libs\LASTools2019
QUAZIPLIB_PATH="E:\Qt5CreatorPrograms\TIDOP_TOOLS_USAL\libs\libQuaZip"

SOURCES += \
    PointCloudFileManager.cpp \
    PointCloudFile.cpp \
    Point.cpp

HEADERS += \
    libPointCloudFileManager_global.h \
    PointCloudFileManager.h \
    PointCloudFileDefinitions.h \
    PointCloudFile.h \
    Point.h

INCLUDEPATH += "C:\Qt\Qt5.6.3\5.6.3\Src\qtbase\src\3rdparty\zlib"
INCLUDEPATH += . $$QUAZIPLIB_PATH/include

INCLUDEPATH += . ../libProcessTools
INCLUDEPATH += . ../libCRS
INCLUDEPATH += . ../libIGDAL
INCLUDEPATH += . ../libParameters
INCLUDEPATH += ../
#INCLUDEPATH += . ../libProcessTools
INCLUDEPATH += $$LASTOOLS_PATH\LASlib\inc
INCLUDEPATH += $$LASTOOLS_PATH\LASzip\src

debug{
    DESTDIR = $$DESTDIR_DEBUG
    LIBS += -L$$DESTDIR_DEBUG
    LIBS += -L$$LASTOOLS_PATH/lib64D
    LIBS += -llaslib
#    LIBS += -L$$QUAZIPLIB_PATH/lib
#    LIBS += -lquazipd
    LIBS += $$QUAZIPLIB_PATH\lib\quazipd.lib
}else{
    DESTDIR = $$DESTDIR_RELEASE
    LIBS += -L$$DESTDIR_RELEASE
    LIBS += -L$$LASTOOLS_PATH/lib64
    LIBS += -llaslib
    LIBS += $$QUAZIPLIB_PATH\lib\quazip.lib
#    LIBS += -L$$QUAZIPLIB_PATH/lib
#    LIBS += -lquazip
}

LIBS += -llibCRS
LIBS += -llibIGDAL
LIBS += -llibParameters
LIBS += -llibProcessTools
#LIBS += -llibProcessTools

INCLUDEPATH += . $$OSGEO4W_PATH/include
LIBS += -L$$OSGEO4W_PATH\bin
#LIBS += $$OSGEO4W_PATH\lib\gsl.lib
LIBS += $$OSGEO4W_PATH\lib\proj_i.lib
LIBS += $$OSGEO4W_PATH\lib\gdal_i.lib
LIBS += $$OSGEO4W_PATH\lib\geos_c.lib

#unix {
#    target.path = /usr/lib
#    INSTALLS += target
#}

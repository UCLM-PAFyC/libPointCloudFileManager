# Prueba con clave publica en github
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

#DESTDIR_RELEASE= ./../../../build/release
#DESTDIR_DEBUG= ./../../../build/debug
DESTDIR_RELEASE= ./../../../build_osgeo4w/release
DESTDIR_DEBUG= ./../../../build_osgeo4w/debug
#OSGEO4W_PATH="C:\Program Files\QGIS 2.18"
#OSGEO4W_PATH="C:\Program Files\QGIS 3.4"
OSGEO4W_PATH="E:\dev\OSGeo4Wltr"
LASTOOLS_PATH = ./../../../depends/LASTools2019
#QUAZIPLIB_PATH= ./../../../depends/libQuaZip
QUAZIPLIB_PATH= ./../../../depends/libQuaZip-1.2
#QT_3RDPARTY= C:/Qt/Qt5.6.3/5.6.3/Src/qtbase/src/3rdparty
CGAL_PATH= ./../../../depends/CGAL-5.3.1
BOOST_PATH= ./../../../depends/boost_1_76_0_vs2014_x64

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

INCLUDEPATH += \
#        $$CGAL_PATH\install\include \
        $$CGAL_PATH\include \
        $$BOOST_PATH \
        $$CGAL_PATH/auxiliary/gmp/include

#INCLUDEPATH += $$QT_3RDPARTY/zlib
INCLUDEPATH += . $$QUAZIPLIB_PATH/include

INCLUDEPATH += . ../libICGAL
INCLUDEPATH += . ../libProcessTools
INCLUDEPATH += . ../libCRS
INCLUDEPATH += . ../libIGDAL
INCLUDEPATH += . ../libParameters
INCLUDEPATH += . ../libLicenseManager
INCLUDEPATH += ../
#INCLUDEPATH += . ../libProcessTools
INCLUDEPATH += $$LASTOOLS_PATH\LASlib\inc
INCLUDEPATH += $$LASTOOLS_PATH\LASzip\src

debug{
    DESTDIR = $$DESTDIR_DEBUG
    LIBS += -L$$DESTDIR_DEBUG
    LIBS += -L$$LASTOOLS_PATH/lib64D
    LIBS += -llaslib
#    LIBS += $$QUAZIPLIB_PATH\lib\quazipd.lib
    LIBS += $$QUAZIPLIB_PATH\lib\quazip1-qt5d.lib
#    LIBS += -L$$QUAZIPLIB_PATH/lib
#    LIBS += -lquazipd
}else{
    DESTDIR = $$DESTDIR_RELEASE
    LIBS += -L$$DESTDIR_RELEASE
    LIBS += -L$$LASTOOLS_PATH/lib64
    LIBS += -llaslib
#    LIBS += $$QUAZIPLIB_PATH\lib\quazip.lib
    LIBS += $$QUAZIPLIB_PATH\lib\quazip1-qt5.lib
#    LIBS += -L$$QUAZIPLIB_PATH/lib
#    LIBS += -lquazip
}

LIBS += -llibICGAL
LIBS += -llibCRS
LIBS += -llibIGDAL
LIBS += -llibParameters
LIBS += -llibProcessTools
LIBS += -llibLicenseManager
#LIBS += -llibProcessTools

INCLUDEPATH += . $$OSGEO4W_PATH/include
LIBS += -L$$OSGEO4W_PATH\bin
#LIBS += $$OSGEO4W_PATH\lib\gsl.lib
#LIBS += $$OSGEO4W_PATH\lib\proj_i.lib
LIBS += $$OSGEO4W_PATH\lib\proj.lib
LIBS += $$OSGEO4W_PATH\lib\gdal_i.lib
LIBS += $$OSGEO4W_PATH\lib\geos_c.lib

#unix {
#    target.path = /usr/lib
#    INSTALLS += target
#}

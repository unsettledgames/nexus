QT       += core gui widgets

TARGET   = nxscompress
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++11

DEFINES += _FILE_OFFSET_BITS=64 TEXTURE
DEFINES += _USE_MATH_DEFINES

INCLUDEPATH += \
    ../../../vcglib \
    ../../../vcglib/eigenlib

win32:INCLUDEPATH += "C:/Program Files/vcpkg/packages/glew_x64-windows/lib/include" ../../../corto/include
win32:LIBS += opengl32.lib GLU32.lib "C:/Program Files/vcpkg/packages/glew_x64-windows/lib/glew32.lib" ../../../corto/Build/Release/corto.lib \
        "C:\Users\nicol\Desktop\Lavoro\Repo\corto\Build\deps\lz4\build\cmake\Release\lz4.lib"

unix:INCLUDEPATH += /usr/local/lib
unix:LIBS += -L /usr/local/lib -lcorto

SOURCES += \
    ../../../vcglib/wrap/system/qgetopt.cpp \
    ../../../vcglib/wrap/ply/plylib.cpp \
    ../common/virtualarray.cpp \
    ../common/nexusdata.cpp \
    ../common/traversal.cpp \
    ../common/cone.cpp \
    ../common/qtnexusfile.cpp \
    main_compress.cpp \
    extractor.cpp

HEADERS += \
    ../../../vcglib/wrap/system/qgetopt.h \
    ../common/virtualarray.h \
    ../common/qtnexusfile.h \
    ../common/nexusdata.h \
    ../common/traversal.h \
    ../common/signature.h \
    ../nxszip/zpoint.h \
    ../nxszip/model.h \
    ../nxszip/range.h \
    ../nxszip/fpu_precision.h \
    ../nxszip/math_class.h \
    extractor.h

DESTDIR = "../../bin"

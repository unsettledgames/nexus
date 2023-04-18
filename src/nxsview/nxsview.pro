QT       += core gui opengl widgets

TARGET   = nxsview
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++11

DEFINES += GL_COMPATIBILITY
DEFINES += NDEBUG

win32-msvc: DEFINES += NOMINMAX

unix:DEFINES -= USE_CURL
win32:DEFINES += NOMINMAX
DEFINES += _FILE_OFFSET_BITS=64
DEFINES += _USE_MATH_DEFINES

win32:INCLUDEPATH += ../../../glew/include \
    ../../../corto/include \
    ./../../../vcglib \
    ../../../vcglib/eigenlib

win32:LIBS += opengl32.lib GLU32.lib "$$PWD/../../../corto/Build/deps/lz4/build/cmake/Release/lz4.lib" \
    "C:/Program Files/vcpkg/packages/glew_x64-windows/lib/glew32.lib" \
    "./../../../corto/Build/Release/corto.lib"


unix:INCLUDEPATH += /usr/local/lib
unix:LIBS += -lGLEW -lGLU -lcorto
#-lcurl

SOURCES += \
    ../../../vcglib/wrap/gui/trackmode.cpp \
    ../../../vcglib/wrap/gui/trackball.cpp \
    ../../../vcglib/wrap/system/qgetopt.cpp \
    ../common/qtnexusfile.cpp \
    ../common/controller.cpp \
    ../common/nexus.cpp \
    ../common/cone.cpp \
    ../common/traversal.cpp \
    ../common/renderer.cpp \
    ../common/ram_cache.cpp \
    ../common/frustum.cpp \
    ../common/nexusdata.cpp \
    ../nxsedit/extractor.cpp \
    main.cpp \
    gl_nxsview.cpp \
    scene.cpp

HEADERS  += \
    ../../../vcglib/wrap/gcache/token.h \
    ../../../vcglib/wrap/gcache/provider.h \
    ../../../vcglib/wrap/gcache/door.h \
    ../../../vcglib/wrap/gcache/dheap.h \
    ../../../vcglib/wrap/gcache/controller.h \
    ../../../vcglib/wrap/gcache/cache.h \
    ../common/signature.h \
    ../common/qtnexusfile.h \
    ../common/nexus.h \
    ../common/cone.h \
    ../common/traversal.h \
    ../common/token.h \
    ../common/renderer.h \
    ../common/ram_cache.h \
    ../common/metric.h \
    ../common/gpu_cache.h \
    ../common/globalgl.h \
    ../common/frustum.h \
    ../common/dag.h \
    ../common/controller.h \
    ../common/nexusdata.h \
    ../nxszip/zpoint.h \
    gl_nxsview.h \
    scene.h


FORMS    += \
    nxsview.ui

DESTDIR = "../../bin"

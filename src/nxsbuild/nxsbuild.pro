QT       += core gui svg

TARGET   = nxsbuild
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++11 -g -fpermissive

INCLUDEPATH += \
    ../../../vcglib \
    ../../../vcglib/eigenlib \
    ../texture-defrag/texture-defrag

DEFINES += _FILE_OFFSET_BITS=64
DEFINES += _USE_MATH_DEFINES

win32-msvc: DEFINES += NOMINMAX

unix:LIBS += -lGLU

win32:LIBS += "C:/Program Files/vcpkg/packages/glew_x64-windows/lib/glew32.lib" -lopengl32 -lglu32

SOURCES += \
    ../../../vcglib/wrap/system/qgetopt.cpp \
    ../../../vcglib/wrap/ply/plylib.cpp \
    ../common/virtualarray.cpp \
    ../common/cone.cpp \
    main.cpp \
    meshstream.cpp \
    meshloader.cpp \
    plyloader.cpp \
    kdtree.cpp \
    mesh.cpp \
    tsploader.cpp \
    nexusbuilder.cpp \
    objloader.cpp \
    tmesh.cpp \
    texpyramid.cpp \
    stlloader.cpp

SOURCES += \
    texture-defrag/defrag_mesh.cpp \
    texture-defrag/intersection.cpp \
    texture-defrag/mesh_attribute.cpp \
    texture-defrag/packing.cpp \
    texture-defrag/seam_remover.cpp \
    texture-defrag/seams.cpp \
    texture-defrag/texture_optimization.cpp \
    texture-defrag/mesh_graph.cpp \
    texture-defrag/gl_utils.cpp \
    texture-defrag/texture_rendering.cpp \
    texture-defrag/logging.cpp \
    texture-defrag/matching.cpp \
    texture-defrag/arap.cpp \
    texture-defrag/shell.cpp \
    texture-defrag/texture_object.cpp \
    ../../../vcglib/wrap/qt/outline2_rasterizer.cpp


HEADERS += \
    ../../../vcglib/wrap/system/qgetopt.h \
    ../../../vcglib/wrap/ply/plylib.h \
    ../common/signature.h \
    ../common/cone.h \
    ../common/virtualarray.h \
    meshstream.h \
    meshloader.h \
    plyloader.h \
    partition.h \
    kdtree.h \
    trianglesoup.h \
    mesh.h \
    tsploader.h \
    nexusbuilder.h \
    objloader.h \
    tmesh.h \
    vertex_cache_optimizer.h \
    texpyramid.h \
    stlloader.h \
    vcgloader.h \
    vcgloadermesh.h

HEADERS += \
    texture-defrag/defrag_mesh.h \
    texture-defrag/intersection.h \
    texture-defrag/packing.h \
    texture-defrag/seam_remover.h \
    texture-defrag/seams.h \
    texture-defrag/timer.h \
    texture-defrag/types.h \
    texture-defrag/mesh_graph.h \
    texture-defrag/texture_rendering.h \
    texture-defrag/math_utils.h \
    texture-defrag/texture_optimization.h \
    texture-defrag/pushpull.h \
    texture-defrag/gl_utils.h \
    texture-defrag/mesh_attribute.h \
    texture-defrag/logging.h \
    texture-defrag/utils.h \
    texture-defrag/matching.h \
    texture-defrag/arap.h \
    texture-defrag/shell.h \
    texture-defrag/texture_object.h \
    texture-defrag/vcglib/wrap/qt/outline2_rasterizer.h

DESTDIR = "../../bin"

OTHER_FILES += \
    textures_plan.txt

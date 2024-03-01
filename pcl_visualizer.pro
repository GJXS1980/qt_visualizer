QT += concurrent opengl widgets

TARGET = mecheye_qt_visualizer
TEMPLATE = app

# Find PCL
CONFIG += link_pkgconfig
PCL_DIR = /usr/local/pcl-1.14

INCLUDEPATH += $$PCL_DIR/include/pcl-1.13 \
               /usr/local/include/opencv4 \
               /opt/mech-mind/mech-eye-sdk/include \
               /usr/local/vtk-8.2/include/vtk-8.2


LIBS += -L$$PCL_DIR/lib \
        -L/usr/local/lib \
        -L/opt/mech-mind/mech-eye-sdk/lib \
        -L/usr/local/vtk-8.2/lib

# Include directories
INCLUDEPATH += $$MECHEYEAPI_CFLAGS $$PCL_CFLAGS $$OpenCV_CFLAGS $$VTK_CFLAGS

# Library directories
LIBS += $$MECHEYEAPI_LIBS $$PCL_LIBS $$OpenCV_LIBS $$VTK_LIBS



# Source files
SOURCES += main.cpp pclviewer.cpp

QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    ../../include \
    /opt/ros/kinetic/include \
    /usr/include \
    /usr/include/eigen3 \
    /usr/local/include \
    /home/m/ws_msckf_mono/devel/include/ \

SOURCES += \
    ../../src/utils.cpp \
    ../../src/image_processor.cpp \
    ../../src/image_processor_nodelet.cpp \
    ../../src/msckf_mono.cpp \
    ../../src/msckf_mono_nodelet.cpp

DISTFILES += \
    ../../package.xml \
    ../../README.md \
    ../../CMakeLists.txt \
    ../../launch/image_processor_euroc.launch \
    ../../launch/msckf_vio_euroc.launch

HEADERS += \
    ../../include/msckf_mono/cam_state.h \
    ../../include/msckf_mono/feature.hpp \
    ../../include/msckf_mono/imu_state.h \
    ../../include/msckf_mono/math_utils.hpp \
    ../../include/msckf_mono/utils.h \
    ../../include/msckf_mono/image_processor.h \
    ../../include/msckf_mono/image_processor_nodelet.h \
    ../../include/msckf_mono/msckf_mono.h \
    ../../include/msckf_mono/msckf_mono_nodelet.h

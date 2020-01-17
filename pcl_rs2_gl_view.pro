TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        deviceinterface.cpp \
        main.cpp \
        pclinterface.cpp \
        rs2_pcl_converter.cpp \
        rs2device.cpp

HEADERS += \
    deviceinterface.h \
    format.h \
    pclinterface.h \
    rs2_pcl_converter.h \
    rs2device.h


INCLUDEPATH += /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/ni /usr/include/vtk-6.2 /usr/include/boost

LIBS += \
        # General
        -pthread \ #-lflann -lqhull \ # -lOpenNI -lOpenNI2
        -lboost_iostreams -lboost_system -lboost_thread -lboost_filesystem \
        # Realsense
        -lrealsense2 \ # -lrealsense2-gl \
        # PCL
        -lpcl_common -lpcl_octree -lpcl_io -lpcl_kdtree -lpcl_search -lpcl_sample_consensus -lpcl_filters \
        -lpcl_features -lpcl_keypoints -lpcl_surface -lpcl_registration -lpcl_segmentation -lpcl_recognition \
        -lpcl_visualization -lpcl_people -lpcl_outofcore -lpcl_tracking \
        # VTK
        -lvtkCommonCore-6.2 -lvtkalglib-6.2 -lvtkChartsCore-6.2 -lvtkDICOMParser-6.2 -lvtkDomainsChemistry-6.2 \
        -lvtkFiltersCore-6.2 -lvtkGeovisCore-6.2 \
        -lvtkImagingCore-6.2 -lvtkInfovisCore-6.2 -lvtkInteractionImage-6.2 -lvtkInteractionStyle-6.2 -lvtkInteractionWidgets-6.2 \
        -lvtkIOCore-6.2 -lvtkParallelCore-6.2 -lvtkRenderingCore-6.2 -lvtksys-6.2 \
        -lvtkCommonExecutionModel-6.2 \
        -lvtkRenderingLOD-6.2 \
        -lvtkFiltersSources-6.2 \
        -lvtkCommonDataModel-6.2 \
        -lvtkCommonMath-6.2


# PCL - non standard libs
#        -lpcl_stereo -lpcl_ml -lpcl_gpu_containers -lpcl_gpu_utils \
#        -lpcl_gpu_octree -lpcl_gpu_features -lpcl_gpu_kinfu -lpcl_gpu_kinfu_large_scale -lpcl_gpu_segmentation \
#        -lpcl_cuda_features -lpcl_cuda_segmentation -lpcl_cuda_sample_consensus \

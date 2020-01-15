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


INCLUDEPATH += /usr/local/include/pcl-1.9 /usr/include/eigen3 /usr/include/ni /usr/include/vtk-6.3 /usr/include/boost

LIBS += \
        # General
        -pthread \#-lflann -lqhull \ # -lOpenNI -lOpenNI2
        -lboost_iostreams -lboost_system -lboost_thread -lboost_filesystem \
        # Realsense
        -lrealsense2 -lrealsense2-gl \
        # PCL
        -lpcl_common -lpcl_octree -lpcl_io -lpcl_kdtree -lpcl_search -lpcl_sample_consensus -lpcl_filters \
        -lpcl_features -lpcl_keypoints -lpcl_surface -lpcl_registration -lpcl_ml -lpcl_segmentation -lpcl_recognition \
        -lpcl_visualization -lpcl_people -lpcl_outofcore -lpcl_tracking -lpcl_stereo -lpcl_gpu_containers -lpcl_gpu_utils \
        -lpcl_gpu_octree -lpcl_gpu_features -lpcl_gpu_kinfu -lpcl_gpu_kinfu_large_scale -lpcl_gpu_segmentation \
        -lpcl_cuda_features -lpcl_cuda_segmentation -lpcl_cuda_sample_consensus \
        # VTK
        -lvtkCommonCore-6.3 -lvtkalglib-6.3 -lvtkChartsCore-6.3 -lvtkDICOMParser-6.3 -lvtkDomainsChemistry-6.3 \
        -lvtkFiltersCore-6.3 -lvtkGeovisCore-6.3 \
        -lvtkImagingCore-6.3 -lvtkInfovisCore-6.3 -lvtkInteractionImage-6.3 -lvtkInteractionStyle-6.3 -lvtkInteractionWidgets-6.3 \
        -lvtkIOCore-6.3 -lvtkParallelCore-6.3 -lvtkRenderingCore-6.3 -lvtksys-6.3 \
        -lvtkCommonExecutionModel-6.3 \
        -lvtkRenderingLOD-6.3 \
        -lvtkFiltersSources-6.3 \
        -lvtkCommonDataModel-6.3 \
        -lvtkCommonMath-6.3

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        customtypes.cpp \
        deviceinterface.cpp \
        main.cpp \
        ocvdevice.cpp \
        pclinterface.cpp \
        rs2_pcl_converter.cpp \
        rs2device.cpp

HEADERS += \
    b.h \
    customtypes.h \
    deviceinterface.h \
    format.h \
    ocvdevice.h \
    pclinterface.h \
    rs2_pcl_converter.h \
    rs2device.h

CONFIG += UBU18 # UBU16


UBU18 {
INCLUDEPATH += /usr/include/pcl-1.8 /usr/include/eigen3 /usr/include/ni /usr/include/vtk-6.3 /usr/include/boost /usr/local/include/opencv4
LIBS += \
        # General
        -pthread \ #-lflann -lqhull \ # -lOpenNI -lOpenNI2
        -lboost_iostreams -lboost_system -lboost_thread -lboost_filesystem \
        # Realsense
        -lrealsense2 \#-lrealsense2-gl \
        # PCL
        -lpcl_common -lpcl_octree -lpcl_io -lpcl_kdtree -lpcl_search -lpcl_sample_consensus -lpcl_filters \
        -lpcl_features -lpcl_keypoints -lpcl_surface -lpcl_registration -lpcl_segmentation -lpcl_recognition \
        -lpcl_visualization -lpcl_people -lpcl_outofcore -lpcl_tracking -lpcl_stereo -lpcl_ml \
        # OpenCV
        -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_videoio -lopencv_tracking \
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
        # PCL - non standard libs
#        -lpcl_gpu_octree -lpcl_gpu_containers -lpcl_gpu_utils \
#        -lpcl_gpu_features -lpcl_gpu_kinfu -lpcl_gpu_kinfu_large_scale -lpcl_gpu_segmentation \
#        -lpcl_cuda_features -lpcl_cuda_segmentation -lpcl_cuda_sample_consensus
}

UBU16 {
INCLUDEPATH += /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/ni /usr/include/vtk-6.2 /usr/include/boost /usr/local/include/opencv4

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
        # OpenCV
        -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_videoio -lopencv_tracking \
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
}

#ifndef FORMAT_H
#define FORMAT_H

#include <iomanip>

// Verbosity level
#define VERBOSE 2
// Global settings
#define PROC_PIPE_PC_ENABLED 0
#define PROC_PIPE_MAT_ENABLED 1
// Makros
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)
#define areSameD(first, second) (fabs(first - second) < std::numeric_limits<double>::epsilon())
#define areSameF(first, second) (fabs(first - second) < std::numeric_limits<float>::epsilon())

// Realsense acquisition
#define RS_CENTRAL_SERIAL "950122060662"
#define RS_FRONT_SERIAL "950122060486"
#define RS_REAR_SERIAL "950122061070"

#define RS_MASTER_SLAVE_CONF_ENABLED 1
#define RS_DEPTH_ENABLED 1
#define RS_COLOR_ENABLED 0
#define RS_EMITTER_ENABLED 1.f
#define RS_EMITTER_POWER 1.f

#define FRAME_WIDTH_DEPTH 1280 // 848
#define FRAME_HEIGHT_DEPTH 720 // 480
#define FRAME_RATE_DEPTH 6 // 6,15,30,60,90

#define FRAME_PERIOD_MS 1000/FRAME_RATE_DEPTH
#define FRAME_PERIOD_NS 1000000000/FRAME_RATE_DEPTH
#define FRAME_DATA_LENGTH FRAME_WIDTH_DEPTH*FRAME_HEIGHT_DEPTH
#define FRAME_DATA_SIZE 3*FRAME_DATA_LENGTH




// Converter
#define CONV_RS_ROTATION_AXES "yxz"

#define TRAN_FRONT_TO_CENTRAL_X_M 0.0
#define TRAN_FRONT_TO_CENTRAL_Y_M 0.0//-0.020
#define TRAN_FRONT_TO_CENTRAL_Z_M 0.0//-0.010
#define ROT_FRONT_TO_CENTRAL_ANG_X 0.0
#define ROT_FRONT_TO_CENTRAL_ANG_Y -45.0//-45.0
#define ROT_FRONT_TO_CENTRAL_ANG_Z 0.0
#define TRAN_REAR_TO_CENTRAL_X_M 0.0
#define TRAN_REAR_TO_CENTRAL_Y_M 0.0//0.020
#define TRAN_REAR_TO_CENTRAL_Z_M 0.0//-0.010
#define ROT_REAR_TO_CENTRAL_ANG_X 0.0
#define ROT_REAR_TO_CENTRAL_ANG_Y 45.0//45.0
#define ROT_REAR_TO_CENTRAL_ANG_Z 0.0

#define CONV_THREAD_POOL_SIZE 7
#define CONV_SPLIT_DATA 0 // not yet implemented
// PCL processing
#define PCL_CLOUD_ORGANIZED 1



// Buffer sizes
#define QUE_SIZE_RS2FRAMES 10
#define QUE_SIZE_CLOUDS 10
#define BUF_SIZE_MATS 10
#define QUE_SIZE_SEG 10
#define BUF_SIZE_CVCAP 10
#define BUF_SIZE_TOBJ 10
#define BUF_SIZE_VEL CV_FRAME_RATE*2 // x, y pairs
// Thread poll delays
#define DELAY_CONVERTER_NS FRAME_PERIOD_NS/2/3
#define DELAY_PROCINTERFACE_NS FRAME_PERIOD_NS/2/3
#define DELAY_PROC_SEGM_NS FRAME_PERIOD_NS/2

// Thread idle delays
#define DELAY_SEGM CV_FRAME_PERIOD_MS/2
#define DELAY_TRAC CV_FRAME_PERIOD_MS/2
#define DELAY_SHOW CV_FRAME_PERIOD_MS/2

// OpenCV acquisition
#define CV_FRAME_WIDTH 1280  //1600
#define CV_FRAME_HEIGHT 720 //1200
#define CV_FRAME_RATE 10 //5
#define CV_FRAME_PERIOD_MS 1000/CV_FRAME_RATE
#define CV_REF_PIXEL 60
#define CV_REF_SIZE_MM 26

// Toggle viewers
#define IMSHOW_CV 1

// Predefined sizes
#define SIZE_XYZ 3
#define SIZE_RGB 3







#endif // FORMAT_H

#ifndef FORMAT_H
#define FORMAT_H

// Verbosity level
#define VERBOSE 0
// Makros
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

// Realsense acquisition
#define RS_FRAME_WIDTH 1280 //1280 //640
#define RS_FRAME_HEIGHT 720 //720 //480
#define RS_FRAME_RATE 30 //15 //30

#define RS_FRAME_PERIOD_MS 1000/RS_FRAME_RATE
#define RS_FRAME_PERIOD_NS 1000000/RS_FRAME_RATE

#define RS_FRAME_POINTS_COUNT RS_FRAME_WIDTH*RS_FRAME_HEIGHT

#define RS_EMITTER_ENABLED 1.f
#define RS_EMITTER_POWER 1.f

#define RS_MASTER_SLAVE_CONFIG 1
#define RS_FILTER_FRAMES 0

#define RS_DEPTH_ENABLED 0
#define RS_COLOR_ENABLED 1
// OpenCV acquisition
#define CV_FRAME_WIDTH 1280  //1600
#define CV_FRAME_HEIGHT 720 //1200
#define CV_FRAME_RATE 10 //5
#define CV_FRAME_PERIOD_MS 1000/CV_FRAME_RATE
#define CV_REF_PIXEL 60
#define CV_REF_SIZE_MM 26
// Converter
#define CONV_SPLIT_DATA 0

/*
  opencv cam: x,y (2d) same direction as central rs2, but after conversion to 3d point cs is in cam center
          rotation and translation will change later
  central rs2 camera: +z directing towards conveyor, +y along the conveyor movement
  other rs2 cams: rotation of z and y around x by a given angle + translation
  opencv cam: x,y (2d) same direction as central rs2, but after conversion to 3d point cs is in cam center
            rotation and translation will change later
*/
// RS0 central
#define RS_CENTRAL_SERIAL "950122060662"
#define TRAN_RS0_TO_CV0_X_M 3.0
#define TRAN_RS0_TO_CV0_Y_M 0.0
#define TRAN_RS0_TO_CV0_Z_M 0.1
#define ROT_RS0_TO_CV0_X_ANG 0.0
//#define ROT_RS0_TO_CV0_Y_ANG 0.0
//#define ROT_RS0_TO_CV0_Z_ANG 0.0
// RS1 front
#define RS_FRONT_SERIAL "950122060486"
#define TRAN_RS1_TO_RS0_X_M 0.0
#define TRAN_RS1_TO_RS0_Y_M -0.020
#define TRAN_RS1_TO_RS0_Z_M -0.010
//#define ROT_RS1_TO_RS0_X_ANG -0.090
#define ROT_RS1_TO_RS0_X_ANG -90.0
//#define ROT_RS1_TO_RS0_Y_ANG 0.0
//#define ROT_RS1_TO_RS0_Z_ANG 0.0
// RS2 rear
#define RS_REAR_SERIAL "950122061070"
#define TRAN_RS2_TO_RS0_X_M 0.0
#define TRAN_RS2_TO_RS0_Y_M 0.020
#define TRAN_RS2_TO_RS0_Z_M -0.010
//#define ROT_RS2_TO_RS0_X_ANG 0.090
#define ROT_RS2_TO_RS0_X_ANG 90.0
//#define ROT_RS2_TO_RS0_Y_ANG 0.0
//#define ROT_RS2_TO_RS0_Z_ANG 0.0



// Buffer sizes
#define QUE_SIZE_RS2FRAMES 100
#define QUE_SIZE_PCL 100
#define BUF_SIZE_CLOUDS 100
#define BUF_SIZE_MATS 100
#define BUF_SIZE_CVCAP 100
#define BUF_SIZE_TOBJ 100
#define BUF_SIZE_VEL CV_FRAME_RATE*2 // x, y pairs
// Thread poll delays
#define DELAY_CONV_POLL_NS 10
#define DELAY_PCL_POLL_NS 100
#define DELAY_PCL_VIEW_NS RS_FRAME_PERIOD_NS/2
// Thread idle delays
#define DELAY_SEGM CV_FRAME_PERIOD_MS/2
#define DELAY_TRAC CV_FRAME_PERIOD_MS/2
#define DELAY_SHOW CV_FRAME_PERIOD_MS/2
// Toggle viewers
#define PCL_VIEWER 0
#define IMSHOW_CV 1





#endif // FORMAT_H

#ifndef FORMAT_H
#define FORMAT_H

// Verbosity level
#define VERBOSE 3
// Makros
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

// Realsense acquisition
#define FRAME_WIDTH_RS 1280 //1280 //640
#define FRAME_HEIGHT_RS 720 //720 //480
#define FRAME_RATE_RS 30 //15 //30
#define FRAME_PERIOD_RS_MS 1000/FRAME_RATE_RS
#define FRAME_POINTS_COUNT_RS FRAME_WIDTH_RS*FRAME_HEIGHT_RS
// OpenCV acquisition
#define FRAME_WIDTH_CV 1280  //1600
#define FRAME_HEIGHT_CV 720 //1200
#define FRAME_RATE_CV 10 //5
#define FRAME_PERIOD_CV_MS 1000/FRAME_RATE_CV
// Hardware parameters
#define REF_PIXEL 60
#define REF_SIZE_MM 26

/*
  opencv cam: x,y (2d) same direction as central rs2, but after conversion to 3d point cs is in cam center
          rotation and translation will change later
  central rs2 camera: +z directing towards conveyor, +y along the conveyor movement
  other rs2 cams: rotation of z and y around x by a given angle + translation
  opencv cam: x,y (2d) same direction as central rs2, but after conversion to 3d point cs is in cam center
            rotation and translation will change later
*/
// RS0 central
#define RS0_CENTRAL_SERIAL "#950122060662"
#define TRAN_RS0_TO_CV0_X_M 3.0
#define TRAN_RS0_TO_CV0_Y_M 0.0
#define TRAN_RS0_TO_CV0_Z_M 0.1
#define ROT_RS0_TO_CV0_X_ANG 0.0
//#define ROT_RS0_TO_CV0_Y_ANG 0.0
//#define ROT_RS0_TO_CV0_Z_ANG 0.0
// RS1 front
#define RS1_FRONT_SERIAL "#950122060486"
#define TRAN_RS1_TO_RS0_X_M 0.0
#define TRAN_RS1_TO_RS0_Y_M -0.020
#define TRAN_RS1_TO_RS0_Z_M -0.010
#define ROT_RS1_TO_RS0_X_ANG -0.030
//#define ROT_RS1_TO_RS0_Y_ANG 0.0
//#define ROT_RS1_TO_RS0_Z_ANG 0.0
// RS2 rear
#define RS2_REAR_SERIAL "#950122061070"
#define TRAN_RS2_TO_RS0_X_M 0.0
#define TRAN_RS2_TO_RS0_Y_M 0.020
#define TRAN_RS2_TO_RS0_Z_M -0.010
#define ROT_RS2_TO_RS0_X_ANG 0.030
//#define ROT_RS2_TO_RS0_Y_ANG 0.0
//#define ROT_RS2_TO_RS0_Z_ANG 0.0



// Buffer sizes
#define BUF_SIZE_POINTS 100
#define BUF_SIZE_CLOUDS 100
#define BUF_SIZE_MATS 100
#define BUF_SIZE_CVCAP 100
#define BUF_SIZE_TOBJ 100
#define BUF_SIZE_VEL FRAME_RATE_CV*2 // x, y pairs
// Thread idle delays
#define DELAY_CONV FRAME_PERIOD_RS_MS/2
#define DELAY_PROC FRAME_PERIOD_RS_MS/2
#define DELAY_SEGM FRAME_PERIOD_CV_MS/2
#define DELAY_TRAC FRAME_PERIOD_CV_MS/2
#define DELAY_SHOW FRAME_PERIOD_CV_MS/2
// Toggle viewers
#define PCL_VIEWER 1
#define IMSHOW 1





#endif // FORMAT_H

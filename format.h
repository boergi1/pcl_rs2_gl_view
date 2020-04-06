#ifndef FORMAT_H
#define FORMAT_H

// Verbosity level
#define VERBOSE 2
// Realsense acquisition
#define FRAME_WIDTH_RS 1280 //1280 //640
#define FRAME_HEIGHT_RS 720 //720 //480
#define FRAME_RATE_RS 15 //15 //30
#define FRAME_PERIOD_RS_MS 1000/FRAME_RATE_RS
// OpenCV acquisition
#define FRAME_WIDTH_CV 1280  //1600
#define FRAME_HEIGHT_CV 720 //1200
#define FRAME_RATE_CV 10 //5
#define FRAME_PERIOD_CV_MS 1000/FRAME_RATE_CV
// Hardware parameters
#define REF_PIXEL 60
#define REF_SIZE_MM 26
#define DIST_RS2_CV_X_MM 3000
#define DIST_RS2_CV_Y_MM 0
#define DIST_RS2_CV_Z_MM 100
#define DIST_RS2_CONV 1000


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
//#define IMSHOW_CAP 0
//#define IMSHOW_SEG 0
//#define IMSHOW_TRA 0



#endif // FORMAT_H

#ifndef FORMAT_H
#define FORMAT_H

// Verbosity level
#define VERBOSE 1
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

#define RS_FRAME_WIDTH_DEPTH 1280 //1280 //640
#define RS_FRAME_HEIGHT_DEPTH 720 //720 //480
#define RS_FRAME_RATE_DEPTH 30 //15 //30

#define RS_FRAME_PERIOD_MS 1000/RS_FRAME_RATE_DEPTH
#define RS_FRAME_PERIOD_NS 1000000/RS_FRAME_RATE_DEPTH
#define RS_FRAME_POINTS_COUNT RS_FRAME_WIDTH_DEPTH*RS_FRAME_HEIGHT_DEPTH

// Realsense frame filters
#define RS_FILTER_FRAMES_ENABLED 1      // Toggle filters
#define RS_FILTER_DECIMATION_ENABLED 0  // Decimation - reduces depth frame density
#define RS_FILTER_THRESHOLD_ENABLED 1   // Threshold - removes values outside recommended range
#define RS_FILTER_SPATIAL_ENABLED 1     // Spatial - edge-preserving spatial smoothing
#define RS_FILTER_HOLEFILL_ENABLED 0    // Hole filling - Rectify missing data

#if RS_FILTER_FRAMES_ENABLED
#if RS_FILTER_DECIMATION_ENABLED
#define RS_FILTER_DEC_MAG 2.0f // (min: 1, max: 8, step: 1)
#endif
#if RS_FILTER_THRESHOLD_ENABLED
#define RS_FILTER_THR_MIN 0.4f // (min: 0, max: 16, step: 0.1)
#define RS_FILTER_THR_MAX 8.0f // (min: 0, max: 16, step: 0.1)
#endif
#if RS_FILTER_SPATIAL_ENABLED
#define RS_FILTER_SPA_MAG 2.0f // (min: 1, max: 5, step: 1)
#define RS_FILTER_SPA_ALPHA 0.50f // (min: 0.25, max: 1, step: 0.01)
#define RS_FILTER_SPA_DELTA 20.0f // (min: 1, max: 50, step: 1)
#define RS_FILTER_SPA_HOLE 3.0f // [0-5] range mapped to [none,2,4,8,16,unlimited] pixels
#endif
#if RS_FILTER_HOLEFILL_ENABLED
#define RS_FILTER_HOLE_MODE 0 // 0: fill_from_left; 1: farest_from_around; 2: nearest_from_around
#endif
#endif

// Converter
#define CONV_RS_ROTATION_AXIS "y" // todo: init t matrix in converter constructor
#define TRAN_FRONT_TO_CENTRAL_X_M 0.0
#define TRAN_FRONT_TO_CENTRAL_Y_M -0.020
#define TRAN_FRONT_TO_CENTRAL_Z_M -0.010
#define ROT_FRONT_TO_CENTRAL_ANG -90.0
#define TRAN_REAR_TO_CENTRAL_X_M 0.0
#define TRAN_REAR_TO_CENTRAL_Y_M 0.020
#define TRAN_REAR_TO_CENTRAL_Z_M -0.010
#define ROT_REAR_TO_CENTRAL_ANG 90.0

#define CONV_THREAD_POOL_SIZE 7
#define CONV_SPLIT_DATA 0 // not yet implemented
// PCL processing
#define PCL_FILTER_GLOBAL_REGION_ENABLED 1
#if PCL_FILTER_GLOBAL_REGION_ENABLED
#define PCL_GLOBAL_REGION_X_MIN_M -1.0 // +-
#define PCL_GLOBAL_REGION_X_MAX_M 1.0 // +-
#define PCL_GLOBAL_REGION_Y_MIN_M -2.0 // +-
#define PCL_GLOBAL_REGION_Y_MAX_M 2.0 // +-
#define PCL_GLOBAL_REGION_Z_MIN_M 0.0
#define PCL_GLOBAL_REGION_Z_MAX_M 3.0
#endif

#define PCL_FILTER_PLANE_ENABLED 1
#if PCL_FILTER_PLANE_ENABLED
#define PCL_FILTER_PLANE_THRES 0.01
#define PCL_FILTER_PLANE_TOL_ANGLE 10.0
#endif

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

// OpenCV acquisition
#define CV_FRAME_WIDTH 1280  //1600
#define CV_FRAME_HEIGHT 720 //1200
#define CV_FRAME_RATE 10 //5
#define CV_FRAME_PERIOD_MS 1000/CV_FRAME_RATE
#define CV_REF_PIXEL 60
#define CV_REF_SIZE_MM 26

// Toggle viewers
#define PCL_VIEWER 1
#define IMSHOW_CV 1

#define GL_DRAW_POINTCLOUD 0
#define GL_DRAW_MOSAIC 0







#endif // FORMAT_H

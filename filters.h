#ifndef FILTERS_H
#define FILTERS_H

// Global region filters
#define GLOBAL_REGION_X_MIN_M -1.5f
#define GLOBAL_REGION_X_MAX_M 1.5f
#define GLOBAL_REGION_Y_MIN_M -0.5f
#define GLOBAL_REGION_Y_MAX_M 0.5f
#define GLOBAL_REGION_Z_MIN_M 0.10f
#define GLOBAL_REGION_Z_MAX_M 1.0f


// Realsense frame filters
#define FILTER_DEPTH_RS_ENABLED 1      // Toggle filters

#if FILTER_DEPTH_RS_ENABLED
#define RS_FILTER_DECIMATION_ENABLED 0  // Decimation - reduces depth frame density
#define RS_FILTER_THRESHOLD_ENABLED 1   // Threshold - removes values outside recommended range
#define RS_FILTER_SPATIAL_ENABLED 0     // Spatial - edge-preserving spatial smoothing
#define RS_FILTER_HOLEFILL_ENABLED 0    // Hole filling - Rectify missing data

#define RS_FILTER_DEC_MAG 2.0f // (min: 1, max: 8, step: 1)
#define RS_FILTER_THR_MIN 0.4f // (min: 0, max: 16, step: 0.1)
#define RS_FILTER_THR_MAX 2.0f // (min: 0, max: 16, step: 0.1)
#define RS_FILTER_SPA_MAG 2.0f // (min: 1, max: 5, step: 1)
#define RS_FILTER_SPA_ALPHA 0.50f // (min: 0.25, max: 1, step: 0.01)
#define RS_FILTER_SPA_DELTA 20.0f // (min: 1, max: 50, step: 1)
#define RS_FILTER_SPA_HOLE 3.0f // [0-5] range mapped to [none,2,4,8,16,unlimited] pixels
#define RS_FILTER_HOLE_MODE 0 // 0: fill_from_left; 1: farest_from_around; 2: nearest_from_around
#endif



// PCL filters
#define PCL_FILTER_GLOBAL_REGION_ENABLED 0

#define PCL_FILTER_PLANE_ENABLED 0
#if PCL_FILTER_PLANE_ENABLED
#define PCL_FILTER_PLANE_THRES 0.01
#define PCL_FILTER_PLANE_TOL_ANGLE 10.0
#endif

#endif // FILTERS_H

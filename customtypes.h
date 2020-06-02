#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H


#include <queue>
#include <mutex>
#include <cstdint>
#include <math.h>

#include <iostream>


typedef struct camera_intrinsics_s {
    float fx, fy, cx, cy;       // camera coeffs
    float k1, k2, p1, p2, k3;   // distortion coeffs
    float fov_x, fov_y;         // field of view
    camera_intrinsics_s()
    {

    }
    camera_intrinsics_s(float focal_x, float focal_y, float center_x, float center_y)
    {
        fx = focal_x; fy = focal_y; cx = center_x; cy = center_y;
    }
} camera_intrinsics_t;

typedef struct camera_extrinsics_s {
    float* R;
    float* T;
    camera_extrinsics_s(float rotation[9], float translation[3])
    {
        R = rotation; T = translation;
        std::cout << std::endl << "DEBUG Extrinsics STRUCT" << std::endl
                  << R[0] << " " << R[1] << " " << R[2] << " | " << T[0] << std::endl
                  << R[3] << " " << R[4] << " " << R[5] << " | " << T[1] << std::endl
                  << R[6] << " " << R[7] << " " << R[8] << " | " << T[2] <<  std::endl << std::endl;
    }
    camera_extrinsics_s()
    {

    }
} camera_extrinsics_t;

typedef struct tracked_object_s {
    int x;
    int y;
    int w;
    int h;
    int area;
    int unique_id;
    int lost_ctr;
    int seen_ctr;
    double cx;
    double cy;
    double vx;
    double vy;
} tracked_object_t;

typedef struct shared_objects_s
{
    std::queue<tracked_object_t*> tobj_ptr_queue;
    std::queue<size_t> arr_size_queue;
    uint64_t flag;
    std::mutex mutex;
} shared_objects_t;

typedef enum
{
    CENTRAL = 0,
    FRONT,
    REAR,
    OTHER
} CameraType_t;

typedef enum
{
    BACKGROUND = 0,
    UNIDENTIFIED,
    OBJECTS
} LabelType_t;

typedef struct shared_references_s
{
    void* buf_ref;
    std::mutex* mtx_ref;
    size_t* w_idx_ref;
    size_t* r_idx_ref;
    shared_references_s()
    {
        buf_ref = nullptr; mtx_ref = nullptr; w_idx_ref = nullptr; r_idx_ref = nullptr;
    }
    shared_references_s(void* buf, std::mutex* mtx, size_t* w, size_t* r)
    {
        buf_ref = buf; mtx_ref = mtx; w_idx_ref = w; r_idx_ref = r;
    }
} shared_references_t;

//typedef struct rs2_references_s : shared_references_t
//{
//    Rs2Position_t pos_type;
//    rs2_references_s()
//    {
//        buf_ref = nullptr; mtx_ref = nullptr; w_idx_ref = nullptr; r_idx_ref = nullptr;
//    }
//    rs2_references_s(void* buf, std::mutex* mtx, size_t* w, size_t* r, Rs2Position_t pos)
//    {
//        buf_ref = buf; mtx_ref = mtx; w_idx_ref = w; r_idx_ref = r; pos_type = pos;
//    }
//} rs2_references_t;

// helper functions
std::string rs2PositionToString(CameraType_t pos);

int roundToInt(double val);

//bool areSameD(double a, double b);

//bool areSameF(float a, float b);



#endif // CUSTOMTYPES_H

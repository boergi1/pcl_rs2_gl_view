#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H


#include <queue>
#include <mutex>
#include <cstdint>
#include <math.h>


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
} Rs2Position_t;

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

typedef struct rs2_references_s : shared_references_t
{
    Rs2Position_t pos_type;
    rs2_references_s()
    {
        buf_ref = nullptr; mtx_ref = nullptr; w_idx_ref = nullptr; r_idx_ref = nullptr;
    }
    rs2_references_s(void* buf, std::mutex* mtx, size_t* w, size_t* r, Rs2Position_t pos)
    {
        buf_ref = buf; mtx_ref = mtx; w_idx_ref = w; r_idx_ref = r; pos_type = pos;
    }
} rs2_references_t;






// helper functions
std::string rs2PositionToString(Rs2Position_t pos);

int roundToInt(double val);

bool areSameD(double a, double b);

bool areSameF(float a, float b);



#endif // CUSTOMTYPES_H

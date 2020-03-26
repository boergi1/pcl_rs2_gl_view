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
    double cx;
    double cy;
    int unique_id;
    int lost_ctr;
} tracked_object_t;

typedef struct shared_data_s
{
    std::queue<tracked_object_t*> tobj_ptr_queue;
    std::queue<size_t> arr_size_queue;
    uint64_t flag;
    std::mutex mutex;
} shared_data_t;




// helper functions
int roundToInt(double val);

bool areSame(double a, double b);



#endif // CUSTOMTYPES_H

#ifndef B_H
#define B_H

#include <iostream>
#include <queue>
#include <mutex>
/* CUSTOM TYPES */

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

typedef struct shared_objects_s
{
    std::queue<tracked_object_t*> queue;
    uint64_t some_flags_to_keep_track_of_stuff;
    std::mutex mutex;
} shared_stuff_between_threads_t;


void t1_producer (shared_stuff_between_threads_t* b)
{
    tracked_object_t* tmp;
    while(1)
    {
        tmp = new tracked_object_t();
        //tmp->h = ...
        //tmp->cx = ...
        b->mutex.lock();
        b->queue.push(tmp);
        b->mutex.unlock();
       // nanosleep(10);
    }
}

void t2_consumer (shared_stuff_between_threads_t* b)
{
    while(1)
    {
        b->mutex.lock();
        if ( !b->queue.empty() )
        {
            std::cout << ' ' << b->queue.front();
            b->queue.pop();
            // queue::pop() ->
            // Removes the next element in the queue, effectively reducing its size by one.
            // The element removed is the "oldest" element in the queue whose value can be retrieved by calling member queue::front.
            // !!This calls the removed element's destructor.
        }
        b->mutex.unlock();
      //  nanosleep(1);
    }
}

#endif // B_H

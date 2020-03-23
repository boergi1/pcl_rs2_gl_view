#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <cstdint>
#include <vector>

struct tracked_object {
    int x;
    int y;
    int w;
    int h;
    int area;
    double cx;
    double cy;
    int unique_id;
    int lost_ctr;
};

class TrackedObjects
{
private:
   // std::vector<tracked_object> *m_obj_ptr = new std::vector<tracked_object>();
    tracked_object *m_obj_ptr;
    int m_size = 0;
public:
    TrackedObjects (tracked_object* obj_ptr = nullptr, int size = 0)
    {
        m_obj_ptr = obj_ptr;
        m_size = size;
    }
    ~TrackedObjects ()
    {
        delete m_obj_ptr;
    }
    tracked_object* getTrackedObjectsPtr()
    {
        return m_obj_ptr;
    }
    int getSize()
    {
        return m_size;
    }
};

class TrackedObjectArr
{
private:
    tracked_object* m_obj_ptr_arr = new tracked_object();
    int m_object_count;
public:
    TrackedObjectArr (int o_cnt = 0, tracked_object* tr_obj_ptr_arr = nullptr)
    {
        m_object_count = o_cnt;
        m_obj_ptr_arr = tr_obj_ptr_arr;
    }
    ~TrackedObjectArr ()
    {
        delete m_obj_ptr_arr;
    }
    tracked_object* getTrackedObjects()
    {
        return m_obj_ptr_arr;
    }
    int getSize()
    {
        return m_object_count;
    }
};

// helper functions
int roundToInt(double val);



#endif // CUSTOMTYPES_H

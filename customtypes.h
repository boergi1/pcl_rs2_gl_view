#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#endif // CUSTOMTYPES_H

struct tracked_object {
    int x;
    int y;
    int w;
    int h;
    int area;
    double cx;
    double cy;
};

class TrackedObject
{
private:
    tracked_object* m_obj_ptr_arr;
    int m_object_count;
public:
    TrackedObject (int o_cnt = 0, tracked_object* tr_obj_ptr_arr = nullptr)
    {
        m_object_count = o_cnt;
        m_obj_ptr_arr = tr_obj_ptr_arr;
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

#ifndef RS2DEVICE_H
#define RS2DEVICE_H

#include <iostream>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/hpp/rs_frame.hpp>

#include <opencv2/core/core.hpp>

#include "format.h"
#include "customtypes.h"

typedef enum
{
    DEFAULT = 0,
    MASTER,
    SLAVE
} SyncType_t;

class FrameQueue
{
    std::string m_name;
public:
    FrameQueue(CameraType_t CameraType, std::string name){
        m_camtype = CameraType;
        m_name = name;
    }

    void addFrame(rs2::frame frame)
    {
        frame.keep();
        m_mtx.lock();
        m_fqueue.push_back(frame);
        if (m_fqueue.size() > QUE_SIZE_RS2FRAMES)
        {
            std::cerr << "(FrameQueue) Too many frames in queue " << m_camtype << " " << m_name << std::endl;
            m_fqueue.pop_front();
        }
        m_mtx.unlock();
    }
    rs2::frame getFrame()
    {
        if (m_fqueue.size())
        {
            m_mtx.lock();
            auto frame = m_fqueue.front();
            m_fqueue.pop_front();
            m_mtx.unlock();
            return frame;
        }
        else
        {
            std::cerr << "(FrameQueue) is empty " << m_camtype << " " << m_name << std::endl;
            return *m_fqueue.end();
        }
    }
    const rs2::frame& readFrame()
    {
        if (m_fqueue.size())
        {
            const rs2::frame& frame = m_fqueue.front();
            return frame;
        }
        else
        {
            std::cerr << "(FrameQueue) is empty" << std::endl;
            return *m_fqueue.end();
        }
    }
    void setIntrinsics(camera_intrinsics_t intr) { m_intrinsics = intr; }
    camera_intrinsics_s* getIntrinsics() { return &m_intrinsics; }
    bool isEmpty() { return m_fqueue.size() == 0; }
    bool size() { return m_fqueue.size(); }
    CameraType_t getCameraType() { return m_camtype; }
private:
    std::deque<rs2::frame> m_fqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
    camera_intrinsics_t m_intrinsics;
};

class Rs2Device
{




public:
    Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, FrameQueue* depth_frames);
    Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, FrameQueue *depth_frames, FrameQueue *color_frames);

    ~Rs2Device();

    void setCaptureEnabled(bool running);
    void setRecordingEnabled(bool recording) { m_recording = recording; }
    bool isActive();

    std::string getCamTypeStr()
    {
        switch (m_pos_id) {
        case CameraType_t::CENTRAL: return "CENTRAL";
        case CameraType_t::FRONT: return "FRONT";
        case CameraType_t::REAR: return "REAR";
        default: return "OTHER";
        }
    }
    CameraType_t getCamType() { return m_pos_id; }
    SyncType_t getSyncType() { return m_sync_type; }

private:
    double m_last_frame_time = 0;

    FrameQueue* m_depth_frame_queue;
    FrameQueue* m_color_frame_queue;


    rs2::device m_rs2_dev;
    size_t m_rs2_dev_id;
    CameraType_t m_pos_id;
    SyncType_t m_sync_type;

    std::thread m_capture_thread;

    rs2::pointcloud m_curr_rs2_pc_cpu;
    rs2::points m_curr_rs2_points_cpu;


    bool m_active = false;
    bool m_recording = false;

    // filters
#if FILTER_DEPTH_RS_ENABLED
#if RS_FILTER_DECIMATION_ENABLED
    rs2::decimation_filter m_dec_filter = rs2::decimation_filter(RS_FILTER_DEC_MAG);
#endif
#if RS_FILTER_THRESHOLD_ENABLED
    rs2::threshold_filter m_thr_filter = rs2::threshold_filter(RS_FILTER_THR_MIN, RS_FILTER_THR_MAX);
#endif
#if RS_FILTER_HOLEFILL_ENABLED
    rs2::hole_filling_filter m_hole_filter = rs2::hole_filling_filter(RS_FILTER_HOLE_MODE);
#endif
#if RS_FILTER_SPATIAL_ENABLED
    rs2::spatial_filter m_spat_filter = rs2::spatial_filter(RS_FILTER_SPA_ALPHA, RS_FILTER_SPA_DELTA, RS_FILTER_SPA_MAG, RS_FILTER_SPA_HOLE);
#endif
#endif

    std::string get_device_name(const rs2::device& dev);

    std::string get_sensor_name(const rs2::sensor& sensor);

    void print_sensor_options(const rs2::sensor& sensor);

    void print_device_information(const rs2::device& dev);

    void rs2_capture_thread_func();

};


#endif // RS2DEVICE_H

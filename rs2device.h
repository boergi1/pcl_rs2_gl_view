#ifndef RS2DEVICE_H
#define RS2DEVICE_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <thread>
#include <mutex>

#include "format.h"
#include "customtypes.h"

class FrameQueue
{
public:
    FrameQueue(CameraType_t CameraType){
        m_camtype = CameraType;
    }
    void addFrame(rs2::frame frame)
    {
        frame.keep();
        m_mtx.lock();
        m_fqueue.push_back(frame);
        if (m_fqueue.size() > QUE_SIZE_RS2FRAMES)
        {
            std::cerr << "(FrameQueue) Too many frames in queue " << m_camtype << std::endl;
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
            std::cerr << "(FrameQueue) is empty" << std::endl;
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
    bool isEmpty() { return m_fqueue.size() == 0; }
    CameraType_t getCameraType() { return m_camtype; }
private:
    std::deque<rs2::frame> m_fqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
};

class Rs2Device
{
private:
    double m_last_frame_time = 0;

    //    rs2::frame_queue m_frame_queue;
    FrameQueue* m_depth_frame_queue;
    FrameQueue* m_color_frame_queue;

    rs2::device m_rs2_dev;
    size_t m_rs2_dev_id;
    CameraType_t m_pos_id;

    std::thread m_capture_thread;

    rs2::pointcloud m_curr_rs2_pc_cpu;
    rs2::points m_curr_rs2_points_cpu;


    bool m_active = false;
    bool m_recording = false;

    // filters
    rs2::decimation_filter m_dec_filter = rs2::decimation_filter(2.0f);                 // Decimation - reduces depth frame density
    rs2::threshold_filter m_thr_filter = rs2::threshold_filter(0.4f, 8.0f);             // Threshold  - removes values outside recommended range
    rs2::spatial_filter m_spat_filter = rs2::spatial_filter(0.65f, 20.0f, 2.0f, 2.0f);  // Spatial    - edge-preserving spatial smoothing

    std::string get_device_name(const rs2::device& dev);

    std::string get_sensor_name(const rs2::sensor& sensor);

    void print_sensor_options(const rs2::sensor& sensor);

    void print_device_information(const rs2::device& dev);

    void rs2_capture_thread_func();



public:
    //     Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, rs2::frame_queue &framebuf);
    Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, FrameQueue* depth_frames, FrameQueue *color_frames);

    ~Rs2Device();

    void setCaptureEnabled(bool running);

    void setRecordingEnabled(bool recording) { m_recording = recording; }

    bool isActive();

    std::string getPositionTypeStr()
    {
        switch (m_pos_id) {
        case CameraType_t::CENTRAL: return "CENTRAL";
        case CameraType_t::FRONT: return "FRONT";
        case CameraType_t::REAR: return "REAR";
        default: return "OTHER";
        }
    }

    CameraType_t getPositionType() { return m_pos_id; }


    //    std::function<void (rs2::frame)> depth_callback = [&](const rs2::frame& frame)
    //    {
    //        // hw sync: https://github.com/IntelRealSense/librealsense/issues/2637
    //        std::cout << "(Rs2Device " << getPositionTypeStr() << ") CALLBACK, thread id: " << std::this_thread::get_id() << std::endl;
    //        if (rs2::frameset fs = frame.as<rs2::frameset>())
    //        {
    //#if (VERBOSE > 0)
    //            auto start = std::chrono::high_resolution_clock::now();
    //#endif
    //            if (fs.size() > 1)
    //                std::cerr << "(Rs2Device " << getPositionTypeStr() << ") Multiple frames arrived: " << fs.size() << std::endl;
    //            const rs2::frame& depth_tmp = fs.get_depth_frame();
    //            if (!depth_tmp)
    //            {
    //                std::cerr << "(Rs2Device " << getPositionTypeStr() << ") No depth frame" << std::endl;
    //                return;
    //            }
    //#ifdef tmp_commented_out
    //            std::cout << "(Rs2Device) Drift: " << depth_tmp.get_timestamp() - m_last_frame_time << ", ";
    //            m_last_frame_time = depth_tmp.get_timestamp();
    //            switch(depth_tmp.get_frame_timestamp_domain()) {
    //            case (RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK):
    //                std::cout << "(Rs2Device) Hardware Clock ";
    //                break;
    //            case (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME):
    //                std::cout << "(Rs2Device) System Time ";
    //                break;
    //            default:
    //                std::cout << "(Rs2Device) Unknown Time ";
    //                break;
    //            }
    //            std::cout << "TS: " << depth_tmp.get_timestamp()
    //                      << " (" << depth_tmp.get_frame_number() << ")" << std::endl;
    //#endif
    //            rs2::pointcloud rs2_pc_cpu;
    //            rs2_pc_cpu.map_to(depth_tmp);
    //            m_curr_rs2_points_cpu = rs2_pc_cpu.calculate(depth_tmp);
    //            //            m_ref_RS_to_interface.mtx_ref->lock();
    //            //            static_cast<rs2::points*>( m_ref_RS_to_interface.buf_ref )[ *m_ref_RS_to_interface.w_idx_ref ]
    //            //            = m_curr_rs2_points_cpu;
    //            //            *m_ref_RS_to_interface.w_idx_ref = *m_ref_RS_to_interface.w_idx_ref + 1;
    //            //            std::cout << "(Rs2Device " << getPositionTypeStr() << ") Increased write index: " << *m_ref_RS_to_interface.w_idx_ref
    //            //                      << " size " << m_curr_rs2_points_cpu.size() << std::endl;
    //            //            if (*m_ref_RS_to_interface.w_idx_ref == BUF_SIZE_RS2FRAMES-1)
    //            //                *m_ref_RS_to_interface.w_idx_ref = 0;
    //            //            m_ref_RS_to_interface.mtx_ref->unlock();
    //#if (VERBOSE > 1)
    //            std::cout << "(Rs2Device " << getPositionTypeStr() << ") Acquisition thread took " << std::chrono::duration_cast<std::chrono::milliseconds>
    //                         (std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
    //#endif
    //        }
    //        else
    //        {
    //            std::cerr << "(Rs2Device " << getPositionTypeStr() << ") Unhandled frame arrived: " << fs.size() << std::endl;
    //            // Stream that bypass synchronization (such as IMU) will produce single frames
    //            //  counters[frame.get_profile().unique_id()]++;
    //        }
    //    };

};


#endif // RS2DEVICE_H

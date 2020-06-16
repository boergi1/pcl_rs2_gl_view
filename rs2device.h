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
#include "filters.h"
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

//        // Calculate extrinsic camera parameters
//        cv::Mat R_identity = cv::Mat::eye(3,3, CV_32F);
//        cv::Mat T_zero = cv::Mat::zeros(3,1,CV_32F);
//        float rot_x, rot_y, rot_z, tran_x, tran_y, tran_z;
//        switch (m_camtype) {
//        case CameraType_t::CENTRAL:
//        {
//            // No transformation
//            m_extrinsics = camera_extrinsics_t((float*)R_identity.data, (float*)T_zero.data);
//            return;
//        }
//        case CameraType_t::FRONT:
//        {
//            rot_x = static_cast<float>(degreesToRadians(ROT_FRONT_TO_CENTRAL_ANG_X));
//            rot_y = static_cast<float>(degreesToRadians(ROT_FRONT_TO_CENTRAL_ANG_Y));
//            rot_z = static_cast<float>(degreesToRadians(ROT_FRONT_TO_CENTRAL_ANG_Z));
//            tran_x = static_cast<float>(TRAN_FRONT_TO_CENTRAL_X_M);
//            tran_y = static_cast<float>(TRAN_FRONT_TO_CENTRAL_Y_M);
//            tran_z = static_cast<float>(TRAN_FRONT_TO_CENTRAL_Z_M);
//            break;
//        }
//        case CameraType_t::REAR:
//        {
//            rot_x = static_cast<float>(degreesToRadians(ROT_REAR_TO_CENTRAL_ANG_X));
//            rot_y = static_cast<float>(degreesToRadians(ROT_REAR_TO_CENTRAL_ANG_Y));
//            rot_z = static_cast<float>(degreesToRadians(ROT_REAR_TO_CENTRAL_ANG_Z));
//            tran_x = static_cast<float>(TRAN_REAR_TO_CENTRAL_X_M);
//            tran_y = static_cast<float>(TRAN_REAR_TO_CENTRAL_Y_M);
//            tran_z = static_cast<float>(TRAN_REAR_TO_CENTRAL_Z_M);
//            break;
//        }
//        default:
//        {
//            std::cerr << "(Converter) Extrinsics for camera type " << m_camtype << " not defined" << std::endl;
//            m_extrinsics = camera_extrinsics_t((float*)R_identity.data, (float*)T_zero.data);
//            return;
//        }
//        }

//        cv::Mat R_x = (cv::Mat_<float>(3,3) <<
//                       1, 0, 0,
//                       0, std::cos(rot_x), -std::sin(rot_x),
//                       0, std::sin(rot_x), std::cos(rot_x));
//        cv::Mat R_y = (cv::Mat_<float>(3,3) <<
//                       std::cos(rot_y), 0, std::sin(rot_y),
//                       0, 1, 0,
//                       -std::sin(rot_y), 0, std::cos(rot_y));
//        cv::Mat R_z = (cv::Mat_<float>(3,3) <<
//                       std::cos(rot_z), -std::sin(rot_z), 0,
//                       std::sin(rot_z), std::cos(rot_z), 0,
//                       0, 0, 1);
//        cv::Mat R = (cv::Mat_<float>(3,3));
//        cv::Mat T = (cv::Mat_<float>(3,1) << tran_x, tran_y, tran_z);

//        if (CONV_RS_ROTATION_AXES == "xyz")
//            R = R_z * R_y * R_x;
//        else if (CONV_RS_ROTATION_AXES == "xzy")
//            R = R_y * R_z * R_x;
//        else if (CONV_RS_ROTATION_AXES == "yxz")
//            R = R_z * R_x * R_y;
//        else if (CONV_RS_ROTATION_AXES == "yzx")
//            R = R_x * R_z * R_y;
//        else if (CONV_RS_ROTATION_AXES == "zxy")
//            R = R_y * R_x * R_z;
//        else if (CONV_RS_ROTATION_AXES == "zyx")
//            R = R_x * R_y * R_z;
//        else {
//            std::cerr << "(Converter) Invalid rotation axis specified" << std::endl;
//            m_extrinsics = camera_extrinsics_t((float*)R_identity.data, (float*)T_zero.data);
//            return;
//        }
//        m_extrinsics = camera_extrinsics_t((float*)R.data, (float*)T.data);

//        std::cerr << std::endl << "DEBUG Extrinsics FRAMEQUEUE" << std::endl
//                  << m_extrinsics.R[0] << " " << m_extrinsics.R[1] << " " << m_extrinsics.R[2] << " | " << m_extrinsics.T[0] << std::endl
//                  << m_extrinsics.R[3] << " " << m_extrinsics.R[4] << " " << m_extrinsics.R[5] << " | " << m_extrinsics.T[1] << std::endl
//                  << m_extrinsics.R[6] << " " << m_extrinsics.R[7] << " " << m_extrinsics.R[8] << " | " << m_extrinsics.T[2] <<  std::endl << std::endl;
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
    camera_extrinsics_s* getExtrinsics() { return &m_extrinsics; }
    bool isEmpty() { return m_fqueue.size() == 0; }
    bool size() { return m_fqueue.size(); }
    CameraType_t getCameraType() { return m_camtype; }
private:
    std::deque<rs2::frame> m_fqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
    camera_intrinsics_t m_intrinsics;
    camera_extrinsics_t m_extrinsics;
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

#ifndef RS2_PCL_CONVERTER_H
#define RS2_PCL_CONVERTER_H

#include <iostream>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>

#include <pcl/common/transforms.h>

#include "format.h"
#include "customtypes.h"
#include "deviceinterface.h"
#include "processinginterface.h"

#include "threadcontroller.h"

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// custom ids
typedef enum
{
    TSKTYPE_Frame2Pts = 0,
    TSKTYPE_Pts2Cld,
    TSKTYPE_Frame2Depth,
    TSKTYPE_Frame2Color,
    TSKTYPE_Frame2Cloud
} TaskType_t;

//class FrameToDepthTask : public BaseTask
//{
//public:
//    FrameToDepthTask()
//    {

//    }
//    virtual ~FrameToDepthTask() override
//    {

//    }

//    std::deque<std::pair<unsigned long long, rs2::frame>> in;
//    std::vector<std::tuple<unsigned long long, cv::Mat, long long>> out;
//    void process() override
//    {
//        size_t i = 0;
//        out.resize(in.size());
//#if (VERBOSE > 1)
//        auto start = std::chrono::high_resolution_clock::now();
//#endif
//        while (in.size())
//        {
//            auto tmp_pair = in.front();
//            in.pop_front();
//            rs2::depth_frame frame = tmp_pair.second.as<rs2::depth_frame>();
//            long long ts = 0;
//            if (frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
//                ts = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
//            out.at(i++) = std::make_tuple(tmp_pair.first, cv::Mat(cv::Size(frame.get_width(), frame.get_height()), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP), ts);
//        }
//        this->setTaskStatus(TASK_DONE);
//#if (VERBOSE > 1)
//        std::cout << "(Converter) FrameToDepthTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
//                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
//#endif
//    }
//};

//class FrameToDepthCloudTask : public BaseTask
//{
//public:
//    FrameToDepthCloudTask(camera_parameters_t intr)
//    {
//        m_intr = intr;
//    }
//    virtual ~FrameToDepthCloudTask() override
//    {

//    }
//    std::deque<std::pair<unsigned long long, rs2::frame>> in;
//    std::vector<std::tuple<unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>> out;

//    void process() override
//    {
//        size_t i = 0;
//        out.resize(in.size());
//#if (VERBOSE > 1)
//        auto start = std::chrono::high_resolution_clock::now();
//#endif
//        while (in.size())
//        {
//            auto tmp_pair = in.front();
//            in.pop_front();
//            rs2::depth_frame frame = tmp_pair.second.as<rs2::depth_frame>();
//            long long ts = 0;
//            if (frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
//                ts = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);

//            auto pointcloud = MatToCloudXYZ(cv::Mat(cv::Size(frame.get_width(), frame.get_height()), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP),
//                                            m_intr.fx, m_intr.fy, m_intr.cx, m_intr.cy);
//            out.at(i++) = std::make_tuple(tmp_pair.first, pointcloud, ts);
//        }
//        this->setTaskStatus(TaskStatus_t::TASK_DONE);
//#if (VERBOSE > 1)
//        std::cout << "(Converter) FrameToDepthCloudTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
//                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
//#endif
//    }

//    pcl::PointCloud<pcl::PointXYZ>::Ptr MatToCloudXYZ(cv::Mat depthMat, float fx, float fy, float cx, float cy)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);

//        unsigned char* p = depthMat.data;
//        for (int i = 0; i<depthMat.rows; i++)
//        {
//            for (int j = 0; j < depthMat.cols; j++)
//            {
//                float z = static_cast<float>(*p);
//                pcl::PointXYZ point;
//                point.z = 0.001 * z;
//                point.x = point.z * (j - cx) / fx;
//                point.y = point.z * (cy - i) / fy;
//                ptCloud->points.push_back(point);
//                ++p;
//            }
//        }
//        ptCloud->width = (int)depthMat.cols;
//        ptCloud->height = (int)depthMat.rows;

//        return ptCloud;
//    }

//private:
//    camera_parameters_t m_intr;
//};



//class FrameToColorTask : public BaseTask
//{
//public:
//    FrameToColorTask()
//    {

//    }
//    virtual ~FrameToColorTask() override
//    {

//    }

//    std::deque<std::pair<unsigned long long, rs2::frame>> in;
//    std::vector<std::tuple<unsigned long long, cv::Mat, long long>> out;
//    void process() override
//    {
//        size_t i = 0;
//        out.resize(in.size());
//#if (VERBOSE > 1)
//        auto start = std::chrono::high_resolution_clock::now();
//#endif
//        while (in.size())
//        {
//            auto tmp_pair = in.front();
//            in.pop_front();
//            rs2::video_frame frame = tmp_pair.second.as<rs2::video_frame>();
//            long long ts = 0;
//            if (frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
//                ts = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
//            out.at(i++) = std::make_tuple(tmp_pair.first, cv::Mat(cv::Size(frame.get_width(), frame.get_height()), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP), ts);
//        }
//        this->setTaskStatus(TASK_DONE);
//#if (VERBOSE > 1)
//        std::cout << "(Converter) FrameToColorTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
//                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
//#endif
//    }
//};

class FrameToCloudTask : public BaseTask
{
public:
    FrameToCloudTask(int taskId, camera_intrinsics_s* intrinsics, camera_extrinsics_s* extrinsics)
    {
        this->setTaskId(taskId);
        m_intr = intrinsics;
        m_extr = extrinsics;

    }
    virtual ~FrameToCloudTask() override {}
    std::deque<std::tuple<unsigned long long, cv::Mat, long long>> in;
    std::vector<std::tuple<unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>> out;
    void process() override
    {
#if (VERBOSE > 1)
        auto start = std::chrono::high_resolution_clock::now();
#endif
        size_t i = 0;
        out.resize(in.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_WIDTH_DEPTH, RS_FRAME_HEIGHT_DEPTH));

        while (in.size())
        {
            auto tmp_t = in.front();
            in.pop_front();
            cv::Mat depth_mat = std::get<1>(tmp_t);
            MatToCloudXYZ(depth_mat, cloud, m_intr, m_extr);
            out.at(i++) = std::make_tuple(std::get<0>(tmp_t), cloud, std::get<2>(tmp_t));
        }
        this->setTaskStatus(TASK_DONE);
#if (VERBOSE > 1)
        std::cout << "(Converter) FrameToCloudTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
    }
private:
    camera_intrinsics_t* m_intr;
    camera_extrinsics_t* m_extr;

    void MatToCloudXYZ(cv::Mat& depthMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, camera_intrinsics_t* intrinsics, camera_extrinsics_t* extrinsics)
    {
        size_t point_idx = 0, skipped_pts = 0;
        ushort* data_ptr = (ushort*) depthMat.data;

        for (int i = 0; i<depthMat.rows; i++)
        {
            for (int j = 0; j < depthMat.cols; j++)
            {
                float z = static_cast<ushort>(*data_ptr) * 0.001; // segfault
                // z = 0.001 * z;
                if (z > 0.f)
                {
                    float origin[3] { z * (j - intrinsics->cx) / intrinsics->fx, z * (intrinsics->cy - i) / intrinsics->fy, z };
                    float target[3];
                    transformPointToPoint(origin, target, extrinsics);
                    cloud->points.at(point_idx).x = target[0];
                    cloud->points.at(point_idx).y = target[1];
                    cloud->points.at(point_idx).z = target[2];
                    //                    cloud->points.at(point_idx).x = z * (j - intrinsics->cx) / intrinsics->fx;
                    //                    cloud->points.at(point_idx).y = z * (intrinsics->cy - i) / intrinsics->fy;
                    //                    cloud->points.at(point_idx).z = z;
                }
                else
                {
                    cloud->points.at(point_idx).x = 0.f;
                    cloud->points.at(point_idx).y = 0.f;
                    cloud->points.at(point_idx).z = 0.f;
                    skipped_pts++;
                }

                ++point_idx;
                ++data_ptr;
            }
        }
//        std::cout << "DEBUG skipped " << skipped_pts << "(type:" << this->getTaskType() << " id:" << this->getTaskId() << ")" << std::endl;
    }

    inline void transformPointToPoint(const float* origin_pt, float* target_pt, const camera_extrinsics_t* extrin)
    {
        target_pt[0] = extrin->R[0] * origin_pt[0] + extrin->R[1] * origin_pt[1] + extrin->R[2] * origin_pt[2] + extrin->T[0];
        target_pt[1] = extrin->R[3] * origin_pt[0] + extrin->R[4] * origin_pt[1] + extrin->R[5] * origin_pt[2] + extrin->T[1];
        target_pt[2] = extrin->R[6] * origin_pt[0] + extrin->R[7] * origin_pt[1] + extrin->R[8] * origin_pt[2] + extrin->T[2];
    }

    //void PointsToCloudTask::rs2_transform_point_to_point_custom(float *to_point, const rs2_extrinsics *extrin, const float *from_point)
    //{
    //    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    //    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    //    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
    //}

};

//void PointsToCloudTask::rs2_transform_point_to_point_custom(float *to_point, const rs2_extrinsics *extrin, const float *from_point)
//{
//    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
//    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
//    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
//}

//void PointsToCloudTask::points_to_pcl(const rs2::points &points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud)
//{
//    int skipped = 0;
//    const rs2::vertex* rs_vertex = points.get_vertices();
//    size_t pt_count = points.size();
//#if !PCL_CLOUD_ORGANIZED
//    pcloud->reserve(pt_count);
//#endif
//    switch (this->getTaskId()) {
//    case CameraType_t::CENTRAL: // no transformation
//        for (size_t i=0; i<pt_count; i++)
//        {
//            if ( !areSameF(rs_vertex->z, 0.0f) )
//            {
//#if PCL_CLOUD_ORGANIZED
//                pcloud->points.at(i).x = rs_vertex->x;
//                pcloud->points.at(i).y = rs_vertex->y;
//                pcloud->points.at(i).z = rs_vertex->z;
//#else
//                pcloud->push_back(pcl::PointXYZ(rs_vertex->x, rs_vertex->y, rs_vertex->z));
//#endif
//            }
//            else skipped++;
//            rs_vertex++;
//        }
//        break;
//    case CameraType_t::FRONT:
//        for (size_t i=0; i<pt_count; i++)
//        {
//            if ( !areSameF(rs_vertex->z, 0.0f) )
//            {
//                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
//                float target[3];
//                rs2_transform_point_to_point_custom(target, &m_extrinsics_front, origin);
//#if PCL_CLOUD_ORGANIZED
//                pcloud->points.at(i).x = target[0];
//                pcloud->points.at(i).y = target[1];
//                pcloud->points.at(i).z = target[2];
//#else
//                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
//#endif
//            }
//            else skipped++;
//            rs_vertex++;
//        }
//        break;
//    case CameraType_t::REAR:
//        for (size_t i=0; i<pt_count; i++)
//        {
//            if ( !areSameF(rs_vertex->z, 0.0f) )
//            {
//                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
//                float target[3];
//                rs2_transform_point_to_point_custom(target, &m_extrinsics_rear, origin);
//#if PCL_CLOUD_ORGANIZED
//                pcloud->points.at(i).x = target[0];
//                pcloud->points.at(i).y = target[1];
//                pcloud->points.at(i).z = target[2];
//#else
//                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
//#endif
//            }
//            else skipped++;
//            rs_vertex++;
//        }
//        break;
//    }
//}


//class FrameToPointsTask : public BaseTask
//{
//public:
//    FrameToPointsTask();
//    virtual ~FrameToPointsTask() override;
//    std::deque<rs2::frame> in;
//    std::vector<std::tuple <rs2::points, long long, unsigned long long> > out;
//    void process() override;
//};

//class PointsToCloudTask : public BaseTask
//{
//public:
//    PointsToCloudTask();
//    virtual ~PointsToCloudTask() override;
//    std::deque< std::tuple <rs2::points, long long, unsigned long long> > in;
//    std::vector< std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, double, unsigned long long> > out;
//    void process() override;

//private:
//    rs2_extrinsics m_extrinsics_front, m_extrinsics_rear;
//    void points_to_pcl(const rs2::points &points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud);
//    void points_to_pcl_rgb(const rs2::points &points, const rs2::video_frame& color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud);
//    inline void rs2_transform_point_to_point_custom(float* to_point, const struct rs2_extrinsics* extrin, const float* from_point);
//    inline void textureToColor(const rs2::video_frame* rgb_frame, const rs2::texture_coordinate* texture_uv, std::tuple<uint8_t, uint8_t, uint8_t>* output);
//};



class Rs2_PCL_Converter : ThreadController
{
private:

#if RS_DEPTH_ENABLED
    std::vector<FrameQueue*>* m_ref_to_depth_queues;
#endif
#if RS_COLOR_ENABLED
    std::vector<FrameQueue*>* m_ref_to_color_queues;
#endif

    std::vector<CloudQueue*>* m_ref_to_cloud_queues;
    std::vector<MatQueue*>* m_ref_out_depth;
    std::vector<MatQueue*>* m_ref_out_color;

    // todo: camtype + extrinsics can be simplyfied
    std::vector<CameraType_t> m_cam_types;
    std::vector<camera_extrinsics_t> m_extrinsics;

    std::vector<shared_references_t> m_refs_conv_to_PCL;

    bool m_active = false;

    std::thread m_converter_thread;

    void converter_thread_func();

    float* copyMatPtrFloat(const cv::Mat& src)
    {
        float* M_ptr = new float[src.total()];
        std::memcpy(M_ptr, src.data, src.rows*src.step);
        return M_ptr;
    }
//    void* copyMatDataPtr(cv::Mat& src)
//    {
//        void* M_ptr = operator new(src.total());
//        std::memcpy(M_ptr, src.data, src.rows*src.step);
//        return M_ptr;
//    }


public:
    Rs2_PCL_Converter(DeviceInterface* in_interface_ref, ProcessingInterface* out_interface_ref, std::vector<CameraType_t> camera_types );

    virtual ~Rs2_PCL_Converter() override;

    void init(int ThreadPoolSize) override;

    void setActive(bool running)
    {
        if (running)
        {
            if ( m_converter_thread.joinable() )
            {
                std::cerr << "(Converter) Thread already running: " << m_converter_thread.get_id() << std::endl;
                return;
            }
            m_active = true;
            m_converter_thread = std::thread(&Rs2_PCL_Converter::converter_thread_func, this);
        }
        else
        {
            m_active = false;
            if ( m_converter_thread.joinable() )
                m_converter_thread.join();
            else std::cerr << "(Converter) Thread not joinable: " << m_converter_thread.get_id() << std::endl;
        }
        return;
    }


    bool isActive();



    //    void startThread()
    //    {
    //        if (m_rs2_device_count == 1)
    //            m_converter_thread = std::thread(&Rs2_PCL_Converter::converter_thread_func, this);
    //        else std::cout << "Multi cam currently not supported";
    //    }
};





#endif // RS2_PCL_CONVERTER_H

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
#include "pclinterface.h"

#include "threadcontroller.h"

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// custom ids
typedef enum
{
    TSKTYPE_F2P = 0,
    TSKTYPE_P2C
} TaskType_t;

class FrameToPointsTask : public BaseTask
{
public:
    FrameToPointsTask();
    virtual ~FrameToPointsTask() override;

    std::deque<rs2::frame> in;
    std::vector<rs2::points> out;
   // std::vector<std::tuple <rs2::points, double, unsigned long long> > out;

    void process() override;
};

class PointsToCloudTask : public BaseTask
{
public:
    PointsToCloudTask();
    virtual ~PointsToCloudTask() override;

   // std::deque< std::tuple <rs2::points, double, unsigned long long> > in;
    std::deque< rs2::points > in;

    std::vector< std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, double, unsigned long long> > out;

    void process() override;

private:

    inline void rs2_transform_point_to_point_custom(float* to_point, const struct rs2_extrinsics* extrin, const float* from_point);

    inline void points_to_pcl(const rs2::points &points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud);

};

#define CONV_FRAMES_TO_POINTS_TASKS 3
#define CONV_POINTS_TO_CLOUD_TASKS 3

class Rs2_PCL_Converter : ThreadController
{
private:


    std::vector<rs2::frame_queue>* m_ref_to_rs2_frames;
    std::vector<CloudDeque*>* m_ref_to_pcl_queues;
    std::vector<CamPosition_t> m_cam_positions;

    std::vector<FrameToPointsTask*> m_tasks_f2p;
    std::vector<PointsToCloudTask*> m_tasks_p2c;


    std::vector<shared_references_t> m_refs_conv_to_PCL;





    bool m_active = false;




    //    std::mutex* m_points_mutex_ref;
    //    rs2::points* m_points_buf_ref;
    //    size_t* m_points_write_idx_ref;
    //    size_t* m_points_read_idx_ref;

    //  std::mutex* m_clouds_mutex_ref = new std::mutex();
    //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* m_clouds_buf_ref;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_clouds_buf_ref;
    // size_t* m_clouds_write_idx_ref;
    // size_t* m_clouds_read_idx_ref;

    //  DeviceInterface* m_ref_interface;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>(FRAME_WIDTH, FRAME_HEIGHT));
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>(1280,720));
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_current_cloud;

    std::thread m_converter_thread;
    bool m_running = true;

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr());
    // m_dev_thread = std::thread(&Rs2Device::captureClient, this);

    void converter_thread_func_old();

    //    inline void rs2_transform_point_to_point_custom(float* to_point, const struct rs2_extrinsics* extrin, const float* from_point);

    //    inline void points_to_pcl(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud, Rs2Position_t dev_idx);


    void converter_thread_func();



public:
    Rs2_PCL_Converter(DeviceInterface* in_interface_ref, PclInterface* out_interface_ref );

    virtual ~Rs2_PCL_Converter() override;

    void init() override;

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

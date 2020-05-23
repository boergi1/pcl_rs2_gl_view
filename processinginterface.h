#ifndef PROCESSINGINTERFACE_H
#define PROCESSINGINTERFACE_H

#include <pcl/point_types.h>
//#include <pcl/point_cloud.h> // ?
#include <pcl/ModelCoefficients.h>

#include <pcl/common/time.h>
#include <pcl/common/common_headers.h>

#include <pcl/console/parse.h>

//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "format.h"
#include "customtypes.h"
#include <thread>
#include <mutex>
#include <chrono>

/* TODO: Add class for each device with its own processing thread*/

class CloudQueue
{
public:
    CloudQueue(CameraType_t CameraType, std::string name = "");
    void addCloudT(std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> cloud_tuple);
    // <std::tuple<unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>>
    std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> getCloudT();
    const std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> readCloudT();
    bool isEmpty();
    CameraType_t getCameraType();
private:
    std::deque<std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long>>  m_cqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
    std::string m_name;
};

class MatQueue
{
public:
    MatQueue(CameraType_t CameraType, std::string name)
    {
        m_camtype = CameraType;
        m_name = name;
    }
    void addMatT(std::tuple<unsigned long long, cv::Mat, long long> mat_tuple)
    {
        m_mtx.lock();
        m_mqueue.push_back(mat_tuple);
        if (m_mqueue.size() > QUE_SIZE_PCL)
        {
            std::cerr << "(MatQueue) Too many Mats in queue " << m_name << " " << m_camtype << std::endl;
            m_mqueue.pop_front();
        }
        m_mtx.unlock();
    }

    std::tuple<unsigned long long, cv::Mat, long long> getMatT()
    {
        if (m_mqueue.size())
        {
            m_mtx.lock();
            auto mat_t = m_mqueue.front();
            m_mqueue.pop_front();
            m_mtx.unlock();
            return mat_t;
        }
        else
        {
            std::cerr << "(MatQueue) is empty " << m_name << " " << m_camtype << std::endl;
            return *m_mqueue.end();
        }
    }

    const std::tuple<unsigned long long, cv::Mat, long long> readMatT()
    {
        if (m_mqueue.size())
        {
            const auto mat_t = m_mqueue.front();
            return mat_t;
        }
        else
        {
            std::cerr << "(MatQueue) is empty " << m_name << " " << m_camtype << std::endl;
            return *m_mqueue.end();
        }
    }

    bool isEmpty() { return m_mqueue.size() == 0; }

    CameraType_t getCameraType() { return m_camtype; }

private:
    std::deque<std::tuple<unsigned long long, cv::Mat, long long>> m_mqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
    std::string m_name;
};

class ProcessingInterface // todo: add one thread for each camera
{
private:
    bool m_active;
    //    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clouds_buffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_proc_cloud = nullptr; // todo mutex this
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr
            (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector< CloudQueue* > m_input_clouds;
    std::vector< MatQueue* > m_input_depth;
    std::vector< MatQueue* > m_input_color;

    std::thread m_pc_proc_thread;

#if (PCL_VIEWER == 1)
    pcl::visualization::CloudViewer m_pcl_viewer;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > m_viewer_clouds;
    std::mutex m_viewer_mtx;
#endif

    void pc_proc_thread_func();

public:
    ProcessingInterface(std::vector<CameraType_t> device_count);

    std::vector<CloudQueue *>* getInputCloudsRef() { return &m_input_clouds; }
    std::vector<MatQueue *>* getDepthImageRef() { return &m_input_depth; }
    std::vector<MatQueue *>* getColorImageRef() { return &m_input_color; }

#if PCL_VIEWER
    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_callback = [](pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.setBackgroundColor (1.0, 0.5, 1.0);
#if PCL_FILTER_GLOBAL_REGION_ENABLED
        viewer.addCube(PCL_GLOBAL_REGION_X_MIN_M, PCL_GLOBAL_REGION_X_MAX_M, PCL_GLOBAL_REGION_Y_MIN_M, PCL_GLOBAL_REGION_Y_MAX_M,
                       PCL_GLOBAL_REGION_Z_MIN_M, PCL_GLOBAL_REGION_Z_MAX_M, 1, 0, 0, "cropregion");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cropregion");
#endif
    };
    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_update_callback = [this](pcl::visualization::PCLVisualizer& viewer)
    {
#if (VERBOSE > 0)
        auto start = std::chrono::high_resolution_clock::now();
#endif
        bool idle = true;
        for (size_t i=0; i< m_viewer_clouds.size(); i++) {
            m_viewer_mtx.lock();
            auto cloud = m_viewer_clouds.at(i);
            m_viewer_mtx.unlock();
            if (cloud != nullptr && !cloud->points.empty())
            {
                idle = false;
                std::string pc_id = std::to_string(i);

                if(!viewer.updatePointCloud<pcl::PointXYZ>(cloud, pc_id))
                {
                    viewer.addPointCloud<pcl::PointXYZ>(cloud, pc_id);
                    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_id);
                    //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, pc_id);
                    //  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_JET, 0, 255, "rs_cloud" );
                    //                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
                    //                                                        pcl::visualization::PCL_VISUALIZER_LUT_JET, "rs_cloud");
                }

            }
        }
        if (idle)
        {
            std::cerr << "(Pcl-Viewer) Idle" << std::endl;
            std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PCL_VIEW_NS));
            return;
        }
        //  viewer.spinOnce(RS_FRAME_PERIOD_MS);
#if (VERBOSE > 0)
        std::cout << "(Pcl-Viewer) Visualization callback from thread " << std::this_thread::get_id() << " took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif

        //    std::cout << "(Converter) Task (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") converted "
        //              << pcloud->points.size() << " points to clouds, skipped " << skipped << ", took " << std::chrono::duration_cast
        //                 <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;

        //        if (m_proc_cloud != nullptr && !m_proc_cloud->empty())
        //        {
        //            std::cout << "PCL visualization callback - 1 # " << std::this_thread::get_id() << std::endl;
        //            if(!viewer.updatePointCloud<pcl::PointXYZ>(m_proc_cloud, "rs cloud"))
        //            {
        //                viewer.addPointCloud<pcl::PointXYZ>(m_proc_cloud, "rs cloud");
        //                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rs cloud");
        //                //  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_JET, 0, 255, "rs_cloud" );
        //                //                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
        //                //                                                        pcl::visualization::PCL_VISUALIZER_LUT_JET, "rs_cloud");

        //            }
        //        }
        //        if (m_filtered_cloud != nullptr && m_filtered_cloud->size() && !m_filtered_cloud->empty())
        //        {
        //            std::cout << "PCL visualization callback - 2 # " << std::this_thread::get_id() << std::endl;
        //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (m_filtered_cloud, 0, 0, 255);
        //            if(!viewer.updatePointCloud(m_filtered_cloud, rgb, "extract cloud"))
        //            {
        //                viewer.addPointCloud(m_filtered_cloud, rgb, "extract cloud");
        //                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "extract cloud");
        //            }
        //        }



    };
#endif

    void initPcViewer (pcl::visualization::PCLVisualizer& viewer);

    void startThread();

    void setActive(bool running);

    bool isActive();


};

#endif

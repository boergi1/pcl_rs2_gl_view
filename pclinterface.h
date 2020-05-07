#ifndef PCLINTERFACE_H
#define PCLINTERFACE_H

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
#include <pcl/point_cloud.h> // ?
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

#include "format.h"
#include "customtypes.h"
#include <thread>
#include <mutex>
#include <chrono>

class CloudDeque
{
public:
    CloudDeque(CameraType_t CameraType){
        m_camtype = CameraType;
    }
    void addCloudT(std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, double, unsigned long long> cloud_tuple)
    {
        m_mtx.lock();
        m_queue.push_back(cloud_tuple);
        if (m_queue.size() > QUE_SIZE_PCL)
        {
            std::cerr << "(CloudDeque) Too many clouds in queue" << std::endl;
            m_queue.pop_front();
        }
        m_mtx.unlock();
    }
    std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, double, unsigned long long> getCloudT()
    {
        if (m_queue.size())
        {
            m_mtx.lock();
            auto tmp_ptr = m_queue.front();
            m_queue.pop_front();
            m_mtx.unlock();
            return tmp_ptr;
        }
        else
        {
            std::cerr << "(CloudDeque) is empty" << std::endl;
            return std::make_tuple(nullptr, NULL, NULL);
        }
    }
    bool isEmpty() { return m_queue.size() == 0; }
    CameraType_t getCameraType() { return m_camtype; }
private:
    std::deque<std::tuple <pcl::PointCloud<pcl::PointXYZ>::Ptr, double, unsigned long long>>  m_queue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
};

class PclInterface
{
private:
    bool m_active;
    //    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clouds_buffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_proc_cloud = nullptr; // todo mutex this
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr
            (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector< CloudDeque* > m_input_clouds;
    std::thread m_pc_proc_thread;




#if (PCL_VIEWER == 1)
    pcl::visualization::CloudViewer m_pcl_viewer;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > m_viewer_clouds;
#endif

    void pc_proc_thread_func();

public:
    PclInterface(std::vector<CameraType_t> device_count);



    std::vector<CloudDeque *>* getInputCloudsRef();

#if (PCL_VIEWER == 1)
    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_callback = [](pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.setBackgroundColor (1.0, 0.5, 1.0);
    };
    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_update_callback = [this](pcl::visualization::PCLVisualizer& viewer)
    {
#if (VERBOSE > 1)
        auto start = std::chrono::high_resolution_clock::now();
#endif
        bool idle = true;
        for (size_t i=0; i< m_viewer_clouds.size(); i++) {
            auto cloud = m_viewer_clouds.at(i);
            if (cloud != nullptr && !cloud->empty())
            {
                idle = false;
                std::string pc_id = std::to_string(i);
                if(!viewer.updatePointCloud<pcl::PointXYZ>(cloud, pc_id))
                {
                    viewer.addPointCloud<pcl::PointXYZ>(cloud, pc_id);
                    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, pc_id);
                    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, pc_id);
                    //  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_JET, 0, 255, "rs_cloud" );
                    //                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
                    //                                                        pcl::visualization::PCL_VISUALIZER_LUT_JET, "rs_cloud");
                }
            }
        }
        if (idle)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PCL_VIEW_NS / m_viewer_clouds.size()));
            return;
        }
        viewer.spinOnce(RS_FRAME_PERIOD_MS);
#if (VERBOSE > 1)
        std::cout << "(PclInterface) Visualization callback from thread " << std::this_thread::get_id() << " took " << std::chrono::duration_cast
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

    std::vector<shared_references_t> get_pcl_data_refs();

};

#endif // PCLINTERFACE_H

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

#include "format.h"
#include <thread>
#include <mutex>
#include <chrono>

class PclInterface
{
private:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clouds_buffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_proc_cloud = nullptr; // todo mutex this
    size_t m_clouds_write_idx = 0;
    size_t m_clouds_read_idx = 0;


    size_t m_clouds_buf_step = 0;

    std::thread m_pc_proc_thread;

    std::mutex* m_pcl_cvt_mtx = new std::mutex;

    bool m_running = true;

    pcl::visualization::CloudViewer m_pcl_viewer;

    void pc_proc_thread_func()
    {

        std::cout << "PCL thread started # " << std::this_thread::get_id() << std::endl;
        while (m_running)
        {

            // std::cout << "pointcloud processing thread running" << std::endl;
            if (m_clouds_read_idx != m_clouds_write_idx)
            {
                // Read from pcl::PointCloud buffer
                m_pcl_cvt_mtx->lock();
                m_proc_cloud = m_clouds_buffer.at(m_clouds_read_idx++);
                if (m_clouds_read_idx == CLOUD_BUF_SIZE-1)
                    m_clouds_read_idx = 0;
                m_pcl_cvt_mtx->unlock();
                cout << "(PCL) Increased read index: " << m_clouds_read_idx << " size " << m_proc_cloud->size() << endl;
                // PCL processing

            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(PROC_DELAY));
            }

            //            // Read from rs2::points buffer
            //            m_points_mutex_ref->lock();
            //            rs2::points points = m_points_buf_ref[ *m_points_read_idx_ref ];
            //            *m_points_read_idx_ref = *m_points_read_idx_ref + 1;
            //            if (*m_points_read_idx_ref == POINT_BUF_SIZE-1)
            //                *m_points_read_idx_ref = 0;
            //            cout << "(Converter) Increased read index: " << *m_points_read_idx_ref << " size " << points.size() << endl;
            //            m_points_mutex_ref->unlock();


        }
    }

public:
    PclInterface(size_t pc_devices_count) : m_pcl_viewer("Cloud Viewer")
    {
        m_clouds_buf_step = pc_devices_count;
    }

    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_callback = [](pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.setBackgroundColor (1.0, 0.5, 1.0);
    };

    std::function<void (pcl::visualization::PCLVisualizer&)> viewer_update_callback = [this](pcl::visualization::PCLVisualizer& viewer)
    {
        // auto start = chrono::steady_clock::now();
        std::cout << "PCL visualization started # " << std::this_thread::get_id() << std::endl;

        if (m_proc_cloud != nullptr && !m_proc_cloud->empty())
        {
            if(!viewer.updatePointCloud(m_proc_cloud, "rs cloud"))
            {

                viewer.addPointCloud(m_proc_cloud, "rs cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rs cloud");
                //  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_JET, 0, 255, "rs_cloud" );
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
                                                        pcl::visualization::PCL_VISUALIZER_LUT_JET, "rs_cloud");

            }
        }
        viewer.spinOnce (10);
        //  auto end = chrono::steady_clock::now();
        //   cout << "View thread took " << chrono::duration_cast<chrono::milliseconds>(end-start).count() << " ms" << endl;


    };

    void initPcViewer (pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.setBackgroundColor (1.0, 0.5, 1.0);
        //    pcl::PointXYZ o;
        //    o.x = 1.0;
        //    o.y = 0;
        //    o.z = 0;
        //    viewer.addSphere (o, 0.25, "sphere", 0);
    }

    void updatePointCloud (pcl::visualization::PCLVisualizer& viewer)
    {
        // auto start = chrono::steady_clock::now();
        std::cout << "PCL visualization started # " << std::this_thread::get_id() << std::endl;


        if(!viewer.updatePointCloud(m_proc_cloud, "rs cloud"))
        {
            if (!m_proc_cloud->empty())
            {
                viewer.addPointCloud(m_proc_cloud, "rs cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rs cloud");
                //  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_JET, 0, 255, "rs_cloud" );
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
                                                        pcl::visualization::PCL_VISUALIZER_LUT_JET, "rs_cloud");
            }
        }
        viewer.spinOnce (10);
        //  auto end = chrono::steady_clock::now();
        //   cout << "View thread took " << chrono::duration_cast<chrono::milliseconds>(end-start).count() << " ms" << endl;


    }

    void startThread()
    {
        if (m_clouds_buf_step > 0)
            m_pc_proc_thread = std::thread(&PclInterface::pc_proc_thread_func, this);
        else std::cout << "PC thread not started" << std::endl;
        //        pcl::visualization::CloudViewer m_pcl_viewer("Cloud Viewer");
        //  m_pcl_viewer = pcl::visualization::CloudViewer("viewer");


        //    pcl_viewer.runOnVisualizationThreadOnce (&PclInterface::initPcViewer);
        // viewer_callback
        m_pcl_viewer.runOnVisualizationThreadOnce (viewer_callback);
        m_pcl_viewer.runOnVisualizationThread(viewer_update_callback);
        //        pcl_viewer.runOnVisualizationThread(&PclInterface::updatePointCloud);
        //        while (!m_pcl_viewer.wasStopped ())
        //        {
        //            //  std::chrono::steady_clock::now();
        //            std::cout << "PCL viewer main" << std::endl;
        //            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //        }
    }

    std::mutex* getCloudsBufferMutex()
    {
        return m_pcl_cvt_mtx;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* getCloudsBufferRef()
    {
        return &m_clouds_buffer;
    }

    size_t& getCloudsWriteIndexRef()
    {
        return  m_clouds_write_idx;
    }
    size_t& getCloudsReadIndexRef()
    {
        return  m_clouds_read_idx;
    }
};

#endif // PCLINTERFACE_H

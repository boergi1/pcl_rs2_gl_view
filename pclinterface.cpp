#include "pclinterface.h"

//PclInterface::PclInterface()
//{

//}

void PclInterface::pc_proc_thread_func()
{
    std::cout << "PCL thread started # " << std::this_thread::get_id() << std::endl;
#if (PCL_VIEWER == 1)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
#endif
    while (m_active)
    {
#if (VERBOSE > 0)
        auto start = std::chrono::high_resolution_clock::now();
#endif
        bool idle = true;
        for (size_t i = 0; i < m_input_clouds.size(); i++) {
            auto cloudqueue = m_input_clouds.at(i);
            if ( !cloudqueue->isEmpty() )
            {
                idle = false;
                auto camType = cloudqueue->getCameraType();
                auto cloudt = cloudqueue->getCloudT();
                auto pointcloud = std::get<0>(cloudt);
                auto ts = std::get<1>(cloudt);
                auto ctr = std::get<2>(cloudt);
#if (VERBOSE > 0)
                std::cout << "(PclInterface) PointCloud received from camera " << camType << "(" << ctr << "), size: " << pointcloud->size() << std::endl;
#endif
                //  pcl::CropBox


#if (PCL_VIEWER == 1)
//                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//                voxel_grid.setInputCloud (pointcloud);
//                voxel_grid.setLeafSize (0.05, 0.05, 0.05);
//                voxel_grid.filter(*cloud_downsampled);
//                m_viewer_clouds.at(i) = cloud_downsampled;
                   m_viewer_clouds.at(i) = pointcloud;
#endif
            }

        }

        if (idle)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PCL_POLL_NS));
            continue;
        }
#if (VERBOSE > 0)
        std::cout << "(PclInterface) Processing thread " << std::this_thread::get_id() << " took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif


        //        if (m_clouds_read_idx != m_clouds_write_idx)
        //        {
        //            // Read from pcl::PointCloud buffer
        //            m_pcl_cvt_mtx->lock();
        //            m_proc_cloud = m_clouds_buffer.at(m_clouds_read_idx++);
        //            if (m_clouds_read_idx == BUF_SIZE_CLOUDS-1)
        //                m_clouds_read_idx = 0;
        //            m_pcl_cvt_mtx->unlock();
        //            cout << "(PCL) Increased read index: " << m_clouds_read_idx << " size " << m_proc_cloud->size() << endl;
        //            // PCL processing
        //            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        //            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        //            pcl::SACSegmentation<pcl::PointXYZ> seg;
        //            seg.setOptimizeCoefficients (true);
        //            seg.setModelType (pcl::SACMODEL_PLANE);
        //            seg.setMethodType (pcl::SAC_RANSAC);
        //            seg.setDistanceThreshold (0.01);
        //            seg.setInputCloud (m_proc_cloud);
        //            seg.segment (*inliers, *coefficients);
        //            if (inliers->indices.size () == 0)
        //            {
        //                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        //            }
        //            std::cerr << "Model data size: " << m_proc_cloud->size() << std::endl;
        //            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        //                      << coefficients->values[1] << " "
        //                      << coefficients->values[2] << " "
        //                      << coefficients->values[3] << std::endl;
        //            std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
        //            // Extract found surface
        //            pcl::ExtractIndices<pcl::PointXYZ> extract;
        //            extract.setInputCloud (m_proc_cloud);
        //            extract.setIndices (inliers);
        //            extract.setNegative (false);
        //            extract.filter(*m_filtered_cloud);
        //            std::cerr << "Extracted surface: " << m_filtered_cloud->size() << std::endl;
        //            if (false) // This takes very long for huge data
        //            {
        //                // Create the KdTree object for the search method of the extraction
        //                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        //                tree->setInputCloud (m_filtered_cloud);
        //                // create the extraction object for the clusters
        //                std::vector<pcl::PointIndices> cluster_indices;
        //                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        //                // specify euclidean cluster parameters
        //                ec.setClusterTolerance (0.02); // 2cm
        //                ec.setMinClusterSize (100);
        //                ec.setMaxClusterSize (25000);
        //                ec.setSearchMethod (tree);
        //                ec.setInputCloud (m_filtered_cloud);
        //                // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        //                ec.extract (cluster_indices);
        //                std::cerr << "Found clusters: " << cluster_indices.size() << std::endl;
        //            }
        //        }
        //        else {
        //            std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_PROC));
        //        }
    }
    std::cout << "(PclInterface) Exiting thread" << std::endl;
}

PclInterface::PclInterface(std::vector<CameraType_t> device_types)
#if (PCL_VIEWER == 1)
    : m_pcl_viewer("Cloud Viewer")
    #endif
{
    for (auto& camtype : device_types)
    {
        m_input_clouds.push_back(new CloudQueue(camtype));
#if (PCL_VIEWER == 1)
        m_viewer_clouds.push_back(nullptr);
        m_pcl_viewer.runOnVisualizationThreadOnce (viewer_callback);
#endif
    }
}

std::vector<CloudQueue *>* PclInterface::getInputCloudsRef()
{
    return &m_input_clouds;
}

void PclInterface::setActive(bool running)
{
    if (running)
    {
        if ( m_pc_proc_thread.joinable() )
        {
            std::cerr << "(PclInterface) Thread already running: " << m_pc_proc_thread.get_id() << std::endl;
            return;
        }
        m_active = true;
        m_pc_proc_thread = std::thread(&PclInterface::pc_proc_thread_func, this);
#if (PCL_VIEWER == 1)
        // m_pcl_viewer.runOnVisualizationThreadOnce (viewer_callback);
        m_pcl_viewer.runOnVisualizationThread (viewer_update_callback, "ViewerCallback");
#endif

        //        while (!m_pcl_viewer.wasStopped ())
        //        {
        //            //  std::chrono::steady_clock::now();
        //            std::cout << "PCL viewer main" << std::endl;
        //            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //        }
    }
    else
    {
#if (PCL_VIEWER == 1)
        m_pcl_viewer.removeVisualizationCallable("ViewerCallback");
#endif
        m_active = false;
        if ( m_pc_proc_thread.joinable() )
            m_pc_proc_thread.join();
        else std::cerr << "(Converter) Thread not joinable: " << m_pc_proc_thread.get_id() << std::endl;
    }
    return;
}

bool PclInterface::isActive() { return m_active; }

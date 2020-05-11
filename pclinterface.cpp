#include "pclinterface.h"

//PclInterface::PclInterface()
//{

//}

void PclInterface::pc_proc_thread_func()
{
    std::cout << "PCL thread started # " << std::this_thread::get_id() << std::endl;
    //#if (PCL_VIEWER == 1)
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
    //#endif

    size_t max_times = 100;
    std::deque<long> proc_times;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

#if (PCL_FILTER_REGION > 0)
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-PCL_FILTER_REGION_X_M, -PCL_FILTER_REGION_Y_M, 0.0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(PCL_FILTER_REGION_X_M, PCL_FILTER_REGION_Y_M, PCL_FILTER_REGION_Z_M, 1.0));

    pcl::ConditionalRemoval<pcl::PointXYZ> conditionalRemoval;
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -PCL_FILTER_REGION_X_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, PCL_FILTER_REGION_X_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -PCL_FILTER_REGION_Y_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, PCL_FILTER_REGION_Y_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, PCL_FILTER_REGION_Z_M)));
#endif
#if (PCL_REMOVE_PLANE > 0)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
#endif


    while (m_active)
    {
#if (VERBOSE > 0)
        auto pcl_proc_start = std::chrono::high_resolution_clock::now();
#endif
        bool idle = true;
        for (size_t i = 0; i < m_input_clouds.size(); i++)
        {
            auto cloudqueue = m_input_clouds.at(i);
            if ( !cloudqueue->isEmpty() )
            {
                idle = false;
                auto camType = cloudqueue->getCameraType();
                auto cloudt = cloudqueue->getCloudT();
                pointcloud = std::get<0>(cloudt);
                auto ts = std::get<1>(cloudt);
                auto ctr = std::get<2>(cloudt);
#if (VERBOSE > 0)
                std::cout << "(PclInterface) PointCloud received from camera " << camType << "(" << ctr << "), size: " << pointcloud->size() << std::endl;
#endif

                // 1. pcl::PassThrough() +y = conveyor dir, z+ = conveyor dist, x = conveyor width
#if (PCL_FILTER_REGION > 10)
                if ((true)) // 32 - 35 ms (3xtotal)
                {
                    boxFilter.setInputCloud(pointcloud);
                    boxFilter.filter(*filtered);
                }
                if ((false))
                {
                    conditionalRemoval.setInputCloud(pointcloud);
                    conditionalRemoval.setCondition (range_cond);
                    conditionalRemoval.filter(*filtered);
                }
#endif


                // 2. remove planar surface
#if (PCL_REMOVE_PLANE > 0)  // > 2s (3xtotal)
                seg.setInputCloud (pointcloud);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                    PCL_ERROR ("Could not estimate a planar model for the given dataset.");

                // Remove found surface
                extract.setInputCloud (pointcloud);
                extract.setIndices (inliers);
                extract.setNegative (true);
                extract.filter(*filtered);

                std::cerr << "Model data size: " << filtered->size() << std::endl;
                std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                          << coefficients->values[1] << " "
                          << coefficients->values[2] << " "
                          << coefficients->values[3] << std::endl;
                std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

                if (false) // This takes very long for huge data
                {
                    // Create the KdTree object for the search method of the extraction
                    tree->setInputCloud (filtered);
                    // specify euclidean cluster parameters
                    ec.setClusterTolerance (0.02); // 2cm
                    ec.setMinClusterSize (100);
                    ec.setMaxClusterSize (25000);
                    ec.setSearchMethod (tree);
                    ec.setInputCloud (filtered);
                    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
                    ec.extract (cluster_indices);
                    std::cerr << "Found clusters: " << cluster_indices.size() << std::endl;
                }
#endif


                // 3. region growing
                // 4. Iterative Closest Point (registration or recognition)


#if (PCL_VIEWER == 1)
                //                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
                //                voxel_grid.setInputCloud (pointcloud);
                //                voxel_grid.setLeafSize (0.05, 0.05, 0.05);
                //                voxel_grid.filter(*cloud_downsampled);
                //                m_viewer_clouds.at(i) = cloud_downsampled;
                m_viewer_mtx.lock();
                m_viewer_clouds.at(i) = filtered;
                m_viewer_mtx.unlock();
#endif
            }

        }

        if (idle)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PCL_POLL_NS));
            continue;
        }
#if (VERBOSE > 0)
        auto pcl_proc_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(pcl_proc_end-pcl_proc_start).count();

        proc_times.push_back(duration);
        if (proc_times.size() > max_times)
            proc_times.pop_front();

        double average = std::accumulate(proc_times.begin(), proc_times.end(), 0.0) / proc_times.size() ;
        std::cout << "(PclInterface) Processing thread took (avg) " << std::fixed << std::setprecision(2) << average << " ms" << std::endl;
#endif
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

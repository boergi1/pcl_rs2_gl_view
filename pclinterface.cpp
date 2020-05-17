#include "pclinterface.h"

PclInterface::PclInterface(std::vector<CameraType_t> device_types)
#if (PCL_VIEWER == 1)
    : m_pcl_viewer("Cloud Viewer")
    #endif
{
    for (auto& camtype : device_types)
    {
#if RS_COLOR_ENABLED
        m_input_clouds.push_back(new CloudQueueRGB(camtype));
#else
        m_input_clouds.push_back(new CloudQueue(camtype));
#endif
#if PCL_VIEWER
        m_viewer_clouds.push_back(nullptr);
#endif
    }
#if PCL_VIEWER
        m_pcl_viewer.runOnVisualizationThreadOnce (viewer_callback);
#endif
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

void PclInterface::pc_proc_thread_func()
{
    std::cout << "PCL thread started # " << std::this_thread::get_id() << std::endl;
    //#if (PCL_VIEWER == 1)
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
    //#endif

    size_t max_times = 100;
    std::deque<long> proc_times;
#if RS_COLOR_ENABLED
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
#else
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
#endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_global_extr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_plane_extr(new pcl::PointCloud<pcl::PointXYZ>());


#if PCL_FILTER_GLOBAL_REGION_ENABLED
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //    boxFilter.setMin(Eigen::Vector4f(-PCL_GLOBAL_REGION_X_MIN_M, -PCL_GLOBAL_REGION_Y_MIN_M, PCL_GLOBAL_REGION_Z_MIN_M, 1.0));
    //    boxFilter.setMax(Eigen::Vector4f(PCL_GLOBAL_REGION_X_MAX_M, PCL_GLOBAL_REGION_Y_MAX_M, PCL_GLOBAL_REGION_Z_MAX_M, 1.0));
    boxFilter.setMin(Eigen::Vector4f(-1, -2, 0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(1, 2, 3, 1.0));

    pcl::ConditionalRemoval<pcl::PointXYZ> conditionalRemoval;
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, PCL_GLOBAL_REGION_X_MIN_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, PCL_GLOBAL_REGION_X_MAX_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, PCL_GLOBAL_REGION_Y_MIN_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, PCL_GLOBAL_REGION_Y_MAX_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, PCL_GLOBAL_REGION_Z_MIN_M)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, PCL_GLOBAL_REGION_Z_MAX_M)));
#endif

#if PCL_FILTER_PLANE_ENABLED
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // pcl::SACMODEL_PLANE
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (PCL_FILTER_PLANE_THRES);
    Eigen::Vector3f seg_vertical_axis;
    seg_vertical_axis << 0, 0, 1; // z
#endif

    bool savepcd = true;

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
                std::cout << "(PclInterface) PointCloud received from camera " << camType << " (" << ctr << "), organized: " << pointcloud->isOrganized()
                          << " size: " << pointcloud->points.size() << std::endl;
#endif                
                /*
                 *
                 *
                 *
                 */
                // 1. pcl::PassThrough() +y = conveyor dir, z+ = conveyor dist, x = conveyor width
#if PCL_FILTER_GLOBAL_REGION_ENABLED && !RS_COLOR_ENABLED
                if ((true)) // 32 - 35 ms (3xtotal)
                {
                    boxFilter.setInputCloud(pointcloud);
                    boxFilter.filter(*pc_global_extr);
                }
                if ((false))
                {
                    conditionalRemoval.setInputCloud(pointcloud);
                    conditionalRemoval.setCondition (range_cond);
                    conditionalRemoval.filter(*pc_global_extr);
                }
                std::cout << "(PclInterface) DEBUG after region camera " << camType << " size: " << pc_global_extr->points.size() << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;
#endif
                // 2. remove planar surface
                /* without axis: > 2s (3xtotal)
                 * with axis: > 200ms
                 *
                 *
                 */
#if PCL_FILTER_PLANE_ENABLED
#if !PCL_FILTER_GLOBAL_REGION_ENABLED
                seg.setInputCloud (pointcloud);
#else
                seg.setInputCloud (pc_global_extr);
#endif
                seg.setDistanceThreshold(PCL_FILTER_PLANE_THRES);
                seg.setAxis(seg_vertical_axis);
                seg.setEpsAngle(pcl::deg2rad(PCL_FILTER_PLANE_TOL_ANGLE));
                // coeff contains the coefficients of the plane:
                // ax + by + cz + d = 0
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                    PCL_ERROR ("Could not estimate a planar model for the given dataset.");

                std::cout << "(PclInterface) DEBUG after planar segmentation camera " << camType << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;

                // Remove found surface -> 25ms

                //#if PCL_FILTER_GLOBAL_REGION_ENABLED
                //                extract.setInputCloud (pc_global_extr);
                //#else
                //                extract.setInputCloud (pointcloud);
                //#endif
                extract.setInputCloud (seg.getInputCloud());
                extract.setIndices (inliers);
                extract.setNegative (true);
                extract.filter(*pc_plane_extr);
#if (VERBOSE > 1)
                std::cout << "Model data size: " << pc_global_extr->size() << std::endl;
                std::cout << "Model coefficients: " << coefficients->values[0] << " "
                          << coefficients->values[1] << " "
                          << coefficients->values[2] << " "
                          << coefficients->values[3] << std::endl;
                std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
#endif

                std::cout << "(PclInterface) DEBUG after planar extraction camera " << camType << " size: " << pc_plane_extr->points.size() << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;


#endif


                // 3. region growing / euclidean clusters
                // 4. Iterative Closest Point (registration or recognition) // correspondence grouping


#if (PCL_VIEWER == 1)
                m_viewer_mtx.lock();
#if PCL_FILTER_PLANE_ENABLED
                m_viewer_clouds.at(i) = pc_plane_extr;
#elif PCL_FILTER_GLOBAL_REGION_ENABLED && !RS_COLOR_ENABLED
                m_viewer_clouds.at(i) = pc_global_extr;
#else
                m_viewer_clouds.at(i) = pointcloud;
#endif
                m_viewer_mtx.unlock();
#endif
//                if (savepcd)
//                {
//                    if (camType == CameraType_t::CENTRAL)
//                    {
//                        savepcd = false;
//                        pcl::io::savePCDFile<pcl::PointXYZ>("/home/boergi/mypcd_orig.pcd", *pointcloud);
//                        pcl::io::savePCDFile<pcl::PointXYZ>("/home/boergi/mypcd_fil.pcd", *pc_plane_extr);
//                        std::cerr << "SAVED" << std::endl;
//                    }
//                }
                /*
                 *
                 *
                 *
                 */
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



//// REGISTRATRION EXAMPLE
//// Downsample
//  pcl::console::print_highlight ("Downsampling...\n");
//  pcl::VoxelGrid<PointNT> grid;
//  const float leaf = 0.2f;
//  grid.setLeafSize (leaf, leaf, leaf);
//  grid.setInputCloud (object);
//  grid.filter (*object);
//  grid.setInputCloud (scene);
//  grid.filter (*scene);
//  // Estimate normals for scene
//  pcl::console::print_highlight ("Estimating scene normals...\n");
//  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//  nest.setRadiusSearch (0.01);
//  nest.setInputCloud (scene);
//  nest.compute (*scene);
//  // Estimate features
//  pcl::console::print_highlight ("Estimating features...\n");
//  FeatureEstimationT fest;
//  fest.setRadiusSearch (0.01);
//  fest.setInputCloud (object);
//  fest.setInputNormals (object);
//  fest.compute (*object_features);
//  fest.setInputCloud (scene);
//  fest.setInputNormals (scene);
//  fest.compute (*scene_features);
//  // Perform alignment
//  pcl::console::print_highlight ("Starting alignment...\n");
//  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
//  align.setInputSource (object);
//  align.setSourceFeatures (object_features);
//  align.setInputTarget (scene);
//  align.setTargetFeatures (scene_features);
//  align.setMaximumIterations (100000); // Number of RANSAC iterations
//  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//  align.setCorrespondenceRandomness (4); // Number of nearest features to use
//  align.setSimilarityThreshold (0.6f); // Polygonal edge length similarity threshold
//  align.setMaxCorrespondenceDistance (1.4f * leaf); // Inlier threshold
//  align.setInlierFraction (0.15f); // Required inlier fraction for accepting a pose hypothesis

CloudQueue::CloudQueue(CameraType_t CameraType){
    m_camtype = CameraType;
}

void CloudQueue::addCloudT(std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> cloud_tuple)
{
    m_mtx.lock();
    m_cqueue.push_back(cloud_tuple);
    if (m_cqueue.size() > QUE_SIZE_PCL)
    {
        std::cerr << "(CloudDeque) Too many clouds in queue" << std::endl;
        m_cqueue.pop_front();
    }
    m_mtx.unlock();
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> CloudQueue::getCloudT()
{
    if (m_cqueue.size())
    {
        m_mtx.lock();
        auto tmp_ptr = m_cqueue.front();
        m_cqueue.pop_front();
        m_mtx.unlock();
        return tmp_ptr;
    }
    else
    {
        std::cerr << "(CloudDeque) is empty" << std::endl;
        return *m_cqueue.end();
        // return std::make_tuple(nullptr, NULL, NULL);
    }
}

const std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long, unsigned long long> CloudQueue::readCloudT()
{
    if (m_cqueue.size())
    {
        const auto cloud = m_cqueue.front();
        return cloud;
    }
    else
    {
        std::cerr << "(CloudDeque) is empty" << std::endl;
        return *m_cqueue.end();
    }
}

bool CloudQueue::isEmpty() { return m_cqueue.size() == 0; }

CameraType_t CloudQueue::getCameraType() { return m_camtype; }

CloudQueueRGB::CloudQueueRGB(CameraType_t CameraType){
    m_camtype = CameraType;
}

void CloudQueueRGB::addCloudT(std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, long long, unsigned long long> cloud_tuple)
{
    m_mtx.lock();
    m_cqueue.push_back(cloud_tuple);
    if (m_cqueue.size() > QUE_SIZE_PCL)
    {
        std::cerr << "(CloudDeque) Too many clouds in queue" << std::endl;
        m_cqueue.pop_front();
    }
    m_mtx.unlock();
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, long long, unsigned long long> CloudQueueRGB::getCloudT()
{
    if (m_cqueue.size())
    {
        m_mtx.lock();
        auto tmp_ptr = m_cqueue.front();
        m_cqueue.pop_front();
        m_mtx.unlock();
        return tmp_ptr;
    }
    else
    {
        std::cerr << "(CloudDeque) is empty" << std::endl;
        return *m_cqueue.end();
        // return std::make_tuple(nullptr, NULL, NULL);
    }
}

const std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, long long, unsigned long long> CloudQueueRGB::readCloudT()
{
    if (m_cqueue.size())
    {
        const auto cloud = m_cqueue.front();
        return cloud;
    }
    else
    {
        std::cerr << "(CloudDeque) is empty" << std::endl;
        return *m_cqueue.end();
    }
}

bool CloudQueueRGB::isEmpty() { return m_cqueue.size() == 0; }

CameraType_t CloudQueueRGB::getCameraType() { return m_camtype; }

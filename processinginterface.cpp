#include "processinginterface.h"

ProcessingInterface::ProcessingInterface(std::vector<CameraType_t> device_types, MainWindowGL* Window)
#if (PCL_VIEWER == 1)
    : m_pcl_viewer("Cloud Viewer")
    #endif
{
    _WindowRef = Window;
    for (auto& camtype : device_types)
    {
        //#if PROC_PIPE_PC_ENABLED
        m_input_clouds.push_back(new CloudQueue(camtype));
        //#endif
#if PROC_PIPE_MAT_ENABLED
        m_input_depth.push_back(new MatQueue(camtype, "depth"));
#if RS_COLOR_ENABLED
        m_input_color.push_back(new MatQueue(camtype, "color"));
#endif
#endif

#if PCL_VIEWER
        m_viewer_clouds.push_back(nullptr);
#endif
    }
#if PCL_VIEWER
    m_pcl_viewer.runOnVisualizationThreadOnce (viewer_callback);
#endif
}

void ProcessingInterface::setActive(bool running)
{
    if (running)
    {
        if ( m_pc_proc_thread.joinable() )
        {
            std::cerr << "(ProcessingInterface) Thread already running: " << m_pc_proc_thread.get_id() << std::endl;
            return;
        }
        m_active = true;
        m_pc_proc_thread = std::thread(&ProcessingInterface::pc_proc_thread_func, this);
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

bool ProcessingInterface::isActive() { return m_active; }

void ProcessingInterface::pc_proc_thread_func()
{
    std::cout << "Proc thread started # " << std::this_thread::get_id() << std::endl;
    //#if (PCL_VIEWER == 1)
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
    //#endif

    size_t max_times = 100;
    std::deque<long> proc_times;

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_global_extr(new pcl::PointCloud<pcl::PointXYZ>());
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_plane_extr(new pcl::PointCloud<pcl::PointXYZ>());


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

    bool savepcd = false;
    bool executeOnce = true;

    while (m_active)
    {
#if VERBOSE
        auto proc_start = std::chrono::high_resolution_clock::now();
#endif
        bool idle = true;

#if PROC_PIPE_MAT_ENABLED

        for (size_t i = 0; i < m_input_depth.size(); i++)
        {
            auto depthqueue = m_input_depth.at(i);
            if ( !depthqueue->isEmpty() )
            {
                idle = false;
                auto camType = depthqueue->getCameraType();
                auto mat_t_depth = depthqueue->getMatT();
                auto count = std::get<0>(mat_t_depth);
                auto mat_depth = std::get<1>(mat_t_depth);
                auto ts = std::get<2>(mat_t_depth);
#if (VERBOSE > 0)
                std::cout << "(ProcessingInterface) Depth image received from camera " << camType << " (" << count << ") size: " << mat_depth.size() << std::endl;
#endif
                if ((false))
                {
                    //  cv::applyColorMap(mat_depth, mat_depth, cv::COLORMAP_JET);
                    double min, max;
                    cv::minMaxLoc(mat_depth, &min, &max);
                    auto test = std::numeric_limits<unsigned short>::max();
                    double scale = test / max;
                    cv::imshow("depth "+std::to_string(camType), mat_depth*scale);
                    // cv::waitKey(1);
                }

                if ((false))
                {
                    if (executeOnce && camType == CameraType_t::CENTRAL)
                    {
                        //   pointcloud->
                        executeOnce = false;

                    }
                }


            }
        }
#if RS_COLOR_ENABLED
        for (size_t i = 0; i < m_input_color.size(); i++)
        {
            auto colorqueue = m_input_color.at(i);
            if ( !colorqueue->isEmpty() )
            {
                idle = false;
                auto camType = colorqueue->getCameraType();
                auto mat_t_color = colorqueue->getMatT();
                auto count = std::get<0>(mat_t_color);
                auto mat_color = std::get<1>(mat_t_color);
                auto ts = std::get<2>(mat_t_color);
#if (VERBOSE > 0)
                std::cout << "(ProcessingInterface) Color image received from camera " << camType << " (" << count << ") size: " << mat_color.size() << std::endl;
#endif
                //  cv::imshow("color "+std::to_string(camType), mat_color);
            }
        }
#endif
        //  cv::waitKey(1);
#endif


        for (size_t i = 0; i < m_input_clouds.size(); i++)
        {
            auto cloudqueue = m_input_clouds.at(i);
            if ( !cloudqueue->isEmpty() )
            {
                idle = false;
                auto camType = cloudqueue->getCameraType();
                auto cloudt = cloudqueue->getCloudT();
                auto pointcloud = (std::get<1>(cloudt));
                auto ts = std::get<1>(cloudt);
                auto ctr = std::get<2>(cloudt);
#if (VERBOSE > 0)
                std::cout << "(ProcessingInterface) PointCloud received from camera " << camType << " (" << ctr
                          << ") length: " << pointcloud->size()/3 << " size: " << pointcloud->size() << std::endl;
#endif

#if PCL_VIEWER
                if (camType == CameraType_t::REAR)
                {
                    pcl::PointCloud <pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud <pcl::PointXYZ>(RS_FRAME_WIDTH_DEPTH,RS_FRAME_HEIGHT_DEPTH));
                    for (size_t y=0;y<pcl_cloud->height;y++)
                        for (size_t x=0;x<pcl_cloud->width;x++)
                        {
                            size_t i = y*pcl_cloud->width + x;
                            pcl_cloud->at(x,y).x = pointcloud->at(i).x();
                            pcl_cloud->at(x,y).y = pointcloud->at(i).y();
                            pcl_cloud->at(x,y).z = pointcloud->at(i).z();
                        }

                    //                    for (size_t i=0;i<pointcloud->size();i++)
                    //                    {
                    //                        pcl_cloud->at(i).x = pointcloud->at(i).x();
                    //                        pcl_cloud->at(i).y = pointcloud->at(i).y();
                    //                        pcl_cloud->at(i).z = pointcloud->at(i).z();
                    //                    }
                    m_viewer_mtx.lock();
                    m_viewer_clouds.at(i) = pcl_cloud;
                    m_viewer_mtx.unlock();
                }
#endif

                // euclideanUnionFind(pointcloud);




                if ((true))
                {
                    if (executeOnce && camType == CameraType_t::REAR)
                    {
                        if (_WindowRef->isActive())
                        {
//                            for (size_t i=0; i<pointcloud->size(); i++)
//                                std::cout << "i " << pointcloud->at(i) << std::endl;
                            executeOnce = false;
                            std::cout << "executeOnce" << std::endl;
                            _WindowRef->setVerticesBuffer(0, pointcloud);
                        }

                        // euclideanDepthFirstSearch(pointcloud);
                        // euclideanUnionFind(pointcloud);
                        // euclideanConnectedComponentsOrganized(pointcloud);


                        //                        pcl::IndicesPtr indices (new std::vector <int>);
                        //                        pcl::PassThrough<pcl::PointXYZ> pass;
                        //                        pass.setInputCloud (cloud);
                        //                        pass.setFilterFieldName ("z");
                        //                        pass.setFilterLimits (0.0, 1.0);
                        //                        pass.filter (*indices);
                        //                        pcl::MinCutSegmentation<pcl::PointXYZ> seg;
                        //                        seg.setInputCloud (cloud);
                        //                        seg.setIndices (indices);
                        //                        pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
                        //                        pcl::PointXYZ point;
                        //                        point.x = 68.97;
                        //                        point.y = -18.55;
                        //                        point.z = 0.57;
                        //                        foreground_points->points.push_back(point);
                        //                        seg.setForegroundPoints (foreground_points);
                        //                        seg.setSigma (0);
                        //                        seg.setRadius (0.01);
                        //                        seg.setNumberOfNeighbours (0);
                        //                        seg.setSourceWeight (0.3);
                        //                        std::vector <pcl::PointIndices> clusters;
                        //                        seg.extract (clusters);
                        //                        std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;
                        //                        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
                        //                        pcl::visualization::CloudViewer viewer ("Cluster viewer");
                        //                        viewer.showCloud(colored_cloud);
                        //                        while (!viewer.wasStopped ())
                        //                        {
                        //                        }

                    }
                }

                delete pointcloud;


            }
        }







        //#if PROC_PIPE_PC_ENABLED
        //        for (size_t i = 0; i < m_input_clouds.size(); i++)
        //        {
        //            auto cloudqueue = m_input_clouds.at(i);
        //            if ( !cloudqueue->isEmpty() )
        //            {
        //                idle = false;
        //                auto camType = cloudqueue->getCameraType();
        //                auto cloudt = cloudqueue->getCloudT();
        //                pointcloud = std::get<0>(cloudt);
        //                auto ts = std::get<1>(cloudt);
        //                auto ctr = std::get<2>(cloudt);
        //#if (VERBOSE > 0)
        //                std::cout << "(ProcessingInterface) PointCloud received from camera " << camType << " (" << ctr << "), organized: " << pointcloud->isOrganized()
        //                          << " size: " << pointcloud->points.size() << std::endl;
        //#endif
        //                /*
        //                 *
        //                 *
        //                 *
        //                 */
        //                // 1. pcl::PassThrough() +y = conveyor dir, z+ = conveyor dist, x = conveyor width
        //#if PCL_FILTER_GLOBAL_REGION_ENABLED
        //                if ((true)) // 32 - 35 ms (3xtotal)
        //                {
        //                    boxFilter.setInputCloud(pointcloud);
        //                    boxFilter.filter(*pc_global_extr);
        //                }
        //                if ((false))
        //                {
        //                    conditionalRemoval.setInputCloud(pointcloud);
        //                    conditionalRemoval.setCondition (range_cond);
        //                    conditionalRemoval.filter(*pc_global_extr);
        //                }
        //                std::cout << "(ProcessingInterface) DEBUG after region camera " << camType << " size: " << pc_global_extr->points.size() << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
        //                             (std::chrono::high_resolution_clock::now()-proc_start).count() << " ms" << std::endl;
        //#endif
        //                // 2. remove planar surface
        //                /* without axis: > 2s (3xtotal)
        //                 * with axis: > 200ms
        //                 *
        //                 *
        //                 */
        //#if PCL_FILTER_PLANE_ENABLED
        //#if !PCL_FILTER_GLOBAL_REGION_ENABLED
        //                seg.setInputCloud (pointcloud);
        //#else
        //                seg.setInputCloud (pc_global_extr);
        //#endif
        //                seg.setDistanceThreshold(PCL_FILTER_PLANE_THRES);
        //                seg.setAxis(seg_vertical_axis);
        //                seg.setEpsAngle(pcl::deg2rad(PCL_FILTER_PLANE_TOL_ANGLE));
        //                // coeff contains the coefficients of the plane:
        //                // ax + by + cz + d = 0
        //                seg.segment (*inliers, *coefficients);
        //                if (inliers->indices.size() == 0)
        //                    PCL_ERROR ("Could not estimate a planar model for the given dataset.");

        //                std::cout << "(ProcessingInterface) DEBUG after planar segmentation camera " << camType << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
        //                             (std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;

        //                // Remove found surface -> 25ms

        //                //#if PCL_FILTER_GLOBAL_REGION_ENABLED
        //                //                extract.setInputCloud (pc_global_extr);
        //                //#else
        //                //                extract.setInputCloud (pointcloud);
        //                //#endif
        //                extract.setInputCloud (seg.getInputCloud());
        //                extract.setIndices (inliers);
        //                extract.setNegative (true);
        //                extract.filter(*pc_plane_extr);
        //#if (VERBOSE > 1)
        //                std::cout << "Model data size: " << pc_global_extr->size() << std::endl;
        //                std::cout << "Model coefficients: " << coefficients->values[0] << " "
        //                          << coefficients->values[1] << " "
        //                          << coefficients->values[2] << " "
        //                          << coefficients->values[3] << std::endl;
        //                std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
        //#endif

        //                std::cout << "(ProcessingInterface) DEBUG after planar extraction camera " << camType << " size: " << pc_plane_extr->points.size() << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
        //                             (std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;


        //#endif


        //                // 3. region growing / euclidean clusters
        //                // 4. Iterative Closest Point (registration or recognition) // correspondence grouping


        //#if (PCL_VIEWER == 1)
        //                m_viewer_mtx.lock();
        //#if PCL_FILTER_PLANE_ENABLED
        //                m_viewer_clouds.at(i) = pc_plane_extr;
        //#elif PCL_FILTER_GLOBAL_REGION_ENABLED
        //                m_viewer_clouds.at(i) = pc_global_extr;
        //#else
        //                m_viewer_clouds.at(i) = pointcloud;
        //#endif
        //                m_viewer_mtx.unlock();
        //#endif
        //                //                if (savepcd)
        //                //                {
        //                //                    if (camType == CameraType_t::CENTRAL)
        //                //                    {
        //                //                        savepcd = false;
        //                //                        pcl::io::savePCDFile<pcl::PointXYZ>("/home/boergi/mypcd_orig.pcd", *pointcloud);
        //                //                        pcl::io::savePCDFile<pcl::PointXYZ>("/home/boergi/mypcd_fil.pcd", *pc_plane_extr);
        //                //                        std::cerr << "SAVED" << std::endl;
        //                //                    }
        //                //                }
        //                /*
        //                 *
        //                 *
        //                 *
        //                 */
        //            }
        //        }
        //#endif












        if (idle)
        {
#if (VERBOSE > 2)
            std::cerr << "(ProcessingInterface) Idle" << std::endl;
#endif
            std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PCL_POLL_NS));
            continue;
        }
#if (VERBOSE > 0)
        auto proc_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(proc_end-proc_start).count();

        proc_times.push_back(duration);
        if (proc_times.size() > max_times)
            proc_times.pop_front();

        double average = std::accumulate(proc_times.begin(), proc_times.end(), 0.0) / proc_times.size() ;
        std::cout << "(ProcessingInterface) Processing thread took (avg) " << std::fixed << std::setprecision(2) << average << " ms" << std::endl;
#endif
    }
    std::cout << "(ProcessingInterface) Exiting thread" << std::endl;
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

CloudQueue::CloudQueue(CameraType_t CameraType, std::string name){
    m_camtype = CameraType;
    m_name = name;
}

void CloudQueue::addCloudT(std::tuple<unsigned long long, std::vector< float >*, long long> cloud_tuple)
{
    m_mtx.lock();
    m_cqueue.push_back(cloud_tuple);
    if (m_cqueue.size() > QUE_SIZE_PCL)
    {
#if VERBOSE
        std::cerr << "(CloudQueue) Too many clouds in queue " << m_name << " " << m_camtype << std::endl;
#endif
        auto tmpptr = std::get<1>(m_cqueue.front());
        m_cqueue.pop_front();
        delete tmpptr;
    }
    m_mtx.unlock();
}

std::tuple<unsigned long long, std::vector< float >*, long long> CloudQueue::getCloudT()
{
    if (m_cqueue.size())
    {
        m_mtx.lock();
        auto cloud_t = m_cqueue.front();
        m_cqueue.pop_front();
        m_mtx.unlock();
        return cloud_t;
    }
    else
    {
        std::cerr << "(CloudQueue) is empty " << m_name << " " << m_camtype << std::endl;
        return *m_cqueue.end();
        // return std::make_tuple(nullptr, NULL, NULL);
    }
}

const std::tuple<unsigned long long, std::vector< float >*, long long> CloudQueue::readCloudT()
{
    if (m_cqueue.size())
    {
        const auto cloud_t = m_cqueue.front();
        return cloud_t;
    }
    else
    {
        std::cerr << "(CloudDeque) is empty" << std::endl;
        return *m_cqueue.end();
    }
}

bool CloudQueue::isEmpty() { return m_cqueue.size() == 0; }

CameraType_t CloudQueue::getCameraType() { return m_camtype; }

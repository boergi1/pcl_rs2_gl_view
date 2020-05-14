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


#if PCL_FILTER_REGION
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
    //#if (PCL_REMOVE_PLANE > 0)

#define PCL_SEGM_PLANE 0
#if PCL_SEGM_PLANE
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
#define PCL_NORM_SEGM 1
#if PCL_NORM_SEGM

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_n;

    //    //   pcl::search::Search<pcl::PointXYZ>::Ptr tree_n;
    //    std::vector<pcl::PointIndices> cluster_indices;
    //    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

#endif
#define PCL_DON_SEGM 0
#if PCL_DON_SEGM
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZ> ne_omp; // undefined reference to computeFeature
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_don;
    ne_omp.setNumberOfThreads(8);

#endif
#define PCL_AXIS_SEGM 0
#if PCL_AXIS_SEGM
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // pcl::SACMODEL_PLANE
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    Eigen::Vector3f seg_vertical_axis;
    seg_vertical_axis << 0, 0, 1;
#endif
#endif

    bool savepcd = false;

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

                // 1. pcl::PassThrough() +y = conveyor dir, z+ = conveyor dist, x = conveyor width
#if PCL_FILTER_REGION
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


                std::cout << "(PclInterface) DEBUG before planar " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;

                if (savepcd)
                {
                    savepcd = false;
                    pcl::io::savePCDFile<pcl::PointXYZ>("/home/boergi/mypcd.pcd", *pointcloud);
                    std::cerr << "SAVED" << std::endl;
                }


                //                // Create the normal estimation class, and pass the input dataset to it
                //                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
                //                normalEstimation.setInputCloud (pointcloud);
                //                // Create an empty kdtree representation, and pass it to the normal estimation object.
                //                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                //                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                //                normalEstimation.setSearchMethod (tree);
                //                // Output datasets
                //                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
                //                // Use all neighbors in a sphere of radius 0.03 = 3cm
                //                normalEstimation.setRadiusSearch (0.003);
                //                // Compute the features
                //                normalEstimation.compute (*cloud_normals);

                // 2. remove planar surface
#if PCL_SEGM_PLANE


                //               // pcl::SACSegmentationFromNormals()


                // Estimate point normals

#if PCL_DON_SEGM

                //The smallest scale to use in the DoN filter.
                double scale1 = 0.01;
                //The largest scale to use in the DoN filter.
                double scale2 = 0.1;
                //The minimum DoN magnitude to threshold by
                double threshold;
                //segment scene into clusters with given distance tolerance using euclidean clustering
                double segradius;

                if (pointcloud->isOrganized ())
                {
                    tree_don.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
                }
                else
                {
                    tree_don.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
                }
                tree_don->setInputCloud(pointcloud);

                ne_omp.setInputCloud(pointcloud);
                ne_omp.setSearchMethod(tree_don);
                /*
                 * NOTE: setting viewpoint is very important, so that we can ensure
                 * normals are all pointed in the same direction!
                 */
                ne_omp.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

                // calculate normals with the small scale
                pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

                //                pcl::PointCloud<pcl::PointNormal> normals_small_scale;

                ne_omp.setRadiusSearch (scale1);
                ne_omp.compute (*normals_small_scale);

                // calculate normals with the large scale
                pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

                ne_omp.setRadiusSearch (scale2);
                ne_omp.compute (*normals_large_scale);



                //                pcl::NormalEstimationOMP< pcl::PointXYZ, pcl::PointXYZ >::PointCloudOut::Ptr omp_output (pcl::NormalEstimationOMP< pcl::PointXYZ, pcl::PointXYZ >::PointCloudOut);

                //                pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
                //                ne_omp.setRadiusSearch (0.01);
                //                // ne_omp.compute (*normals_small_scale);
                //                ne_omp.compute (omp_output);


#endif

                //                ne.setSearchMethod (tree);
                //                ne.setInputCloud (pointcloud);
                //                ne.setKSearch (8);
                //                ne.compute (*cloud_normals);


#if PCL_NORM_SEGM
                ne.setInputCloud(pointcloud);
                ne.setSearchMethod(tree_n);
                ne.setRadiusSearch(0.03);
                ne.compute(*cloud_normals);


                std::cout << "(PclInterface) DEBUG after normals " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms"
                          << " size: " << cloud_normals->size() << std::endl;

                // Create the segmentation object for the planar model and set all the parameters
                seg_n.setOptimizeCoefficients (true);
                seg_n.setModelType (pcl::SACMODEL_NORMAL_PLANE);
                seg_n.setNormalDistanceWeight (0.1);
                seg_n.setMethodType (pcl::SAC_RANSAC);
                seg_n.setMaxIterations (100);
                seg_n.setDistanceThreshold (0.03);
                seg_n.setInputCloud (pointcloud);
                seg_n.setInputNormals (cloud_normals);
                // Obtain the plane inliers and coefficients
                seg_n.segment (*inliers, *coefficients);
#endif

                std::cout << "(PclInterface) DEBUG after segmentation " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms"
                          << " size: " << inliers->indices.size() << std::endl;

#if PCL_AXIS_SEGM
                /* without axis: > 2s (3xtotal)
                 * with axis: > 200ms
                 *
                 *
                 */
                seg.setInputCloud (pointcloud);
                seg.setDistanceThreshold(0.01);
                seg.setAxis(seg_vertical_axis);
                seg.setEpsAngle(pcl::deg2rad(10.0));
                // coeff contains the coefficients of the plane:
                // ax + by + cz + d = 0
                seg.segment (*inliers, *coefficients);
#endif




                if (inliers->indices.size() == 0)
                    PCL_ERROR ("Could not estimate a planar model for the given dataset.");

                std::cout << "(PclInterface) DEBUG after planar extraction " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;

                // Remove found surface -> 25ms
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

                //                if (false) // This takes very long for huge data
                //                {
                //                    // Create the KdTree object for the search method of the extraction
                //                    tree->setInputCloud (filtered);
                //                    // specify euclidean cluster parameters
                //                    ec.setClusterTolerance (0.02); // 2cm
                //                    ec.setMinClusterSize (100);
                //                    ec.setMaxClusterSize (25000);
                //                    ec.setSearchMethod (tree);
                //                    ec.setInputCloud (filtered);
                //                    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
                //                    ec.extract (cluster_indices);
                //                    std::cerr << "Found clusters: " << cluster_indices.size() << std::endl;
                //                }
                std::cout << "(PclInterface) DEBUG after remove " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-pcl_proc_start).count() << " ms" << std::endl;
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
                m_viewer_clouds.at(i) = pointcloud;
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

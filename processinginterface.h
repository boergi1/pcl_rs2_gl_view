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
    void addCloudT(std::tuple <unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long> cloud_tuple);
    // <std::tuple<unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>>
    std::tuple <unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long> getCloudT();
    const std::tuple <unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long> readCloudT();
    bool isEmpty();
    CameraType_t getCameraType();
private:
    std::deque<std::tuple <unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>>  m_cqueue;
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
#if VERBOSE
            std::cerr << "(MatQueue) Too many Mats in queue " << m_name << " " << m_camtype << std::endl;
#endif
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

class TrackedObject
{
public:
    TrackedObject(int ID)
    {
        _ID = ID;
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud (new pcl::PointCloud<pcl::PointXYZ>());
        _objectCloud = objectCloud;
    }

    void addPoint(pcl::PointXYZ point) { _objectCloud->push_back(point); }

    void addOrganizedPoint(pcl::PointXYZ point, int index_X, int index_Y )
    {
        _objectCloud->push_back(point);
        _pointIndices.push_back(std::make_pair(index_X, index_Y));
    }

    pcl::PointXYZ* getPoint () { return &_objectCloud->points.back(); }
    pcl::PointXYZ* getOrganizedPoint (int index_X, int index_Y)
    {
        static double overall_duration = 0;
        auto start = std::chrono::high_resolution_clock::now();
        auto pointSize = _pointIndices.size();
        for (size_t i = 0; i < pointSize; i++) {
            if ( index_X == _pointIndices.at(i).first && index_Y == _pointIndices.at(i).second )
            {
                auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-start).count();
                overall_duration = overall_duration + duration;
                std::cout << "DEBUG getOrganizedPoint took: " << duration/1e6 << " ms, TOTAL " << overall_duration/1e6 << std::endl;
                return &_objectCloud->at(i);
            }
        }
        return nullptr;
    }

    pcl::PointXYZ* getPointByIndex (int index) { return &_objectCloud->points.at(index); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() { return _objectCloud; }
    int getID() { return _ID; }
    /* todo: add cloud and group to existing cloud,
     * calculate center of unorganized cloud
        OR do this with organized cloud */
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _objectCloud;
    std::vector< std::pair<int,int> > _pointIndices;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr _objectCloud (new pcl::PointCloud<pcl::PointXYZ>);
    int _ID;
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


    //                            cv::Mat centroidDistances(std::vector<tracked_object_t> *obj_centr, tracked_object_t *inp_centr, size_t size_in)
    //                            {
    //                                size_t size_obj = obj_centr->size();
    //                                cv::Mat result = cv::Mat::zeros(static_cast<int>(size_obj), static_cast<int>(size_in), CV_64F);
    //                                for (size_t i = 0; i < size_obj; i++)
    //                                {
    //                                    double x_o = obj_centr->at(i).cx;
    //                                    double y_o = obj_centr->at(i).cy;
    //                                    for (size_t j = 0; j < size_in; ++j)
    //                                    {
    //                                        double dx = inp_centr[j].cx - x_o;
    //                                        double dy = inp_centr[j].cy - y_o;
    //                                        result.at<double>(static_cast<int>(i), static_cast<int>(j)) = std::sqrt(dx * dx + dy * dy);
    //                                    }
    //                                }
    //                                return result;
    //                            }

    float euclideanDistance3D(pcl::PointXYZ* point_1, pcl::PointXYZ* point_2)
    {
        float dx = point_2->x - point_1->x;
        float dy = point_2->y - point_1->y;
        float dz = point_2->z - point_1->z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<TrackedObject*> euclideanConnectedComponentsOrganized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radiusThreshold, float distanceThreshold)
    {
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<TrackedObject*> objectClouds;


        uint16_t cluster_id = 1;

        //  cv::Mat labels = cv::Mat(height, width, CV_32SC1, cv::Scalar(-1)); // 32bit signed
        cv::Mat labels = cv::Mat::zeros(cloud->height, cloud->width, CV_16UC1);

        pcl::PointXYZ tmp_point;

        int outOfRange = 0;

        std::cout << "euclideanConnectedComponentsOrganized " << labels.rows << "x" << labels.cols <<  std::endl;

        for (int row = 0; row < labels.rows; row++)
        {
            for (int col = 0; col < labels.cols; col++)
            {
                tmp_point = cloud->at(col,row);

                // Check for valid point
                 if ((tmp_point.z > 0) && (tmp_point.z < distanceThreshold))
              //  if (tmp_point.z > 0)
                {
                    int upper_row = row - 1;
                    int left_col = col - 1;
                    bool new_object = false;
                    TrackedObject* neighborObject = nullptr;
                //    pcl::PointXYZ* neighbor_point_ptr = nullptr;
                    pcl::PointXYZ neighbor_point;

                    if (upper_row < 0)
                    {
                        // First pixel
                        if (left_col < 0)
                        {
                            new_object = true;
                        }
                        else
                        {
                            uint16_t neighbor_label = labels.at<uint16_t>(row, left_col);
                            // First row - check left neighbor
                            if (neighbor_label > 0)
                            {
                             //   std::cout << "DEBUG found neighbor LEFT (first row) " << row << " " << left_col << std::endl;
                                neighborObject = objectClouds.at(neighbor_label-1);
                              //  neighbor_point_ptr = neighborObject->getOrganizedPoint(left_col, row);//(row, left_col);
                                neighbor_point = cloud->at(left_col, row);
                            }
                            else
                                new_object = true;
                        }
                    }
                    else if (left_col < 0)
                    {
                        // First column - check upper neighbor
                        uint16_t neighbor_label = labels.at<uint16_t>(upper_row, col);
                        if (neighbor_label > 0)
                        {
                        //    std::cout << "DEBUG found neighbor UPPER (first column) " << upper_row << " " << col << std::endl;
                            neighborObject = objectClouds.at(neighbor_label-1);
                        //    neighbor_point_ptr = neighborObject->getOrganizedPoint(col, upper_row);//(upper_row, col);
                            neighbor_point = cloud->at(col, upper_row);
                        }
                        else
                            new_object = true;
                    }
                    else
                    {
                        // Rest - check upper first, then left
                        uint16_t neighbor_label_up = labels.at<uint16_t>(upper_row, col);
                        if (neighbor_label_up > 0)
                        {
                         //   std::cout << "DEBUG found neighbor UPPER (body) " << upper_row << " " << col << std::endl;
                            neighborObject = objectClouds.at(neighbor_label_up-1);
                           // neighbor_point_ptr = neighborObject->getOrganizedPoint(col, upper_row);//(upper_row, col);
                            neighbor_point = cloud->at(col, upper_row);
                        }
                        else
                        {
                            uint16_t neighbor_label_left = labels.at<uint16_t>(row, left_col);
                            if (neighbor_label_left > 0)
                            {
                            //    std::cout << "DEBUG found neighbor LEFT (body) " << row << " " << left_col << std::endl;
                                neighborObject = objectClouds.at(neighbor_label_left-1);
                              //  neighbor_point_ptr = neighborObject->getOrganizedPoint(left_col, row);//(row, left_col);
                                neighbor_point = cloud->at(left_col, row);
                            }
                            else
                                new_object = true;
                        }
                    }


                    if (new_object)
                    {
                        // Add new object
                        objectClouds.push_back(new TrackedObject(cluster_id));
                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
                        labels.at<uint16_t>(row, col) = cluster_id;
                    //    std::cout << "DEBUG added NEW object with point: [" << labels.at<uint16_t>(row, col) << "] " << row << " " << col << " " << tmp_point << std::endl;
                        cluster_id++;
                    }
                    else
                    {
                        // Add point to existing object
                        if (neighbor_point.z)
                        {
                            if ( euclideanDistance3D(&neighbor_point, &tmp_point) < radiusThreshold )
                            {
                                neighborObject->addOrganizedPoint(tmp_point, col, row);
                                labels.at<uint16_t>(row, col) = neighborObject->getID();
                      //          std::cout << "DEBUG added EXISTING point to object: [" << labels.at<uint16_t>(row, col) << "] " << row << " " << col << " " << tmp_point << std::endl;
                            }
                        }
                        else std::cerr << "DEBUG got no neighbor point from row " << row << " col " << col << std::endl;
                    }
                }
                else
                    outOfRange++;
            }
        }
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
        std::cout << std::endl << "DEBUG objectClouds " << objectClouds.size() << " out of range: " << outOfRange << " took: " << duration << std::endl << std::endl;
    //     std::cout << std::endl << "DEBUG label matrix " << labels.rows << " " << labels.cols << std::endl << labels << std::endl << std::endl;

        cv::Mat show;
        labels.copyTo(show);
        show.convertTo(show, CV_8UC1);
        double min, max;
        cv::minMaxIdx(show, &min, &max);
        double scale = std::numeric_limits<uint8_t>::max() / max;
        cv::applyColorMap( show*scale, show, cv::COLORMAP_JET);
        cv::imshow("components", show);
        cv::waitKey(0);

        return objectClouds;
    }

    //    std::vector<TrackedObject*> euclideanConnectedComponentsOrganized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radiusThreshold, float distanceThreshold)
    //    {
    //        std::cout << "euclideanConnectedComponentsOrganized" <<  std::endl;
    //        std::vector<TrackedObject*> objectClouds;

    //        auto width = cloud->width;
    //        auto height = cloud->height;
    //        uint16_t cluster_id = 0;

    //        cv::Mat labels = cv::Mat(height, width, CV_32SC1, cv::Scalar(-1));

    //        pcl::PointXYZ tmp_point;
    //        int outOfRange = 0;

    //        for (size_t row = 0; row < height; row++)
    //        {
    //            for (size_t col = 0; col < width; col++)
    //            {
    //                tmp_point = cloud->at(col,row);

    //                if ((tmp_point.z > 0) && (tmp_point.z < distanceThreshold))
    //                {
    //                    std::cout << "DEBUG process: " << (row == 0) << " " << (col == 0) << std::endl;
    //                    if (row == 0)
    //                    {
    //                        if (col == 0)
    //                        {
    //                            // First pixel - new label
    //                            objectClouds.push_back(new TrackedObject(cluster_id));
    //                            objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
    //                            labels.at<int32_t>(row, col) = cluster_id;
    //                            std::cout << "DEBUG added first point: " << cluster_id << " "  << tmp_point << std::endl;
    //                            cluster_id++;
    //                        }
    //                        else
    //                        {
    //                            // First row
    //                            int32_t neighbor_label = labels.at<int32_t>(row, col-1);
    //                            if (neighbor_label >= 0)
    //                            {
    //                                auto neighbor_point = objectClouds.at(neighbor_label)->getOrganizedPoint(row, col-1);
    //                                if (neighbor_point)
    //                                {
    //                                    if ( radiusThreshold > euclideanDistance3D(neighbor_point, &tmp_point) )
    //                                    {
    //                                        // Existing label
    //                                        objectClouds.at(neighbor_label)->addOrganizedPoint(tmp_point, col, row);
    //                                        labels.at<int32_t>(row, col) = objectClouds.at(neighbor_label)->getID();
    //                                        std::cout << "DEBUG added EXISTING first row point: " << objectClouds.at(neighbor_label)->getID() << " " << tmp_point << std::endl;
    //                                    }
    //                                    else
    //                                    {
    //                                        // New label
    //                                        objectClouds.push_back(new TrackedObject(cluster_id));
    //                                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
    //                                        labels.at<int32_t>(row, col) = cluster_id;
    //                                        std::cout << "DEBUG added NEW first row point: " << cluster_id << " " << tmp_point << std::endl;
    //                                        cluster_id++;
    //                                    }

    //                                }
    //                            }

    //                        }
    //                    }
    //                    else
    //                    {
    //                        // Check upper neighbor
    //                        int32_t neighbor_label_up = labels.at<int32_t>(row-1, col);
    //                        bool found_upper = false;
    //                        if (neighbor_label_up >= 0)
    //                        {
    //                            auto neighbor_point_up = objectClouds.at(neighbor_label_up)->getOrganizedPoint(row-1, col);
    //                            if (neighbor_point_up)
    //                            {
    //                                found_upper = true;
    //                                if ( radiusThreshold > euclideanDistance3D(neighbor_point_up, &tmp_point) )
    //                                {
    //                                    // Existing label
    //                                    objectClouds.at(neighbor_label_up)->addOrganizedPoint(tmp_point, col, row);
    //                                    labels.at<int32_t>(row, col) = objectClouds.at(neighbor_label_up)->getID();
    //                                    std::cout << "DEBUG added EXISTING upper point: " << objectClouds.at(neighbor_label_up)->getID() << " " << tmp_point << std::endl;
    //                                }
    //                                else
    //                                {
    //                                    // New label
    //                                    objectClouds.push_back(new TrackedObject(cluster_id));
    //                                    objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
    //                                    labels.at<int32_t>(row, col) = cluster_id;
    //                                    std::cout << "DEBUG added NEW upper point: " << cluster_id << " " << tmp_point << std::endl;
    //                                    cluster_id++;
    //                                }
    //                            }

    //                        }

    //                        if (!found_upper && col > 0)
    //                        {
    //                            // Check left neighbor
    //                            int32_t neighbor_label_left = labels.at<int32_t>(row, col-1);
    //                            if (neighbor_label_left >= 0)
    //                            {
    //                                auto neighbor_point_left = objectClouds.at(neighbor_label_left)->getOrganizedPoint(row, col-1);
    //                                if (neighbor_point_left)
    //                                {
    //                                    if ( radiusThreshold > euclideanDistance3D(neighbor_point_left, &tmp_point) )
    //                                    {
    //                                        // Existing label
    //                                        objectClouds.at(neighbor_label_left)->addOrganizedPoint(tmp_point, col, row);
    //                                        labels.at<int32_t>(row, col) = objectClouds.at(neighbor_label_left)->getID();
    //                                        std::cout << "DEBUG added EXISTING left point: " << objectClouds.at(neighbor_label_left)->getID() << " " << tmp_point << std::endl;
    //                                    }
    //                                    else
    //                                    {
    //                                        // New label
    //                                        objectClouds.push_back(new TrackedObject(cluster_id));
    //                                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
    //                                        labels.at<int32_t>(row, col) = cluster_id;
    //                                        std::cout << "DEBUG added NEW left point: " << cluster_id << " " << tmp_point << std::endl;
    //                                        cluster_id++;
    //                                    }
    //                                }

    //                            }
    //                        }
    //                    }

    //                }
    //                else
    //                    outOfRange++;
    //            }
    //        }
    //        std::cout << std::endl << "objectClouds " << objectClouds.size() << " out of range: " << outOfRange << std::endl << std::endl;
    //        //      std::cout << std::endl << "euclideanConnectedComponentsOrganized" << std::endl << labels << std::endl << std::endl;
    //        return objectClouds;
    //    }

    void euclideanConnectedComponentsUnorganized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<TrackedObject> *objects)
    {

    }

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
#if VERBOSE
            std::cerr << "(Pcl-Viewer) Idle" << std::endl;
#endif
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

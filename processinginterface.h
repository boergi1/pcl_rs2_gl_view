#ifndef PROCESSINGINTERFACE_H
#define PROCESSINGINTERFACE_H

#include <pcl/point_types.h>
//#include <pcl/point_cloud.h> // ?
#include <pcl/ModelCoefficients.h>

#include <pcl/common/time.h>
#include <pcl/common/common_headers.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

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
#include "filters.h"
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

    void clear()
    {
        _ID = 0;
        _objectCloud->clear();
        _pointIndices.clear();
    }

    void addOrganizedIndices( std::vector< std::pair<int,int> > otherPointIndices )
    {
        _pointIndices.insert(_pointIndices.end(), otherPointIndices.begin(), otherPointIndices.end());
    }

    void addCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr otherObjectCloud)
    {
        _objectCloud->insert(_objectCloud->end(), otherObjectCloud->begin(), otherObjectCloud->end());
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
    std::vector< std::pair<int,int> > getOrganizedIndices() { return _pointIndices; }
    int getID() { return _ID; }
    void setID(int ID) { _ID = ID; }
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

    void recursive_dfs(int x, int y, int label, pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, cv::Mat& Labels)
    {
        //   std::cout << "recursive_dfs X" << x << " Y" << y << " L " << label <<  std::endl;
        if (Labels.at<int32_t>(y,x) < LabelType_t::UNIDENTIFIED) return; // "<" or "!=" ???

        // r d l u
        const int dx_n4[] = {+1, 0, -1, 0};
        const int dy_n4[] = {0, +1, 0, -1};
        // r d l u rd ld lu ru
        //                const int dx[] = {+1, 0, -1, 0, +1, -1, -1, +1};
        //                const int dy[] = {0, +1, 0, -1, +1, +1, -1, -1};
        // r rd d ld l lu u ru
        const int dx_n8[] = {+1, +1, 0, -1, -1, -1, 0, +1};
        const int dy_n8[] = {0, +1, +1, +1, 0, -1, -1, -1};

        float distanceThresholdMax = 0.76f;//0.75f;
        float distanceThresholdMin = 0.70f;// 0.70f;
        float radiusThreshold = 0.010f;
        int w = Cloud->width;
        int h = Cloud->height;

        if ( (x == w) || (y == h) || x < 0 || (y < 0) ) return; // out of bounds

        auto center_point = Cloud->at(x,y);
        if ((center_point.z < distanceThresholdMin) || (center_point.z > distanceThresholdMax))
        {
            Labels.at<int32_t>(y,x) = LabelType_t::BACKGROUND;
            //     std::cout << "background X" << x << " Y " << y << std::endl;
            return;
        }

        //   Labels.at<int32_t>(y,x) = label;
        bool previousIsBackground = false;
        bool backgroundFound = false;
        bool validFound = false;

        for (int direction = 0; direction < 8; ++direction)
        {
            auto x2 = x + dx_n8[direction];
            auto y2 = y + dy_n8[direction];
            if ( (x2 == w) || (y2 == h) || x2 < 0 || (y2 < 0) ) continue; // out of bounds
            auto neighbor_point = Cloud->at(x2,y2);

            if ((neighbor_point.z < distanceThresholdMin) || (neighbor_point.z > distanceThresholdMax))
            {
                Labels.at<int32_t>(y2,x2) = LabelType_t::BACKGROUND;
                if (direction == 7)
                {
                 //   cout << "DEBUG Test RU:" << Labels.at<int32_t>(y2,x2) << " R:"<< Labels.at<int32_t>(y+dy_n8[0],x+dx_n8[0]) << endl;
                    if  (Labels.at<int32_t>(y+dy_n8[0],x+dx_n8[0] != LabelType_t::BACKGROUND))
                    {
                        std::cout << " recursive call with edge neighbor - X2:" << x+dx_n8[0] << " Y2:" << y+dy_n8[0] << " L:" << Labels.at<int32_t>(y,x) << std::endl;
                        recursive_dfs(x+dx_n8[0], y+dy_n8[0], Labels.at<int32_t>(y,x), Cloud, Labels);
                    }
                }
                backgroundFound = true;
                previousIsBackground = true;
                continue;
            }

            //       bool inRange = euclideanDistance3D(&neighbor_point, &center_point) < radiusThreshold;

            //            if (inRange)
            //            {
            //                std::cout << " in range X2:" << x2 << " Y2:" << y2 << std::endl;
            //                recursive_dfs(x2, y2, label, Cloud, Labels);
            //            }

            //            if (inRange) // ?????????????
            //                Labels.at<int32_t>(y2,x2) = label;
            //            else recursive_dfs(x2, y2, label, Cloud, Labels);





            if (euclideanDistance3D(&neighbor_point, &center_point) < radiusThreshold)
            {
                validFound = true;
                if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    cout << "Center: identified; Neighbor: unidentified; X2:" << x2 << " Y2:" << y2 << endl;

                    Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                    if (previousIsBackground)
                    {
                        previousIsBackground = false;
                        std::cout << " recursive call with unidentified neighbor - X2:" << x2 << " Y2:" << y2 << " L:" << Labels.at<int32_t>(y,x) << std::endl;
                        recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                    }


                    //   std::cout << " recursive call with unidentified neighbor - X2:" << x2 << " Y2:" << y2 << " L:" << Labels.at<int32_t>(y,x) << std::endl;
                    //   recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                    continue;
                }

                if (Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    cout << "Center: unidentified; Neighbor: unidentified; X2:" << x2 << " Y2:" << y2 << endl;

                    Labels.at<int32_t>(y,x) = label;
                    Labels.at<int32_t>(y2,x2) = label;
                    if (previousIsBackground)
                    {
                        previousIsBackground = false;
                        std::cout << " recursive call with unidentified neighbor and center - X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                        recursive_dfs(x2, y2, label, Cloud, Labels);
                    }

                    //                    std::cout << " recursive call with unidentified neighbor and center - X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                    //                    recursive_dfs(x2, y2, label, Cloud, Labels);
                    continue;
                }

                if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) > LabelType_t::UNIDENTIFIED)
                {
                    previousIsBackground = false;
                    if ((false))
                    {
                        if (Labels.at<int32_t>(y,x) == Labels.at<int32_t>(y2,x2))
                            continue;
                        cout << " recursive call Center: identified; Neighbor: identified; X2:" << x2 << " Y2:" << y2 << endl;
                        if (Labels.at<int32_t>(y,x) < Labels.at<int32_t>(y2,x2))
                            Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                        else
                            Labels.at<int32_t>(y,x) = Labels.at<int32_t>(y2,x2);
                        recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                    }

                }






                //     std::cout << " in range - X2:" << x2 << " Y2:" << y2 << std::endl;
                //   continue;
            }
            else
            {
                //                std::cout << " recursive call - X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                //                recursive_dfs(x2, y2, label, Cloud, Labels);
                //                if (Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                //                {
                //                    std::cout << " recursive call with out of range neighbor - X2:" << x2 << " Y2:" << y2 << std::endl;
                //                    recursive_dfs(x2, y2, label, Cloud, Labels);
                //                }
            }




        }


        if ((false))
        {
            if (backgroundFound && validFound)
            {
                cout << "Edge pixel detected" << endl;
                //            if (Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED)
                //                cerr << "Edge pixel invalid " << x << " " << y << endl;
                // Edge pixel detected
                for (int direction = 0; direction < 4; ++direction)
                {
                    auto x2 = x + dx_n4[direction];
                    auto y2 = y + dy_n4[direction];
                    if ( (x2 == w) || (y2 == h) || x2 < 0 || (y2 < 0) ) continue; // out of bounds
                    cout << "Edge loop 1; X2:" << x2 << " Y2:" << y2 << endl;

                    // Search for the valid neighbor
                    if (Labels.at<int32_t>(y2,x2) == Labels.at<int32_t>(y,x))
                    {
                        //       Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED;

                        for (int direction = 0; direction < 8; ++direction)
                        {
                            auto x3 = x2 + dx_n8[direction];
                            auto y3 = y2 + dy_n8[direction];
                            if ( (x3 == w) || (y3 == h) || x3 < 0 || (y3 < 0) ) continue; // out of bounds
                            cout << "Edge loop 2; X3:" << x3 << " Y3:" << y3 << endl;
                            // Search for a background neighbor
                            if (Labels.at<int32_t>(y3,x3) == LabelType_t::BACKGROUND)
                            {
                                // Edge detected
                                std::cout << " recursive call edge test - X2:" << x2 << " Y2:" << y2<<" X3:" << x3 << " Y3:" << y3 << std::endl;
                                //    recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                                break;
                            }
                        }

                        break;
                    }
                }

            }
        }


    }

    void euclideanDepthFirstSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        std::cout << "euclideanDepthFirstSearch" << std::endl;


        int w = cloud->width;
        int h = cloud->height;
        cv::Mat labelMat = cv::Mat(h,w,CV_32S,cv::Scalar(LabelType_t::UNIDENTIFIED));
        //  cv::Mat labelMat = cv::Mat::ones(h,w,CV_32S);

        int32_t current_label = LabelType_t::OBJECTS;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                std::cout << "dfs loop X:" << x << " Y:" << y << " L:" << labelMat.at<int32_t>(y,x) << std::endl;
                if (labelMat.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED)
                    recursive_dfs(x, y, current_label++, cloud, labelMat);
            }

        if ((true))
        {
            cv::Mat show;
            //     cout << "Labels:" << endl << labelMat << endl;
            show = cv::Mat(labelMat.rows, labelMat.cols, CV_8U);
            double scale = (1.0/(double)(w*h)) * (double)std::numeric_limits<uint8_t>::max();
            labelMat.convertTo(show, CV_8U, scale);
            cv::applyColorMap( show, show, cv::COLORMAP_JET);
            cv::imshow("euclideanDepthFirstSearch", show);
            cv::waitKey(0);
        }

    }




    void doUnion(int a, int b, int* component)
    {
        // cout << "doUnion a "<<a << " b "<<b<<endl;
        // get the root component of a and b, and set the one's parent to the other
        while (component[a] != a)
            a = component[a];
        while (component[b] != b)
            b = component[b];
        component[b] = a;
        //   cout << b << " is root of "<< a <<endl;
    }

    //    void unionCoords(int x, int y, int x2, int y2)
    //    {
    //     //   cout << "unionCoords x "<<x<<" y "<<y<<" x2 "<<x2<<" y2 "<<y2<<endl;
    //        if (y2 < h && x2 < w && input[x][y] && input[x2][y2])
    //            doUnion(x*h + y, x2*h + y2);
    //    }

    void euclideanUnionFind(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        std::cout << "euclideanUnionFind" << std::endl;
        float distanceThresholdMax = 0.76f;//0.75f;
        float distanceThresholdMin = 0.70f;// 0.70f;
        float radiusThreshold = 0.010f;

        int w = cloud->width;
        int h = cloud->height;
        int array_size = w*h;

        int component [array_size];
        for (int i = 0; i < array_size; i++)
            component[i] = i;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                int x2 = x+1;
                int y2 = y+1;

                auto current_idx = x*h + y;
                auto center_point = cloud->at(x,y);

                if (center_point.z < distanceThresholdMin || center_point.z > distanceThresholdMax)
                {
                    component[current_idx] = 0;
                    continue;
                }
                if (x2 < w)
                {
                    int neighbor_idx = x2*h + y;
                    pcl::PointXYZ neighbor_point = cloud->at(x2,y);

                    if (neighbor_point.z < distanceThresholdMin || neighbor_point.z > distanceThresholdMax)
                    {
                        component[neighbor_idx] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(&neighbor_point, &center_point) < radiusThreshold)
                        doUnion(current_idx, neighbor_idx, component);

                }
                if (y2 < h)
                {
                    int neighbor_idx = x*h + y2;
                    pcl::PointXYZ neighbor_point = cloud->at(x,y2);

                    if (neighbor_point.z < distanceThresholdMin || neighbor_point.z > distanceThresholdMax)
                    {
                        component[neighbor_idx] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(&neighbor_point, &center_point) < radiusThreshold)
                        doUnion(current_idx, neighbor_idx, component);
                }
            }

        cv::Mat labelMat = cv::Mat::zeros(h,w,CV_32S);
        // cv::Mat labelMat = cv::Mat_<int32_t>(h,w,-1);

        // fill labels
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                int c = x*h + y;
                if (c == component[c]) cout << "Parent root found: " << c << endl;
                while (component[c] != c) c = component[c];
                labelMat.at<int32_t>(y,x) = c;
            }
        }

        if ((false))
        {
            cv::Mat show;
            show = cv::Mat(labelMat.rows, labelMat.cols, CV_8U);
            double scale = (1.0 / (double)array_size) * (double)std::numeric_limits<uint8_t>::max();
            labelMat.convertTo(show, CV_8U, scale);
            cv::applyColorMap( show, show, cv::COLORMAP_HOT);
            cv::imshow("euclideanUnionFind", show);
            cv::waitKey(0);
        }
    }

    std::vector<TrackedObject*> euclideanConnectedComponentsOrganized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        auto start = std::chrono::high_resolution_clock::now();
        float distanceThresholdMax = 0.75f;
        float distanceThresholdMin = 0.70f;
        float radiusThreshold = 0.001f;
        bool debugPrint = false;

        //     std::vector<TrackedObject*> objectClouds;


        uint16_t cluster_id = LabelType_t::OBJECTS;

        //  cv::Mat labels = cv::Mat(height, width, CV_32SC1, cv::Scalar(-1)); // 32bit signed
        cv::Mat labels = cv::Mat::ones(cloud->height, cloud->width, CV_16UC1);

        int num_kernels = 3;

        //        cv::Mat kernel = cv::Mat::ones(3,3, CV_16UC1);
        //        cv::Point kernel_center = cv::Point(1,1);
        cv::Mat kernel = cv::Mat::ones(num_kernels, num_kernels, CV_16UC1);
        cv::Point kernel_center = cv::Point((num_kernels-1)/2, (num_kernels-1)/2);

        pcl::PointXYZ tmp_point;

        int outOfRange = 0;

        std::cout << "euclideanConnectedComponentsOrganized   kernel_center " << kernel_center<< " " << labels.rows << "x" << labels.cols <<  std::endl;



        for (int row = kernel_center.y; row < labels.rows - kernel_center.y; row++)
        {
            for (int col = kernel_center.x; col < labels.cols - kernel_center.x; col++)
            {
                uint16_t current_label = labels.at<uint16_t>(row, col);
                // Only use 4N for background?
                if ( current_label == LabelType_t::BACKGROUND )
                    continue;
                // Check for valid point
                tmp_point = cloud->at(col, row);
                if (tmp_point.z < distanceThresholdMin || tmp_point.z > distanceThresholdMax)
                {
                    outOfRange++;
                    labels.at<uint16_t>(row, col) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                    continue;
                }

                if (debugPrint)
                    std::cout << "DEBUG Center(x,y) = [" << col << "," << row << "] type: " << labels.at<uint16_t>(row, col)
                              << " 3D Point: (" << tmp_point.x << "," << tmp_point.y << "," << tmp_point.z << ")" << std::endl;

                // Only have a look at unidentified center pixels, != could be changed to < to also use center when already identified
                //                    if ( labels.at<uint16_t>(row, col) < LabelType_t::UNIDENTIFIED )
                //                        continue;
                // Using 3x3 kernel / N8
                for (int k_y = -kernel_center.y; k_y < kernel_center.y+1; k_y++)
                {
                    for (int k_x = -kernel_center.x; k_x < kernel_center.x+1; k_x++)
                    {
                        if ((k_x == 0) && (k_y == 0))
                            continue;

                        int n_x = col + k_x;
                        int n_y = row + k_y;
                        uint16_t neighbor_label = labels.at<uint16_t>(n_y, n_x);
                        if (neighbor_label == LabelType_t::BACKGROUND)
                            continue;

                        if (debugPrint)
                            std::cout << "DEBUG Neighbor(x,y) = [" << n_x << "," << n_y << "] type: " << labels.at<uint16_t>(n_y, n_x) << std::endl;

                        current_label = labels.at<uint16_t>(row, col);
                        // Neighbor is unidentified or an identified object
                        auto neighbor_point = cloud->at(n_x, n_y);
                        if (neighbor_point.z < distanceThresholdMin || neighbor_point.z > distanceThresholdMax)
                        {
                            if (debugPrint)
                                std::cout << "DEBUG Neighbor is background [" << n_x << "," << n_y << "]" << std::endl;
                            // Neighbor belongs to background, but isn't marked yet
                            outOfRange++;
                            labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                            continue;
                        }
                        // Neighbor has valid coordinates
                        bool inRange = euclideanDistance3D(&neighbor_point, &tmp_point) < radiusThreshold;
                        if ( !inRange )
                        {
                            if (debugPrint)
                                std::cout << "DEBUG Neighbor is not in radius [" << n_x << "," << n_y << "]" << std::endl;
                            continue;
                        }
                        // Neighbor point is in range of center point
                        if (neighbor_label == LabelType_t::UNIDENTIFIED)
                        {
                            // Unidentified neighbor
                            if (current_label == LabelType_t::UNIDENTIFIED)
                            {
                                if (debugPrint)
                                    std::cout << "DEBUG combine with Neighbor [" << n_x << "," << n_y << "]" << std::endl;
                                // Center pixel is also unidentified, create a new object
                                //                                        objectClouds.push_back(new TrackedObject(cluster_id));
                                //                                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
                                //                                        objectClouds.back()->addOrganizedPoint(neighbor_point, n_x, n_y);
                                labels.at<uint16_t>(row, col) = static_cast<uint16_t>( cluster_id );
                                labels.at<uint16_t>(n_y, n_x) =static_cast<uint16_t>(  cluster_id );
                                //     current_label = cluster_id;
                                cluster_id++;

                                //                                        std::cout << "DEBUG created NEW object with points: C [" << labels.at<uint16_t>(row, col)
                                //                                                  << "] " << row << " " << col << " " << tmp_point << " and N ["<< labels.at<uint16_t>(n_y, n_x)
                                //                                                  << "] " << n_y << " " << n_x << " " << neighbor_point << std::endl;
                            }
                            else
                            {
                                if (debugPrint)
                                    std::cout << "DEBUG add Neighbor to current [" << n_x << "," << n_y << "]" << std::endl;
                                // Center pixel belongs to object, add neighbor to current label
                                //                                        objectClouds.at(current_label - LabelType_t::OBJECTS)->addOrganizedPoint(neighbor_point, n_x, n_y);
                                labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( current_label );

                                //                                        std::cout << "DEBUG added NEW neighbor to existing center object with points: C [" << labels.at<uint16_t>(row, col)
                                //                                                  << "] " << row << " " << col << " " << tmp_point << " and N ["<< labels.at<uint16_t>(n_y, n_x)
                                //                                                  << "] " << n_y << " " << n_x << " " << neighbor_point << std::endl;
                            }
                        }
                        else
                        {
                            // Neighbor is identified and already belongs to an object
                            if (current_label == LabelType_t::UNIDENTIFIED)
                            {
                                if (debugPrint)
                                    std::cout << "DEBUG add Center to Neighbor [" << n_x << "," << n_y << "]" << std::endl;
                                // Add center point to existing neighbor object
                                //                                        objectClouds.at(neighbor_label - LabelType_t::OBJECTS)->addOrganizedPoint(tmp_point, col, row);
                                labels.at<uint16_t>(row, col) = static_cast<uint16_t>( neighbor_label );
                                //   current_label = neighbor_label;

                                //                                        std::cout << "DEBUG added NEW center to existing neighbor object with points: C [" << labels.at<uint16_t>(row, col)
                                //                                                  << "] " << row << " " << col << " " << tmp_point << " and N ["<< labels.at<uint16_t>(n_y, n_x)
                                //                                                  << "] " << n_y << " " << n_x << " " << neighbor_point << std::endl;
                            }
                            else
                            {
                                if (debugPrint)
                                    std::cout << "DEBUG Center and Neighbor already identified [" << n_x << "," << n_y << "]" << std::endl;
                                // Neighbor and center are both identified,
                                // which means both belong to the same objects
                                if (neighbor_label != current_label)
                                {
                                    if (current_label < neighbor_label)
                                    {
                                        labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( current_label );
                                    }
                                    else
                                    {
                                        labels.at<uint16_t>(row, col) = static_cast<uint16_t>( neighbor_label );
                                    }

                                }
                                //                                    else
                                //                                    {
                                //                                        labels.at<uint16_t>(row, col) = static_cast<uint16_t>( neighbor_label );
                                //                                    }


                                //                                    // Neighbor and center are both identified,
                                //                                    // which means both belong to the same OR different objects
                                //                                    if (neighbor_label != current_label)
                                //                                    {
                                //                                        // Check if both points belong to the same object
                                //                                        if (euclideanDistance3D(&neighbor_point, &tmp_point) < radiusThreshold)
                                //                                        {
                                //                                            // Both belong to the same object, combine them
                                //                                            //                                                auto neighbor_object = objectClouds.at(neighbor_label - LabelType_t::OBJECTS);
                                //                                            //                                                auto center_object = objectClouds.at(current_label - LabelType_t::OBJECTS);
                                //                                            if (current_label < neighbor_label)
                                //                                            {
                                //                                                //                                                    center_object->addOrganizedIndices(neighbor_object->getOrganizedIndices());
                                //                                                //                                                    center_object->addCloud(neighbor_object->getCloud());
                                //                                                //                                                    neighbor_object->clear();
                                //                                                labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( current_label );
                                //                                            }
                                //                                            else
                                //                                            {
                                //                                                //                                                    neighbor_object->addOrganizedIndices(center_object->getOrganizedIndices());
                                //                                                //                                                    neighbor_object->addCloud(center_object->getCloud());
                                //                                                //                                                    center_object->clear();
                                //                                                labels.at<uint16_t>(row, col) = static_cast<uint16_t>( neighbor_label );
                                //                                                //     current_label = neighbor_label;
                                //                                            }

                                //                                            //                                                std::cout << "DEBUG combined existing center with existing neighbor object with points: C [" << labels.at<uint16_t>(row, col)
                                //                                            //                                                          << "] " << row << " " << col << " " << tmp_point << " and N ["<< labels.at<uint16_t>(n_y, n_x)
                                //                                            //                                                          << "] " << n_y << " " << n_x << " " << neighbor_point << std::endl;

                                //                                        }
                                //                                    }
                            }
                        }

                    }
                }

                if (labels.at<uint16_t>(row, col) == LabelType_t::UNIDENTIFIED)
                {
                    if (debugPrint)
                        std::cout << "DEBUG Nothing found for center [" << col << "," << row << "]" << std::endl;
                    // No partner found, move to background
                    outOfRange++;
                    labels.at<uint16_t>(row, col) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                }




            }
        }



        int found_clusters = cluster_id - LabelType_t::OBJECTS;


        std::vector<TrackedObject*> objectClouds;
        cv::Mat cvMask, cvStats, cvCentroids;
        cv::Mat converted_labels;
        //     labels.copyTo(converted_labels)
        labels.convertTo(converted_labels, CV_8U);

        //                std::cout << "DEBUG unconverted labels" << std::endl << converted_labels << std::endl;

        //   std::cout << "DEBUG converted labels" << std::endl << converted_labels << std::endl;

        //   std::cout << "DEBUG stats" << std::endl;

        int cvObjectsCount = cv::connectedComponentsWithStats(converted_labels, cvMask, cvStats, cvCentroids, 4, CV_32S); // 10-15 ms


        std::cout << "DEBUG found cv objects: " << cvObjectsCount << " found clusters: " << found_clusters << std::endl;

        cv::Mat show_markers = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);

        for (int i = 0; i < cvObjectsCount; i++)
        {
            auto x = cvStats.at<int>(i, cv::CC_STAT_LEFT);
            auto y = cvStats.at<int>(i, cv::CC_STAT_TOP);
            auto w = cvStats.at<int>(i, cv::CC_STAT_WIDTH);
            auto h = cvStats.at<int>(i, cv::CC_STAT_HEIGHT);
            auto area = cvStats.at<int>(i, cv::CC_STAT_AREA);
            auto cx = cvCentroids.at<double>(i, 0);
            auto cy = cvCentroids.at<double>(i, 1);
            //     std::cout << "X " << x << " Y " << y << " W " << w << " H " << h << " A " << area << " CX " << cx << " CY " << cy << std::endl;

            cv::rectangle(show_markers, cv::Rect(x, y, w, h), cv::Scalar(255), 1);
            //            cv::circle(show_markers, cv::Point2d(cx, cy), 10, cv::Scalar(255), -1);
            //            cv::putText(show_markers, "id: "+std::to_string(i)+" area: " + std::to_string(w) + " x "+std::to_string(h), cv::Point(x, cy), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
        }


        //        // Get object positions
        //        cv::Mat labels, stats, centroids;
        //        int marker_count = cv::connectedComponentsWithStats(marker, labels, stats, centroids, 4, CV_32S); // 10-15 ms
        //        int skip = 2; // !!! First two objects are just background
        //        int objects_count = marker_count - skip;
        //        if (marker_count > skip)
        //        {
        //            tmp_tr_obj = new tracked_object_t[objects_count]();
        //            // Fill array with segmented objects
        //            for (int i = 0; i < objects_count; i++)
        //            {
        //                tmp_tr_obj[i].x = stats.at<int>(i+skip, cv::CC_STAT_LEFT);
        //                tmp_tr_obj[i].y = stats.at<int>(i+skip, cv::CC_STAT_TOP);
        //                tmp_tr_obj[i].w = stats.at<int>(i+skip, cv::CC_STAT_WIDTH);
        //                tmp_tr_obj[i].h = stats.at<int>(i+skip, cv::CC_STAT_HEIGHT);
        //                tmp_tr_obj[i].area = stats.at<int>(i+skip, cv::CC_STAT_AREA);
        //                tmp_tr_obj[i].cx = centroids.at<double>(i+skip, 0);
        //                tmp_tr_obj[i].cy = centroids.at<double>(i+skip, 1);
        //                tmp_tr_obj[i].unique_id = -1;
        //                tmp_tr_obj[i].lost_ctr = 0;
        //                tmp_tr_obj[i].seen_ctr = 0;
        //#if (IMSHOW_CV > 0 && VERBOSE > 0)
        //                // Draw markers on cv window
        //                cv::rectangle(marker, cv::Rect(tmp_tr_obj[i].x, tmp_tr_obj[i].y,
        //                                               tmp_tr_obj[i].w, tmp_tr_obj[i].h), cv::Scalar(0), 1);
        //                cv::circle(marker, cv::Point2d(tmp_tr_obj[i].cx, tmp_tr_obj[i].cy),
        //                           10, cv::Scalar(255), -1);
        //                cv::putText(marker, "id: "+std::to_string(i)+" area: "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].w))
        //                            + " x "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].h)),
        //                            cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].cy),
        //                            cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
        //#endif
        //            }
        //        }



        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
        std::cout << std::endl << "DEBUG objectClouds " << objectClouds.size() << " out of range: " << outOfRange << " took: " << duration << std::endl << std::endl;
        //     std::cout << std::endl << "DEBUG label matrix " << labels.rows << " " << labels.cols << std::endl << labels << std::endl << std::endl;

        cv::Mat show;
        labels.copyTo(show);
        show.convertTo(show, CV_8UC1);
        double min, max;
        cv::minMaxIdx(show, &min, &max);
        double scale = std::numeric_limits<uint8_t>::max() / max;
        cv::applyColorMap( show*scale, show, cv::COLORMAP_HOT);
        cv::imshow("components", show);
        cv::imshow("markers", show_markers);
        cv::waitKey(0);

        return objectClouds;





        //        for (int row = 0; row < labels.rows; row++)
        //        {
        //            for (int col = 0; col < labels.cols; col++)
        //            {
        //                tmp_point = cloud->at(col,row);
        //                // GLOBAL_REGION_Z_MAX_M
        //                // Check for valid point
        //                if ((tmp_point.z > 0) && (tmp_point.z < distanceThreshold))
        //                    //  if (tmp_point.z > 0)
        //                {
        //                    int upper_row = row - 1;
        //                    int left_col = col - 1;
        //                    bool new_object = false;
        //                    TrackedObject* neighborObject = nullptr;
        //                    //    pcl::PointXYZ* neighbor_point_ptr = nullptr;
        //                    pcl::PointXYZ neighbor_point;
        //                    if (upper_row < 0)
        //                    {
        //                        // First pixel
        //                        if (left_col < 0)
        //                        {
        //                            new_object = true;
        //                        }
        //                        else
        //                        {
        //                            uint16_t neighbor_label = labels.at<uint16_t>(row, left_col);
        //                            // First row - check left neighbor
        //                            if (neighbor_label > 0)
        //                            {
        //                                //   std::cout << "DEBUG found neighbor LEFT (first row) " << row << " " << left_col << std::endl;
        //                                neighborObject = objectClouds.at(neighbor_label-1);
        //                                //  neighbor_point_ptr = neighborObject->getOrganizedPoint(left_col, row);//(row, left_col);
        //                                neighbor_point = cloud->at(left_col, row);
        //                            }
        //                            else
        //                                new_object = true;
        //                        }
        //                    }
        //                    else if (left_col < 0)
        //                    {
        //                        // First column - check upper neighbor
        //                        uint16_t neighbor_label = labels.at<uint16_t>(upper_row, col);
        //                        if (neighbor_label > 0)
        //                        {
        //                            //    std::cout << "DEBUG found neighbor UPPER (first column) " << upper_row << " " << col << std::endl;
        //                            neighborObject = objectClouds.at(neighbor_label-1);
        //                            //    neighbor_point_ptr = neighborObject->getOrganizedPoint(col, upper_row);//(upper_row, col);
        //                            neighbor_point = cloud->at(col, upper_row);
        //                        }
        //                        else
        //                            new_object = true;
        //                    }
        //                    else
        //                    {
        //                        // Rest - check upper first, then left
        //                        uint16_t neighbor_label_up = labels.at<uint16_t>(upper_row, col);
        //                        if (neighbor_label_up > 0)
        //                        {
        //                            //   std::cout << "DEBUG found neighbor UPPER (body) " << upper_row << " " << col << std::endl;
        //                            neighborObject = objectClouds.at(neighbor_label_up-1);
        //                            // neighbor_point_ptr = neighborObject->getOrganizedPoint(col, upper_row);//(upper_row, col);
        //                            neighbor_point = cloud->at(col, upper_row);
        //                        }
        //                        else
        //                        {
        //                            uint16_t neighbor_label_left = labels.at<uint16_t>(row, left_col);
        //                            if (neighbor_label_left > 0)
        //                            {
        //                                //    std::cout << "DEBUG found neighbor LEFT (body) " << row << " " << left_col << std::endl;
        //                                neighborObject = objectClouds.at(neighbor_label_left-1);
        //                                //  neighbor_point_ptr = neighborObject->getOrganizedPoint(left_col, row);//(row, left_col);
        //                                neighbor_point = cloud->at(left_col, row);
        //                            }
        //                            else
        //                                new_object = true;
        //                        }
        //                    }




        //                    if (new_object)
        //                    {
        //                        // Add new object
        //                        objectClouds.push_back(new TrackedObject(cluster_id));
        //                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
        //                        labels.at<uint16_t>(row, col) = cluster_id;
        //                        //    std::cout << "DEBUG added NEW object with point: [" << labels.at<uint16_t>(row, col) << "] " << row << " " << col << " " << tmp_point << std::endl;
        //                        cluster_id++;
        //                    }
        //                    else
        //                    {
        //                        // Add point to existing object
        //                        if (neighbor_point.z)
        //                        {
        //                            if ( euclideanDistance3D(&neighbor_point, &tmp_point) < radiusThreshold )
        //                            {
        //                                neighborObject->addOrganizedPoint(tmp_point, col, row);
        //                                labels.at<uint16_t>(row, col) = neighborObject->getID();
        //                                //          std::cout << "DEBUG added EXISTING point to object: [" << labels.at<uint16_t>(row, col) << "] " << row << " " << col << " " << tmp_point << std::endl;
        //                            }
        //                        }
        //                        else std::cerr << "DEBUG got no neighbor point from row " << row << " col " << col << std::endl;
        //                    }
        //                }
        //                else
        //                    outOfRange++;
        //            }
        //        }





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

    void euclideanConnectedComponentsUnorganized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        std::cout << "euclideanConnectedComponentsUnorganized " << cloud->height << "x" << cloud->width << std::endl;

        //        auto start = std::chrono::high_resolution_clock::now();
        //        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();

        float distanceThresholdMax = 0.75f;
        float distanceThresholdMin = 0.70f;
        float radiusThreshold = 0.001f;
        bool debugPrint = false;

        //     std::vector<TrackedObject*> objectClouds;


        int cluster_id = LabelType_t::OBJECTS;

        pcl::console::TicToc tt;
        tt.tic();
        std::vector<int> labels = std::vector<int>(cloud->size(), LabelType_t::UNIDENTIFIED);

        for (int i = 0 ; i < cloud->size() ; i++ )
        {

            if (labels.at(i) != LabelType_t::UNIDENTIFIED) // only unidentified
                continue;
            //           std::cout << i+1 << " out of " << cloud->size() << std::endl;
            auto point_1 = cloud->at(i);
            if (point_1.z == 0.0)
            {
                labels.at(i) = LabelType_t::BACKGROUND;
                continue;
            }

            for (int j = 0 ; j < cloud->size() ; j++ )
            {
                if (labels.at(j) < LabelType_t::UNIDENTIFIED) // no background
                    continue;
                if (labels.at(i) == labels.at(j))
                    continue;

                auto point_2 = cloud->at(j);
                if (point_2.z == 0.0)
                {
                    labels.at(j) = LabelType_t::BACKGROUND;
                    continue;
                }
                if ( euclideanDistance3D(&point_1, &point_2)  < radiusThreshold ) // in range
                {

                    if (labels.at(i) > LabelType_t::UNIDENTIFIED)
                    {
                        // Center identified
                        if (labels.at(j) > LabelType_t::UNIDENTIFIED)
                        {
                            // Both identified, combine labels if needed
                            if (labels.at(i) < labels.at(j))
                                labels.at(j) = labels.at(i);
                            else
                                labels.at(i) = labels.at(j);
                        }
                        else
                        {
                            // Neighbor unidentified, add to current label
                            labels.at(j) = labels.at(i);
                        }
                    }
                    else
                    {
                        // Center unidentified
                        if (labels.at(j) > LabelType_t::UNIDENTIFIED)
                        {
                            // Neighbor identified, add to other label
                            labels.at(i) = labels.at(j);
                        }
                        else
                        {
                            // Neighbor unidentified, create a new cluster
                            labels.at(i) = cluster_id;
                            labels.at(j) = cluster_id;
                            cluster_id++;
                        }
                    }



                }
            }
        }


        //        cv::Mat centroidDistances(std::vector<tracked_object_t> *obj_centr, tracked_object_t *inp_centr, size_t size_in)
        //        {
        //            size_t size_obj = obj_centr->size();
        //            cv::Mat result = cv::Mat::zeros(static_cast<int>(size_obj), static_cast<int>(size_in), CV_64F);
        //            for (size_t i = 0; i < size_obj; i++)
        //            {
        //                double x_o = obj_centr->at(i).cx;
        //                double y_o = obj_centr->at(i).cy;
        //                for (size_t j = 0; j < size_in; ++j)
        //                {
        //                    double dx = inp_centr[j].cx - x_o;
        //                    double dy = inp_centr[j].cy - y_o;
        //                    result.at<double>(static_cast<int>(i), static_cast<int>(j)) = std::sqrt(dx * dx + dy * dy);
        //                }
        //            }
        //            return result;
        //        }



        //        pcl::PointXYZ temp_pt;

        //        // just z
        //        for (int a = 0 ; a < 5 ; a++ )
        //            for (int i = 0 ; i < cloud->size()-1 ; i++ )
        //            {
        //                auto &point_1 = cloud->at(i);
        //                auto &point_2 = cloud->at(i+1);
        //                if (point_1.z < point_2.z)
        //                {
        //                    temp_pt = point_1;
        //                    point_1 = point_2;
        //                    point_2 = temp_pt;
        //                }
        //                if (point_1.y < point_2.y)
        //                {
        //                    temp_pt = point_1;
        //                    point_1 = point_2;
        //                    point_2 = temp_pt;
        //                }
        //                if (point_1.x < point_2.x)
        //                {
        //                    temp_pt = point_1;
        //                    point_1 = point_2;
        //                    point_2 = temp_pt;
        //                }
        //            }








        //        float temp;
        //        for (int i = 0 ; i < cloud->size() ; i++ )
        //        {
        //            auto &point_1 = cloud->at(i);
        //            float temp;
        //            for (int j = 0 ; j < cloud->size() ; j++ )
        //            {
        //                if ( i==j )
        //                    continue;
        //                auto &point_2 = cloud->at(j);

        //                if (point_1.z < point_2.z)
        //                {
        //                    temp = point_1.z;
        //                    point_1.z = point_2.z;
        //                    point_2.z = temp;
        //                //    std::cout << "DEBUG changed points Z" << point_2 << " with " << point_1 << std::endl;
        //                    continue;
        //                }
        //                if (point_1.y < point_2.y)
        //                {
        //                    temp = point_1.y;
        //                    point_1.y = point_2.y;
        //                    point_2.y = temp;
        //              //      std::cout << "DEBUG changed points Y" << point_2 << " with " << point_1 << std::endl;
        //                    continue;
        //                }
        //                if (point_1.x < point_2.x)
        //                {
        //                    temp = point_1.x;
        //                    point_1.x = point_2.x;
        //                    point_2.x = temp;
        //                //    std::cout << "DEBUG changed points X" << point_2 << " with " << point_1 << std::endl;
        //                    continue;

        //                }
        //            }

        //            //            for (int i = 0 ; i < 2 ; i++ )
        //            //                for (int j = 0 ; j < 2; j++ )
        //            //                    for (int k = 0 ; k < 5 ; k++ )
        //            //                        for (int a = 0 ; a < 2 ; a++ )
        //            //                            for (int b = 0 ; b < 2 ; b++ )
        //            //                                for (int c = 0 ; c < 5 ; c++ )
        //            //                                    if (array[i][j][k] < array[a][b][c]){
        //            //                                        temp = array[i][j][k];
        //            //                                        array[i][j][k] = array[a][b][c];
        //            //                                        array[a][b][c] = temp;}
        //            //            for (int i = 0 ; i < 2 ; i++ )
        //            //                for (int j = 0 ; j < 2 ; j++ )
        //            //                    for (int k = 0 ; k < 5 ; k++ )
        //            //                    {
        //            //                        cout << array[i][j][k] << endl;
        //            //                    }
        //        }

        std::cout << "DEBUG took " << tt.toc() << std::endl;
        tt.tic();



        for (int i = 0 ; i < labels.size() ; i++ )
            std::cout << "Label #" << i << " " << labels.at(i) << std::endl;






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

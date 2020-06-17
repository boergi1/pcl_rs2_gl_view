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
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "format.h"
#include "filters.h"
#include "customtypes.h"
#include "mainwindowgl.h"

#include <thread>
#include <mutex>
#include <chrono>



/* TODO: Add class for each device with its own processing thread*/

class CloudQueue
{
public:
    CloudQueue(CameraType_t CameraType, std::string name = "");
    void addCloudT(std::tuple <unsigned long long, std::vector< float >*, long long> cloud_tuple);
    // <std::tuple<unsigned long long, pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>>
    std::tuple <unsigned long long, std::vector< float >*, long long> getCloudT();
    const std::tuple <unsigned long long, std::vector< float >*, long long> readCloudT();
    bool isEmpty();
    CameraType_t getCameraType();
private:
    std::deque<std::tuple <unsigned long long, std::vector< float >*, long long>>  m_cqueue;
    std::mutex m_mtx;
    CameraType_t m_camtype;
    std::string m_name;
};
#ifdef pcl_pointcloud
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
#endif

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
        if (m_mqueue.size() > QUE_SIZE_CLOUDS)
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
private:
    float* _objectCloud;
    // std::vector< std::pair<int,int> > _pointIndices;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr _objectCloud (new pcl::PointCloud<pcl::PointXYZ>);
    int _ID;
    unsigned short _x, _y, _w, _h;

    float* _dataXYZ;
    size_t _dataSize;
public:
    TrackedObject(int x, int y, int w, int h, int ID, float* dataXYZ)
    {
        _ID = ID;
        _x = x;
        _y = y;
        _w = w;
        _h = h;
        _dataXYZ = dataXYZ;
        _dataSize = _w*_h;
        //  pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud (new pcl::PointCloud<pcl::PointXYZ>());
        //  _objectCloud = objectCloud;
    }

    int getID() { return _ID; }
    void setID(int ID) { _ID = ID; }
    /* todo: add cloud and group to existing cloud,
     * calculate center of unorganized cloud
        OR do this with organized cloud */

};

class SegmentationProcessor // -> class Tracker -> class CloudCombiner (evtl temporal filter, find the transformation matrix)
{

private:
    std::thread _thr;
    std::mutex _mtxIn, _mtxOut;
    CameraType_t _camType;
    std::deque<std::vector<float>*> _input;
    std::deque<std::vector<TrackedObject>*> _output;
    bool _active = false;

    std::vector<float>* _currentPointCloud;

    void processSegmentation()
    {
        while (_active)
        {
            bool idle = true;
            while(_input.size())
            {
#if (VERBOSE > 1)
                auto seg_start = std::chrono::high_resolution_clock::now();
#endif
                idle = false;
                _mtxIn.lock();
                _currentPointCloud = _input.front();
                _input.pop_front();
                _mtxIn.unlock();

                if((false))
                {
                    std::vector<Eigen::Vector3d> test_vec;
                    test_vec.resize(_currentPointCloud->size()/SIZE_XYZ);
                    int test_idx = 0;
                    for (int i=0;i<_currentPointCloud->size(); i++)
                    {
                        switch(i % SIZE_XYZ)
                        {
                        case 0: test_vec.at(test_idx).x() = _currentPointCloud->at(i); break;
                        case 1: test_vec.at(test_idx).y() = _currentPointCloud->at(i); break;
                        case 2: test_vec.at(test_idx++).z() = _currentPointCloud->at(i); break;
                        default: break;
                        }
                    }
                    euclideanUnionFind(&test_vec);
                }
                else
                    euclideanUnionFind();


                delete _currentPointCloud;


#if (VERBOSE > 1)
                auto seg_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-seg_start).count();
                std::cout << "(SegmentationProcessor) processSegmentation took " << seg_end << " ms " << cameraTypeToString(_camType) << std::endl;
#endif


            }
            if (idle) std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_PROC_SEGM_NS));
        }
    }


    // Segmentation variables
    const int _dx_n8[8] = {+1, +1, 0, -1, -1, -1, 0, +1};
    const int _dy_n8[8] = {0, +1, +1, +1, 0, -1, -1, -1};
    float _distanceThresholdMax = GLOBAL_REGION_Z_MAX_M;
    float _distanceThresholdMin = GLOBAL_REGION_Z_MIN_M;
    float _radiusThreshold = 0.100f;

    float euclideanDistance3D(Eigen::Vector3d* point_1, Eigen::Vector3d* point_2)
    {
        float dx = point_2->x() - point_1->x();
        float dy = point_2->y() - point_1->y();
        float dz = point_2->z() - point_1->z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    float euclideanDistance3D(float* point_1, float* point_2)
    {
        float dx = point_2[0] - point_1[0];
        float dy = point_2[1] - point_1[1];
        float dz = point_2[2] - point_1[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void recursive_dfs(int x, int y, int label, std::vector<Eigen::Vector3d>* Cloud, cv::Mat& Labels)
    {
        if (Labels.at<int32_t>(y,x) < LabelType_t::UNIDENTIFIED) return;

        static int w = FRAME_WIDTH_DEPTH;
        static int h = FRAME_HEIGHT_DEPTH;

        if ( (x == w) || (y == h) || x < 0 || (y < 0) ) return; // out of bounds

        //   std::cout << "Accessing center X:" << x << " Y:" << y << std::endl;
        auto center_point = Cloud->at(y*w+x);
        if ((Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED) && ((center_point.z() < _distanceThresholdMin) || (center_point.z() > _distanceThresholdMax)))
        {
            Labels.at<int32_t>(y,x) = LabelType_t::BACKGROUND;
            //     std::cout << "background X" << x << " Y " << y << std::endl;
            return;
        }

        //   Labels.at<int32_t>(y,x) = label;

        for (int direction = 0; direction < 8; ++direction)
        {

            int x2 = x + _dx_n8[direction];
            int y2 = y + _dy_n8[direction];
            if ( (x2 == w) || (y2 == h) || x2 < 0 || (y2 < 0) ) continue; // out of bounds
            auto neighbor_point = Cloud->at(y2*w+x2);

            if (/*(Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED) &&*/ ((neighbor_point.z() < _distanceThresholdMin) || (neighbor_point.z() > _distanceThresholdMax)))
            {

                Labels.at<int32_t>(y2,x2) = LabelType_t::BACKGROUND;
                continue;
            }
            if (euclideanDistance3D(&neighbor_point, &center_point) < _radiusThreshold)
            {
                if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                }
                else if (Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    Labels.at<int32_t>(y,x) = label;
                    Labels.at<int32_t>(y2,x2) = label;
                }
                else if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) > LabelType_t::UNIDENTIFIED)
                {
                    if (Labels.at<int32_t>(y,x) == Labels.at<int32_t>(y2,x2))
                        continue;
                    //   cout << " recursive call Center: identified; Neighbor: identified; X2:" << x2 << " Y2:" << y2 << endl;
                    if (Labels.at<int32_t>(y,x) < Labels.at<int32_t>(y2,x2))
                        Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                    else
                        Labels.at<int32_t>(y,x) = Labels.at<int32_t>(y2,x2);
                    recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                }
            }
        }
    }

    void recursive_dfs_edge(int x, int y, int label, std::vector<Eigen::Vector3d>* Cloud, cv::Mat& Labels)
    {
        //   std::cout << "recursive_dfs X" << x << " Y" << y << " L " << label <<  std::endl;
        if (Labels.at<int32_t>(y,x) < LabelType_t::UNIDENTIFIED) return;

        static int w = FRAME_WIDTH_DEPTH;
        static int h = FRAME_HEIGHT_DEPTH;

        if ( (x == w) || (y == h) || x < 0 || (y < 0) ) return; // out of bounds

        //  std::cout << "Accessing center X:" << x << " Y:" << y << std::endl;
        auto center_point = Cloud->at(y*w+x);
        if ((Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED) && ((center_point.z() < _distanceThresholdMin) || (center_point.z() > _distanceThresholdMax)))
        {
            Labels.at<int32_t>(y,x) = LabelType_t::BACKGROUND;
            //     std::cout << "background X" << x << " Y " << y << std::endl;
            return;
        }

        //   Labels.at<int32_t>(y,x) = label;
        static bool previousIsInvalid = false;
        //  static int current_label = Labels.at<int32_t>(y,x);


        for (int direction = 0; direction < 8; ++direction)
        {

            int x2 = x + _dx_n8[direction];
            int y2 = y + _dy_n8[direction];
            if ( (x2 == w) || (y2 == h) || x2 < 0 || (y2 < 0) ) continue; // out of bounds

            //   cout << "Neighborhood #" << direction << " X:" << x << " Y:" << y << " X2:" << x2 << " Y2:" << y2 << endl;

            auto neighbor_point = Cloud->at(y2*w+x2);

            if (/*(Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED) &&*/ ((neighbor_point.z() < _distanceThresholdMin) || (neighbor_point.z() > _distanceThresholdMax)))
            {

                Labels.at<int32_t>(y2,x2) = LabelType_t::BACKGROUND;
                if (direction == 7)
                {
                    if  (Labels.at<int32_t>(y+_dy_n8[0],x+_dx_n8[0]) != LabelType_t::BACKGROUND)
                    {
                        //     std::cout << " recursive call with edge neighbor - X:" << x << " Y:" << y << " X2:" << x+_dx_n8[0] << " Y2:" << y+_dy_n8[0] << " L:" << Labels.at<int32_t>(y,x) << std::endl;
                        recursive_dfs_edge(x+_dx_n8[0], y+_dy_n8[0], Labels.at<int32_t>(y,x), Cloud, Labels);
                    }
                }

                previousIsInvalid = true;
                continue;
            }
            if (euclideanDistance3D(&neighbor_point, &center_point) < _radiusThreshold)
            {
                //     std::cout << "DEBUG in range, previousInvalid: "<< previousIsInvalid << endl;

                if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    //        cout << "Center: identified; Neighbor: unidentified; X2:" << x2 << " Y2:" << y2 << endl;

                    Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                    if (previousIsInvalid)
                    {
                        previousIsInvalid = false;
                        //     std::cout << " recursive call with unidentified neighbor - X:" << x << " Y:" << y << " X2:" << x2 << " Y2:" << y2 << " L:" << Labels.at<int32_t>(y,x) << std::endl;
                        recursive_dfs_edge(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);
                    }
                }

                else if (Labels.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) == LabelType_t::UNIDENTIFIED)
                {
                    //      cout << "Center: unidentified; Neighbor: unidentified; X2:" << x2 << " Y2:" << y2 << endl;

                    Labels.at<int32_t>(y,x) = label;
                    Labels.at<int32_t>(y2,x2) = label;
                    if (previousIsInvalid)
                    {
                        previousIsInvalid = false;
                        //   std::cout << " recursive call with unidentified neighbor and center - X:" << x << " Y:" << y << " X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                        recursive_dfs_edge(x2, y2, label, Cloud, Labels);
                    }
                }

                else if (Labels.at<int32_t>(y,x) > LabelType_t::UNIDENTIFIED && Labels.at<int32_t>(y2,x2) > LabelType_t::UNIDENTIFIED)
                {

                    //   cout << "Center: identified; Neighbor: identified; X2:" << x2 << " Y2:" << y2 << endl;
                    if (previousIsInvalid)
                    {
                        previousIsInvalid = false;
                        if (Labels.at<int32_t>(y,x) == Labels.at<int32_t>(y2,x2))
                            continue;
                        //   std::cout << " recursive call with identified neighbor and center - X:" << x << " Y:" << y << " X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                        recursive_dfs_edge(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);


                        //                        if (Labels.at<int32_t>(y,x) == Labels.at<int32_t>(y2,x2))
                        //                            continue;

                        //                        std::cout << " recursive call with identified neighbor and center - X2:" << x2 << " Y2:" << y2 << " L:" << label << std::endl;
                        //                        if (Labels.at<int32_t>(y,x) < Labels.at<int32_t>(y2,x2))
                        //                            Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                        //                        else
                        //                            Labels.at<int32_t>(y,x) = Labels.at<int32_t>(y2,x2);
                        //                        recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud, Labels);




                    }

                    //                    if ((false))
                    //                    {
                    //                        if (Labels.at<int32_t>(y,x) == Labels.at<int32_t>(y2,x2))
                    //                            continue;
                    //                        cout << " recursive call Center: identified; Neighbor: identified; X2:" << x2 << " Y2:" << y2 << endl;
                    //                        if (Labels.at<int32_t>(y,x) < Labels.at<int32_t>(y2,x2))
                    //                            Labels.at<int32_t>(y2,x2) = Labels.at<int32_t>(y,x);
                    //                        else
                    //                            Labels.at<int32_t>(y,x) = Labels.at<int32_t>(y2,x2);
                    //                        recursive_dfs(x2, y2, Labels.at<int32_t>(y,x), Cloud->makeShared(), Labels);
                    //                    }

                }
            }
            else
            {
                //    std::cout << "DEBUG out of range" << endl;
                previousIsInvalid = true;
                //   continue;
            }
        }
    }

    void euclideanDepthFirstSearch(std::vector<Eigen::Vector3d>* cloud)
    {
        std::cout << "euclideanDepthFirstSearch" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        int w = FRAME_WIDTH_DEPTH;
        int h = FRAME_HEIGHT_DEPTH;
        cv::Mat labelMat = cv::Mat(h,w,CV_32S,cv::Scalar(LabelType_t::UNIDENTIFIED));
        //  cv::Mat labelMat = cv::Mat::ones(h,w,CV_32S);

        int32_t current_label = LabelType_t::OBJECTS;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                //      std::cout << "dfs loop X:" << x << " Y:" << y << " L:" << labelMat.at<int32_t>(y,x) << std::endl;
                if (labelMat.at<int32_t>(y,x) == LabelType_t::UNIDENTIFIED)
                    recursive_dfs(x, y, current_label++, cloud, labelMat);
                //  recursive_dfs_edge(x, y, current_label++, cloud, labelMat);
            }

        auto end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
        std::cout << "DEBUG took " << end << " ms" << std::endl;

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



    void euclideanUnionFind()
    {


        int w = FRAME_WIDTH_DEPTH;
        int h = FRAME_HEIGHT_DEPTH;

        int component_size = _currentPointCloud->size()/SIZE_XYZ;

        int component [component_size];
        for (int i = 0; i < component_size; i++)
            component[i] = i;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                int x2 = x+1;
                int y2 = y+1;

                auto current_component = x*h + y;


                int pc_idx_center = y*w + x;
                //  auto center_point = Cloud->at(y*w+x);
                float currentPoint [SIZE_XYZ] = {
                    _currentPointCloud->at(pc_idx_center),     // X
                    _currentPointCloud->at(pc_idx_center+1),   // Y
                    _currentPointCloud->at(pc_idx_center+2) }; // Z

                if (currentPoint[2] < _distanceThresholdMin || currentPoint[2] > _distanceThresholdMax)
                {
                    component[current_component] = 0;
                    continue;
                }
                if (x2 < w)
                {
                    int neighbor_component = x2*h + y;
                    int pc_idx_neighb = y*w + x2;
                    //  auto neighbor_point = Cloud->at(y*w+x2);
                    float neighborPoint [SIZE_XYZ] = {
                        _currentPointCloud->at(pc_idx_neighb),     // X
                        _currentPointCloud->at(pc_idx_neighb+1),   // Y
                        _currentPointCloud->at(pc_idx_neighb+2) }; // Z


                    if (neighborPoint[2] < _distanceThresholdMin || neighborPoint[2] > _distanceThresholdMax)
                    {
                        component[neighbor_component] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(neighborPoint, currentPoint) < _radiusThreshold)
                        doUnion(current_component, neighbor_component, component);

                }
                if (y2 < h)
                {
                    int neighbor_component = x*h + y2;
                    int pc_idx_neighb = y2*w + x;
                    //  auto neighbor_point = Cloud->at(y2*w+x);
                    float neighborPoint [SIZE_XYZ] = {
                        _currentPointCloud->at(pc_idx_neighb),     // X
                        _currentPointCloud->at(pc_idx_neighb+1),   // Y
                        _currentPointCloud->at(pc_idx_neighb+2) }; // Z

                    if (neighborPoint[2] < _distanceThresholdMin || neighborPoint[2] > _distanceThresholdMax)
                    {
                        component[neighbor_component] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(neighborPoint, currentPoint) < _radiusThreshold)
                        doUnion(current_component, neighbor_component, component);
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
                //   if (c == component[c]) cout << "Parent root found: " << c << endl;
                while (component[c] != c) c = component[c];
                labelMat.at<int32_t>(y,x) = c;
            }
        }


        if ((true) && _camType == CameraType_t::CENTRAL)
        {
            cv::Mat show;
            show = cv::Mat(labelMat.rows, labelMat.cols, CV_8U);
            double scale = (1.0 / (double)component_size) * (double)std::numeric_limits<uint8_t>::max();
            labelMat.convertTo(show, CV_8U, scale);
            cv::applyColorMap( show, show, cv::COLORMAP_HOT);
            cv::imshow("euclideanUnionFind", show);
            cv::waitKey(1);
        }
    }







    void euclideanUnionFind(std::vector<Eigen::Vector3d>* Cloud)
    {


        int w = FRAME_WIDTH_DEPTH;
        int h = FRAME_HEIGHT_DEPTH;
        int array_size = Cloud->size();

        int component [array_size];
        for (int i = 0; i < array_size; i++)
            component[i] = i;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                int x2 = x+1;
                int y2 = y+1;

                auto current_idx = x*h + y;
                auto center_point = Cloud->at(y*w+x);

                if (center_point.z() < _distanceThresholdMin || center_point.z() > _distanceThresholdMax)
                {
                    component[current_idx] = 0;
                    continue;
                }
                if (x2 < w)
                {
                    int neighbor_idx = x2*h + y;
                    auto neighbor_point = Cloud->at(y*w+x2);

                    if (neighbor_point.z() < _distanceThresholdMin || neighbor_point.z() > _distanceThresholdMax)
                    {
                        component[neighbor_idx] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(&neighbor_point, &center_point) < _radiusThreshold)
                        doUnion(current_idx, neighbor_idx, component);

                }
                if (y2 < h)
                {
                    int neighbor_idx = x*h + y2;
                    auto neighbor_point = Cloud->at(y2*w+x);

                    if (neighbor_point.z() < _distanceThresholdMin || neighbor_point.z() > _distanceThresholdMax)
                    {
                        component[neighbor_idx] = 0;
                        continue;
                    }
                    if (euclideanDistance3D(&neighbor_point, &center_point) < _radiusThreshold)
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
                //   if (c == component[c]) cout << "Parent root found: " << c << endl;
                while (component[c] != c) c = component[c];
                labelMat.at<int32_t>(y,x) = c;
            }
        }


        if ((true) && _camType == CameraType_t::CENTRAL)
        {
            cv::Mat show;
            show = cv::Mat(labelMat.rows, labelMat.cols, CV_8U);
            double scale = (1.0 / (double)array_size) * (double)std::numeric_limits<uint8_t>::max();
            labelMat.convertTo(show, CV_8U, scale);
            cv::applyColorMap( show, show, cv::COLORMAP_HOT);
            cv::imshow("euclideanUnionFind", show);
            cv::waitKey(1);
        }
    }

    void euclideanConnectedComponentsOrganized(std::vector<Eigen::Vector3d>* Cloud)
    {
        auto start = std::chrono::high_resolution_clock::now();

        int w = FRAME_WIDTH_DEPTH;
        int h = FRAME_HEIGHT_DEPTH;

        //     std::vector<TrackedObject*> objectClouds;

        uint16_t cluster_id = LabelType_t::OBJECTS;
        cv::Mat labels = cv::Mat::ones(h, w, CV_16UC1);

        int num_kernels = 3;
        cv::Mat kernel = cv::Mat::ones(num_kernels, num_kernels, CV_16UC1);
        cv::Point kernel_center = cv::Point((num_kernels-1)/2, (num_kernels-1)/2);

        //     pcl::PointXYZ tmp_point;

        int outOfRange = 0;

        std::cout << "euclideanConnectedComponentsOrganized   kernel_center " << kernel_center<< " " << labels.rows << "x" << labels.cols <<  std::endl;



        for (int y = kernel_center.y; y < labels.rows - kernel_center.y; y++)
        {
            for (int x = kernel_center.x; x < labels.cols - kernel_center.x; x++)
            {
                uint16_t current_label = labels.at<uint16_t>(y, x);
                // Only use 4N for background?
                if ( current_label == LabelType_t::BACKGROUND )
                    continue;
                // Check for valid point
                auto center_point = Cloud->at(y*w+x);
                //   tmp_point = cloud->at(col, row);
                if (center_point.z() < _distanceThresholdMin || center_point.z() > _distanceThresholdMax)
                {
                    outOfRange++;
                    labels.at<uint16_t>(y, x) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                    continue;
                }

                std::cout << "DEBUG Center(x,y) = [" << x << "," << y << "] type: " << labels.at<uint16_t>(y, x)
                          << " 3D Point: (" << center_point.x() << "," << center_point.y() << "," << center_point.z() << ")" << std::endl;

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

                        int n_x = x + k_x;
                        int n_y = y + k_y;
                        uint16_t neighbor_label = labels.at<uint16_t>(n_y, n_x);
                        if (neighbor_label == LabelType_t::BACKGROUND)
                            continue;

                        //  std::cout << "DEBUG Neighbor(x,y) = [" << n_x << "," << n_y << "] type: " << labels.at<uint16_t>(n_y, n_x) << std::endl;

                        current_label = labels.at<uint16_t>(y, x);
                        // Neighbor is unidentified or an identified object
                        auto neighbor_point = Cloud->at(n_y*w+n_x);
                        if (neighbor_point.z() < _distanceThresholdMin || neighbor_point.z() > _distanceThresholdMax)
                        {
                            //   std::cout << "DEBUG Neighbor is background [" << n_x << "," << n_y << "]" << std::endl;

                            // Neighbor belongs to background, but isn't marked yet
                            outOfRange++;
                            labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                            continue;
                        }
                        // Neighbor has valid coordinates
                        bool inRange = euclideanDistance3D(&neighbor_point, &center_point) < _radiusThreshold;
                        if ( !inRange )
                        {
                            // std::cout << "DEBUG Neighbor is not in radius [" << n_x << "," << n_y << "]" << std::endl;
                            continue;
                        }
                        // Neighbor point is in range of center point
                        if (neighbor_label == LabelType_t::UNIDENTIFIED)
                        {
                            // Unidentified neighbor
                            if (current_label == LabelType_t::UNIDENTIFIED)
                            {

                                //   std::cout << "DEBUG combine with Neighbor [" << n_x << "," << n_y << "]" << std::endl;

                                // Center pixel is also unidentified, create a new object
                                //                                        objectClouds.push_back(new TrackedObject(cluster_id));
                                //                                        objectClouds.back()->addOrganizedPoint(tmp_point, col, row);
                                //                                        objectClouds.back()->addOrganizedPoint(neighbor_point, n_x, n_y);
                                labels.at<uint16_t>(y, x) = static_cast<uint16_t>( cluster_id );
                                labels.at<uint16_t>(n_y, n_x) =static_cast<uint16_t>(  cluster_id );
                                //     current_label = cluster_id;
                                cluster_id++;
                            }
                            else
                            {
                                //  std::cout << "DEBUG add Neighbor to current [" << n_x << "," << n_y << "]" << std::endl;

                                // Center pixel belongs to object, add neighbor to current label
                                //                                        objectClouds.at(current_label - LabelType_t::OBJECTS)->addOrganizedPoint(neighbor_point, n_x, n_y);
                                labels.at<uint16_t>(n_y, n_x) = static_cast<uint16_t>( current_label );
                            }
                        }
                        else
                        {
                            // Neighbor is identified and already belongs to an object
                            if (current_label == LabelType_t::UNIDENTIFIED)
                            {
                                // std::cout << "DEBUG add Center to Neighbor [" << n_x << "," << n_y << "]" << std::endl;

                                // Add center point to existing neighbor object
                                //                                        objectClouds.at(neighbor_label - LabelType_t::OBJECTS)->addOrganizedPoint(tmp_point, col, row);
                                labels.at<uint16_t>(y, x) = static_cast<uint16_t>( neighbor_label );
                                //   current_label = neighbor_label;
                            }
                            else
                            {
                                //  std::cout << "DEBUG Center and Neighbor already identified [" << n_x << "," << n_y << "]" << std::endl;

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
                                        labels.at<uint16_t>(y, x) = static_cast<uint16_t>( neighbor_label );
                                    }

                                }
                            }
                        }

                    }
                }

                if (labels.at<uint16_t>(y, x) == LabelType_t::UNIDENTIFIED)
                {
                    // std::cout << "DEBUG Nothing found for center [" << x << "," << y << "]" << std::endl;

                    // No partner found, move to background
                    outOfRange++;
                    labels.at<uint16_t>(y, x) = static_cast<uint16_t>( LabelType_t::BACKGROUND );
                }
            }
        }
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
        std::cout << "DEBUG took " << duration << std::endl;
        int found_clusters = cluster_id - LabelType_t::OBJECTS;
        //   std::vector<TrackedObject*> objectClouds;
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


        //  std::cout << std::endl << "DEBUG objectClouds " << objectClouds.size() << " out of range: " << outOfRange << " took: " << duration << std::endl << std::endl;
        //     std::cout << std::endl << "DEBUG label matrix " << labels.rows << " " << labels.cols << std::endl << labels << std::endl << std::endl;

        cv::Mat show;
        labels.copyTo(show);
        show.convertTo(show, CV_8UC1);
        double min, max;
        cv::minMaxIdx(show, &min, &max);
        double scale = std::numeric_limits<uint8_t>::max() / max;
        cv::applyColorMap( show*scale, show, cv::COLORMAP_JET);
        cv::imshow("components", show);
        cv::imshow("markers", show_markers);
        cv::waitKey(0);

    }

public:
    SegmentationProcessor(CameraType_t camType)
    {
        _camType = camType;
    }
    void setActive(bool running)
    {
        if (running)
        {
            if ( _thr.joinable() )
            {
                std::cerr << "(SegmentationProcessor) Thread already running: " << _thr.get_id() << std::endl;
                return;
            }
            _active = true;
            _thr = std::thread(&SegmentationProcessor::processSegmentation, this);
        }
        else
        {
            _active = false;
            if ( _thr.joinable() )
                _thr.join();
            else std::cerr << "(SegmentationProcessor) Thread not joinable: " << _thr.get_id() << std::endl;
        }
        return;
    }

    void addInput(std::vector<float>* input)
    {
        _mtxIn.lock();
        _input.push_back(input);
        _mtxIn.unlock();
        if (_input.size() > QUE_SIZE_SEG)
        {
#if VERBOSE
            std::cerr << "(SegmentationProcessor) Too many points in input queue " << _camType << std::endl;
#endif
            _mtxIn.lock();
            auto tmpptr = _input.front();
            _input.pop_front();
            delete tmpptr;
            _mtxIn.unlock();
        }
    }
    void addOutput(std::vector<TrackedObject>* output)
    {
        _mtxOut.lock();
        _output.push_back(output);
        _mtxOut.unlock();
        if (_output.size() > QUE_SIZE_SEG)
        {
#if VERBOSE
            std::cerr << "(SegmentationProcessor) Too many objects in output queue " << _camType << std::endl;
#endif
            _mtxOut.lock();
            auto tmpptr = _output.front();
            _output.pop_front();
            delete tmpptr;
            _mtxOut.unlock();
        }
    }
};

class ProcessingInterface // todo: add one thread for each camera
{
private:
    MainWindowGL* _WindowRef;

    bool m_active;
    //    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clouds_buffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_proc_cloud = nullptr; // todo mutex this
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr
            (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector< CloudQueue* > m_input_clouds;
    std::vector< MatQueue* > m_input_depth;
    std::vector< MatQueue* > m_input_color;

    std::vector< SegmentationProcessor* > _segmentThreads;

    std::thread _processingThread;

#if (PCL_VIEWER == 1)
    pcl::visualization::CloudViewer m_pcl_viewer;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > m_viewer_clouds;
    std::mutex m_viewer_mtx;
#endif

    void processIO();



public:
    ProcessingInterface(std::vector<CameraType_t> device_count, MainWindowGL *Window);

    std::vector<CloudQueue *>* getInputCloudsRef() { return &m_input_clouds; }
    std::vector<MatQueue *>* getDepthImageRef() { return &m_input_depth; }
    std::vector<MatQueue *>* getColorImageRef() { return &m_input_color; }

    //   void initPcViewer (pcl::visualization::PCLVisualizer& viewer);

    void startThread();

    void setActive(bool running);

    void setSegmentationActive(bool running)
    {
        for (auto segmenter : _segmentThreads)
            segmenter->setActive(running);
    }

    bool isActive();


};

#endif

#ifndef RS2_PCL_CONVERTER_H
#define RS2_PCL_CONVERTER_H

#include <iostream>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>

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
#include "deviceinterface.h"
#include "pclinterface.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Rs2_PCL_Converter
{
private:
    std::vector<shared_references_t> m_refs_conv_to_RS;
    shared_references_t m_PCL_ref;


    std::mutex* m_points_mutex_ref;
    rs2::points* m_points_buf_ref;
    size_t* m_points_write_idx_ref;
    size_t* m_points_read_idx_ref;

    std::mutex* m_clouds_mutex_ref = new std::mutex();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* m_clouds_buf_ref;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_clouds_buf_ref;
    size_t* m_clouds_write_idx_ref;
    size_t* m_clouds_read_idx_ref;

    //  DeviceInterface* m_ref_interface;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>(FRAME_WIDTH, FRAME_HEIGHT));
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>(1280,720));
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_current_cloud;

    std::thread m_converter_thread;
    bool m_running = true;
    uint32_t m_rs2_device_count = 0;
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr());
    // m_dev_thread = std::thread(&Rs2Device::captureClient, this);

    void converter_thread_func()
    {
        std::cout << "Converter thread started # " << std::this_thread::get_id() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>(FRAME_WIDTH_RS*FRAME_HEIGHT_RS*m_rs2_device_count,1));
        //        converted_cloud->resize(converted_cloud->height*converted_cloud->width);

        while (m_running) {


            if (*m_points_read_idx_ref != *m_points_write_idx_ref)
            {
                // Read from rs2::points buffer
                m_points_mutex_ref->lock();
                rs2::points points = m_points_buf_ref[ *m_points_read_idx_ref ];
                *m_points_read_idx_ref = *m_points_read_idx_ref + 1;
                if (*m_points_read_idx_ref == BUF_SIZE_POINTS-1)
                    *m_points_read_idx_ref = 0;
                cout << "(Converter) Increased read index: " << *m_points_read_idx_ref << " size " << points.size() << endl;
                m_points_mutex_ref->unlock();

                // Convert to pcl::PointCloud cloud
                // pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud = points_to_pcl(points);
                points_to_pcl(points, point_cloud);


                // Write to pcl::PointCloud buffer
                m_clouds_mutex_ref->lock();
                m_clouds_buf_ref->at(*m_clouds_write_idx_ref) = point_cloud; // m_current_cloud->makeShared()
                auto cloud_size =  m_clouds_buf_ref->at(*m_clouds_write_idx_ref)->size();
                *m_clouds_write_idx_ref = *m_clouds_write_idx_ref + 1;
                if (*m_clouds_write_idx_ref == BUF_SIZE_POINTS-1)
                    *m_clouds_write_idx_ref = 0;
                cout << "(Converter) Increased write index: " << *m_clouds_write_idx_ref << " size " << cloud_size << endl;
                m_clouds_mutex_ref->unlock();

            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_CONV));
            }

            //  *m_points_write_idx_ref = *m_points_write_idx_ref+1;



            //            if (point_read_index != point_write_index)
            //            {
            //                // Mutex for acquisition thread - read
            //                pthread_mutex_lock( &rs2points_buf_mtx );
            //                rs2::points points = rs2_points_buf[point_read_index++];
            //                cout << "(Converter) Increased read index: " << point_read_index << " size " << points.size() << endl;
            //                if (point_read_index == POINT_BUF_SIZE-1)
            //                    point_read_index = 0;
            //                pthread_mutex_unlock( &rs2points_buf_mtx );
            //                // Convert to pcl cloud
            //                pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud = points_to_pcl(points);
            //                //            if (cloud_write_index != cloud_read_index)
            //                //            {
            //                // Mutex for processing thread - write
            //                pthread_mutex_lock( &pclclouds_buf_mtx );
            //                pcl_clouds_buf[cloud_write_index++] = converted_cloud;
            //                cout << "(Converter) Increased write index: " << cloud_write_index << " size " << converted_cloud->size() << endl;
            //                if (cloud_write_index == POINT_BUF_SIZE-1)
            //                    cloud_write_index = 0;
            //                pthread_mutex_unlock( &pclclouds_buf_mtx );
            //                //   }
            //                //            else {
            //                //                cout << "Skipped writing to cloud buffer, writeIndex " << cloud_write_index << " readIndex " << cloud_read_index << endl;
            //                //            }
            //            }
            //            else {
            //                cout << "Skipped reading from point buffer, writeIndex " << point_write_index << " readIndex " << point_read_index << endl;
            //                //  cout << "Skipped writing to cloud buffer, writeIndex " << cloud_write_index << " readIndex " << cloud_read_index << endl;
            //                std::this_thread::sleep_for(std::chrono::duration<double, std::milli> (CONV_DELAY));
            //            }





        }
    }

    void points_vec_to_pcl(std::vector<rs2::points> *pointsvec, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud)
    {
#if (VERBOSE > 1)
        auto start = std::chrono::steady_clock::now();
#endif
        for (size_t i = 0;i < pointsvec->size();i++) {
            auto Vertex = pointsvec->at(i).get_vertices();
            for (auto& p : pcloud->points)
            {
                p.x = Vertex->x;
                p.y = Vertex->y;
                p.z = Vertex->z;
                Vertex++;
            }
        }


#if (VERBOSE > 1)
        std::cout << "Converted " << pcloud->points.size() << " elements, took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << std::endl;
#endif
    }

    void points_to_pcl(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud)
    {
#if (VERBOSE > 1)
        auto start = std::chrono::steady_clock::now();
#endif
        auto Vertex = points.get_vertices();
#if (VERBOSE > 1)
        std::cout << "rs2::points::get_vertices() took: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << std::endl;
#endif
        for (auto& p : pcloud->points)
        {
            p.x = Vertex->x;
            p.y = Vertex->y;
            p.z = Vertex->z;
            Vertex++;
        }
#if (VERBOSE > 1)
        std::cout << "Converted " << pcloud->points.size() << " elements, took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << std::endl;
#endif
    }


    //    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
    //    {
    //        auto start = std::chrono::steady_clock::now();
    //        //  auto sp = points.get_profile().as<rs2::video_stream_profile>();
    //        auto Vertex = points.get_vertices();
    //        cout << "points.get_vertices() took: " << std::chrono::duration_cast
    //                <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << endl;
    //        //        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>(1280,720));
    //        //        current_cloud->resize(1280*720);
    //        // m_current_cloud->resize(1280*720);
    //        //   pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>(frame_width, frame_height));
    //        //    std::vector<float> x_vec, y_vec, z_vec;
    //        for (auto& p : m_current_cloud->points)
    //        {
    //            p.x = Vertex->x;
    //            p.y = Vertex->y;
    //            p.z = Vertex->z;
    //            //        x_vec.push_back(Vertex->x);
    //            //        y_vec.push_back(Vertex->y);
    //            //        z_vec.push_back(Vertex->z);
    //            Vertex++;
    //        }
    //        cout << "Converted " << m_current_cloud->points.size() << " elements, took " << std::chrono::duration_cast
    //                <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << endl;
    //        //    for (int i=0;i<cloud_size;i++) {
    //        //        //
    //        //        x_vec.push_back(current_cloud->points[i].x);
    //        //        y_vec.push_back(current_cloud->points[i].y);
    //        //        z_vec.push_back(current_cloud->points[i].z);
    //        //        // float max = *max_element(x_vec.begin(), x_vec.end());
    //        //        // cout << i << " - X: "<< x_vec[i] << " Y: "<< y_vec[i] << " Z: "<< z_vec[i] << endl;
    //        //    }
    //        // cout << " - X: "<< x_vec[cloud_size/2] << " Y: "<< y_vec[cloud_size/2] << " Z: "<< z_vec[cloud_size/2] << endl;
    //        //    cout << "X_max: " << *max_element(x_vec.begin(), x_vec.end()) << " X_min: " << *min_element(x_vec.begin(), x_vec.end())
    //        //         << " Y_max: " << *max_element(y_vec.begin(), y_vec.end()) << " Y_min: " << *min_element(y_vec.begin(), y_vec.end())
    //        //         << " Z_max: " << *max_element(z_vec.begin(), z_vec.end()) << " Z_min: " << *min_element(z_vec.begin(), z_vec.end()) << endl;
    //        // current_cloud->points[i].x
    //        //    pcl::PassThrough<pcl::PointXYZ> pass;
    //        //    pass.setInputCloud(current_cloud);
    //        //    pass.setFilterFieldName("z");
    //        //    pass.setFilterLimits(0.0, 1.0);
    //        //    pass.filter(*cloud_filtered);
    //        return m_current_cloud;
    //    }

public:
    Rs2_PCL_Converter(DeviceInterface* in_interface_ref, PclInterface* out_interface_ref , uint32_t device_count)
    //   : m_current_cloud(new pcl::PointCloud<pcl::PointXYZ>(FRAME_WIDTH_RS,FRAME_HEIGHT_RS))
    {
        // m_ref_interface = interface_ref;
        m_rs2_device_count = device_count;
        //  auto test = currCloud;
        std::cout << "Created instance of Rs2PclConverter " << m_rs2_device_count << std::endl;
        //  m_current_cloud->resize(FRAME_WIDTH*FRAME_HEIGHT);
        // need multiple device refs here // std::vector<rs2::points*>

        m_refs_conv_to_RS.resize(device_count);
        for (uint i = 0;i < device_count; i++) {
            // use this struct already inside DeviceInterface !!
            m_refs_conv_to_RS.at(i).mtx_ref = in_interface_ref->getPointsBufferMutex(i);
            m_refs_conv_to_RS.at(i).buf_ref = in_interface_ref->getPointsBufferRef(i);
            m_refs_conv_to_RS.at(i).w_idx_ref = &in_interface_ref->getPointsWriteIndexRef(i);
            m_refs_conv_to_RS.at(i).r_idx_ref = &in_interface_ref->getPointsReadIndexRef(i);

//            rs2::points test = *((rs2::points *) m_refs_conv_to_RS.at(i).buf_ref);
//            std::cout << "DID IT WORK? " <<  test.get_data_size()<<std::endl;

            //    rs2::points *test = m_RS_refs.at(i).buf_ref;

        }

        m_points_mutex_ref = in_interface_ref->getPointsBufferMutex(0);
        m_points_buf_ref = in_interface_ref->getPointsBufferRef(0);
        m_points_write_idx_ref = &in_interface_ref->getPointsWriteIndexRef(0);
        m_points_read_idx_ref = &in_interface_ref->getPointsReadIndexRef(0);

        m_clouds_mutex_ref = out_interface_ref->getCloudsBufferMutex();
        m_clouds_buf_ref = out_interface_ref->getCloudsBufferRef();
        m_clouds_read_idx_ref = &out_interface_ref->getCloudsReadIndexRef();
        m_clouds_write_idx_ref = &out_interface_ref->getCloudsWriteIndexRef();

        //  boost::shared_ptr<>

        m_clouds_buf_ref->resize(BUF_SIZE_CLOUDS);
        //  m_clouds_buffer.push_back( m_current_cloud->makeShared() );

    }
    void startThread()
    {
        if (m_rs2_device_count == 1)
            m_converter_thread = std::thread(&Rs2_PCL_Converter::converter_thread_func, this);
        else std::cout << "Multi cam currently not supported";
    }
};


#endif // RS2_PCL_CONVERTER_H

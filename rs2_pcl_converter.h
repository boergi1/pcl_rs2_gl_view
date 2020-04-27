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

#include <pcl/point_types.h>

#include <pcl/common/transforms.h>

#include "format.h"
#include "customtypes.h"
#include "deviceinterface.h"
#include "pclinterface.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Rs2_PCL_Converter
{
private:
    std::vector<rs2_references_t> m_refs_conv_to_RS;
    std::vector<shared_references_t> m_refs_conv_to_PCL;


    //    std::mutex* m_points_mutex_ref;
    //    rs2::points* m_points_buf_ref;
    //    size_t* m_points_write_idx_ref;
    //    size_t* m_points_read_idx_ref;

    //  std::mutex* m_clouds_mutex_ref = new std::mutex();
    //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* m_clouds_buf_ref;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_clouds_buf_ref;
    // size_t* m_clouds_write_idx_ref;
    // size_t* m_clouds_read_idx_ref;

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
        /* To do:
         * Device 0 runs in the first thread, and starts the other additional (2) threads.
         * First thread waits for others to be finished
         * After that, combine the pointclouds
         *
         * buffer/tmp_pc -> pcl::PointCloud<pcl::PointXYZ> or pcl::PointCloud<pcl::PointXYZ>::Ptr ???
         *
         */
        std::cout << "Converter thread started # " << std::this_thread::get_id() << std::endl;

        //        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>
        //                                                        (FRAME_POINTS_COUNT_RS*m_rs2_device_count,1));

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_POINTS_COUNT,1));

        //        pcl::PointCloud<pcl::PointXYZ> tmp_pc(pcl::PointCloud<pcl::PointXYZ>(FRAME_POINTS_COUNT_RS,1));

        while (m_running) {
#if (VERBOSE > 1)
            auto start = std::chrono::steady_clock::now();
#endif

            //  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            //  point_cloud->reserve(FRAME_POINTS_COUNT_RS*m_rs2_device_count);

            bool wait = true;

            for (size_t i=0; i < m_refs_conv_to_RS.size(); i++)
            {
                if ( *m_refs_conv_to_RS.at(i).r_idx_ref != *m_refs_conv_to_RS.at(i).w_idx_ref )
                {
                    wait = false;

                    // Read from device buffer
                    m_refs_conv_to_RS.at(i).mtx_ref->lock();
                    rs2::points points = static_cast< rs2::points* >( m_refs_conv_to_RS.at(i).buf_ref )
                            [ *m_refs_conv_to_RS.at(i).r_idx_ref ];
                    *m_refs_conv_to_RS.at(i).r_idx_ref = *m_refs_conv_to_RS.at(i).r_idx_ref + 1;
                    if (*m_refs_conv_to_RS.at(i).r_idx_ref == BUF_SIZE_RS2FRAMES-1)
                        *m_refs_conv_to_RS.at(i).r_idx_ref = 0;
                    m_refs_conv_to_RS.at(i).mtx_ref->unlock();

#if (VERBOSE > 1)
                    std::cout << "(Converter) Increased read index (" << rs2PositionToString(m_refs_conv_to_RS.at(i).pos_type)
                              << " device): " << *m_refs_conv_to_RS.at(i).r_idx_ref
                              << " size " << points.size() << std::endl;
#endif

                    //  points_to_pcl(points, pcl::PointCloud<pcl::PointXYZ>::Ptr(&tmp_pc), i+1);

                    points_to_pcl(points, tmp_pc, i);

#ifdef tmp_commented_out
                    // Write to pcl::PointCloud buffer
                    m_refs_conv_to_PCL.at(i).mtx_ref->lock();
                    static_cast< std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* >
                            (m_refs_conv_to_PCL.at(i).buf_ref)->at( *m_refs_conv_to_PCL.at(i).w_idx_ref ) = tmp_pc;
                    *m_refs_conv_to_RS.at(i).w_idx_ref = *m_refs_conv_to_RS.at(i).w_idx_ref + 1;
                    if (*m_refs_conv_to_RS.at(i).w_idx_ref == BUF_SIZE_CLOUDS-1)
                        *m_refs_conv_to_RS.at(i).w_idx_ref = 0;
                    m_refs_conv_to_PCL.at(i).mtx_ref->unlock();
#if (VERBOSE > 1)
                    std::cout << "(Converter) Increased write index (device " << i << "): " << *m_refs_conv_to_PCL.at(i).w_idx_ref
                              << " size " << tmp_pc->size() << std::endl;
#endif

#endif


                    //                m_clouds_buf_ref->at(*m_clouds_write_idx_ref) = point_cloud;
                    //                auto cloud_size =  m_clouds_buf_ref->at(*m_clouds_write_idx_ref)->size();
                    //                *m_clouds_write_idx_ref = *m_clouds_write_idx_ref + 1;
                    //                if (*m_clouds_write_idx_ref == BUF_SIZE_POINTS-1)
                    //                    *m_clouds_write_idx_ref = 0;
                    //                cout << "(Converter) Increased write index: " << *m_clouds_write_idx_ref << " size " << cloud_size << endl;




                    //            m_refs_conv_to_RS.at(i).mtx_ref->lock();
                    //            rs2::points points = static_cast<rs2::points*>( m_refs_conv_to_RS.at(i).buf_ref )
                    //                    [ *m_refs_conv_to_RS.at(i).r_idx_ref ];
                    //            *m_refs_conv_to_RS.at(i).r_idx_ref = *m_refs_conv_to_RS.at(i).r_idx_ref + 1;
                    //            if (*m_refs_conv_to_RS.at(i).r_idx_ref == BUF_SIZE_POINTS-1)
                    //                *m_refs_conv_to_RS.at(i).r_idx_ref = 0;
                    //            m_refs_conv_to_RS.at(i).mtx_ref->unlock();



                }

            }

            // *point_cloud += *tmp_pc;

            if (wait)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_CONV));
                continue;
            }



#if (VERBOSE > 1)
            std::cout << "(Converter) Thread took " << std::chrono::duration_cast<std::chrono::milliseconds>
                         (std::chrono::steady_clock::now()-start).count() << " ms, total PC size: " << tmp_pc->points.size() << std::endl;
#endif

        }


    }

    inline void rs2_transform_point_to_point_custom(float* to_point, const struct rs2_extrinsics* extrin, const float* from_point)
    {
        to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
        to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
        to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
    }

    inline void points_to_pcl(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud, size_t dev_idx)
    {
#if (VERBOSE > 2)
        auto start = std::chrono::high_resolution_clock::now();
#endif
        int skipped = 0;
        auto rs_vertex = points.get_vertices();

        switch (dev_idx) {
        case 0: // no transformation
            for (auto& pcl_point : pcloud->points)
            {
                pcl_point.x = rs_vertex->x;
                pcl_point.y = rs_vertex->y;
                pcl_point.z = rs_vertex->z;
                rs_vertex++;
            }
            break;
        case 1:
        {
            float rad = static_cast<float>(degreesToRadians(ROT_RS1_TO_RS0_X_ANG));
            rs2_extrinsics extrinsics_T_Rx;
            extrinsics_T_Rx.rotation[0] = 1; extrinsics_T_Rx.rotation[3] = 0;             extrinsics_T_Rx.rotation[6] = 0;
            extrinsics_T_Rx.rotation[1] = 0; extrinsics_T_Rx.rotation[4] = std::cos(rad); extrinsics_T_Rx.rotation[7] = -std::sin(rad);
            extrinsics_T_Rx.rotation[2] = 0; extrinsics_T_Rx.rotation[5] = std::sin(rad); extrinsics_T_Rx.rotation[8] = std::cos(rad);
            extrinsics_T_Rx.translation[0] = static_cast<float>(TRAN_RS1_TO_RS0_X_M);
            extrinsics_T_Rx.translation[1] = static_cast<float>(TRAN_RS1_TO_RS0_Y_M);
            extrinsics_T_Rx.translation[2] = static_cast<float>(TRAN_RS1_TO_RS0_Z_M);
#if (VERBOSE > 3)
            std::cerr << "Transformation matrix:" << std::endl
                      << extrinsics_T_Rx.rotation[0] << " " << extrinsics_T_Rx.rotation[3] << " " << extrinsics_T_Rx.rotation[6] << " " << extrinsics_T_Rx.translation[0] << std::endl
                      << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[7] << " " << extrinsics_T_Rx.translation[1] << std::endl
                      << extrinsics_T_Rx.rotation[2] << " " << extrinsics_T_Rx.rotation[5] << " " << extrinsics_T_Rx.rotation[8] << " " << extrinsics_T_Rx.translation[2] << std::endl;
#endif
            for (auto& pcl_point : pcloud->points)
            {
                if (areSameF(0.0f, rs_vertex->z))
                {
                    skipped++;
                    pcl_point.x = rs_vertex->x;
                    pcl_point.y = rs_vertex->y;
                    pcl_point.z = rs_vertex->z;
                    rs_vertex++;
                    continue;
                }
                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
                float target[3];
                rs2_transform_point_to_point_custom(target, &extrinsics_T_Rx, origin);
                pcl_point.x = target[0];
                pcl_point.y = target[1];
                pcl_point.z = target[2];
                rs_vertex++;
#if (VERBOSE > 4)
                std::cerr << "Point before: (" << origin[0] << "," << origin[1] << "," << origin[2] << ")\t" <<
                             "Point after: (" << target[0] << "," << target[1] << "," << target[2] << ")" << std::endl;
#endif
            }
            break;
        }
        case 2:
        {
            float rad = static_cast<float>(degreesToRadians(ROT_RS2_TO_RS0_X_ANG));
            rs2_extrinsics extrinsics_T_Rx;
            extrinsics_T_Rx.rotation[0] = 1; extrinsics_T_Rx.rotation[3] = 0;             extrinsics_T_Rx.rotation[6] = 0;
            extrinsics_T_Rx.rotation[1] = 0; extrinsics_T_Rx.rotation[4] = std::cos(rad); extrinsics_T_Rx.rotation[7] = -std::sin(rad);
            extrinsics_T_Rx.rotation[2] = 0; extrinsics_T_Rx.rotation[5] = std::sin(rad); extrinsics_T_Rx.rotation[8] = std::cos(rad);
            extrinsics_T_Rx.translation[0] = static_cast<float>(TRAN_RS2_TO_RS0_X_M);
            extrinsics_T_Rx.translation[1] = static_cast<float>(TRAN_RS2_TO_RS0_Y_M);
            extrinsics_T_Rx.translation[2] = static_cast<float>(TRAN_RS2_TO_RS0_Z_M);
#if (VERBOSE > 3)
            std::cerr << "Transformation matrix:" << std::endl
                      << extrinsics_T_Rx.rotation[0] << " " << extrinsics_T_Rx.rotation[3] << " " << extrinsics_T_Rx.rotation[6] << " " << extrinsics_T_Rx.translation[0] << std::endl
                      << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[7] << " " << extrinsics_T_Rx.translation[1] << std::endl
                      << extrinsics_T_Rx.rotation[2] << " " << extrinsics_T_Rx.rotation[5] << " " << extrinsics_T_Rx.rotation[8] << " " << extrinsics_T_Rx.translation[2] << std::endl;
#endif
            for (auto& pcl_point : pcloud->points)
            {
                if (areSameF(0.0f, rs_vertex->z))
                {
                    skipped++;
                    pcl_point.x = rs_vertex->x;
                    pcl_point.y = rs_vertex->y;
                    pcl_point.z = rs_vertex->z;
                    rs_vertex++;
                    continue;
                }
                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
                float target[3];
                rs2_transform_point_to_point_custom(target, &extrinsics_T_Rx, origin);
                pcl_point.x = target[0];
                pcl_point.y = target[1];
                pcl_point.z = target[2];
                rs_vertex++;
#if (VERBOSE > 4)
                std::cerr << "Point before: (" << origin[0] << "," << origin[1] << "," << origin[2] << ")\t" <<
                             "Point after: (" << target[0] << "," << target[1] << "," << target[2] << ")" << std::endl;
#endif
            }
            break;
        }
        default:
            std::cerr << "(Converter) Error: invalid device index" << std::endl;
        }
#if (VERBOSE > 2)
        std::cout << "(Converter) RS device " << dev_idx << " converted " << pcloud->points.size() << " elements, skipped " << skipped << ", took " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
    }



public:
    Rs2_PCL_Converter(DeviceInterface* in_interface_ref, PclInterface* out_interface_ref , uint32_t device_count)
    //   : m_current_cloud(new pcl::PointCloud<pcl::PointXYZ>(FRAME_WIDTH_RS,FRAME_HEIGHT_RS))
    {
        m_rs2_device_count = device_count;

        std::cout << "Created instance of Rs2PclConverter" << std::endl;

        m_refs_conv_to_RS = in_interface_ref->get_rs_data_refs();
        m_refs_conv_to_PCL = out_interface_ref->get_pcl_data_refs();

        //        m_refs_conv_to_RS.resize(device_count);
        //        for (uint i = 0;i < device_count; i++) {
        //            // use this struct already inside DeviceInterface !!
        //            m_refs_conv_to_RS.at(i).mtx_ref = in_interface_ref->getPointsBufferMutex(i);
        //            m_refs_conv_to_RS.at(i).buf_ref = in_interface_ref->getPointsBufferRef(i);
        //            m_refs_conv_to_RS.at(i).w_idx_ref = &in_interface_ref->getPointsWriteIndexRef(i);
        //            m_refs_conv_to_RS.at(i).r_idx_ref = &in_interface_ref->getPointsReadIndexRef(i);
        ////            rs2::points test = *((rs2::points *) m_refs_conv_to_RS.at(i).buf_ref);
        ////            std::cout << "DID IT WORK? " <<  test.get_data_size()<<std::endl;
        //            //    rs2::points *test = m_RS_refs.at(i).buf_ref;
        //        }
        //        m_points_mutex_ref = in_interface_ref->getPointsBufferMutex(0);
        //        m_points_buf_ref = in_interface_ref->getPointsBufferRef(0);
        //        m_points_write_idx_ref = &in_interface_ref->getPointsWriteIndexRef(0);
        //        m_points_read_idx_ref = &in_interface_ref->getPointsReadIndexRef(0);

        //        m_clouds_mutex_ref = out_interface_ref->getCloudsBufferMutex();
        //        m_clouds_buf_ref = out_interface_ref->getCloudsBufferRef();
        //        m_clouds_read_idx_ref = &out_interface_ref->getCloudsReadIndexRef();
        //        m_clouds_write_idx_ref = &out_interface_ref->getCloudsWriteIndexRef();
        //  m_clouds_buf_ref->resize(BUF_SIZE_CLOUDS);

    }



    void startThread()
    {
        if (m_rs2_device_count == 1)
            m_converter_thread = std::thread(&Rs2_PCL_Converter::converter_thread_func, this);
        else std::cout << "Multi cam currently not supported";
    }
};





//    void points_vec_to_pcl(std::vector<const rs2::points&> pointsvec, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud)
//    {
//#if (VERBOSE > 1)
//        auto start = std::chrono::steady_clock::now();
//#endif
//        size_t pt_vec_size = pointsvec.size();
//        for (size_t i = 0; i < pt_vec_size; i++) {
//            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>
//                                                       (FRAME_POINTS_COUNT_RS,1));
//            const rs2::points& points = pointsvec.at(i);
//            auto rs_vertex = points.get_vertices();
//            for (auto& pcl_point : tmp_pc->points)
//            {
//                pcl_point.x = rs_vertex->x;
//                pcl_point.y = rs_vertex->y;
//                pcl_point.z = rs_vertex->z;
//                rs_vertex++;
//            }

//            Eigen::Affine3f transform = Eigen::Affine3f::Identity(); // T in meters, R in rad
//            switch (i) {
//            case 0:
//            {
//                // No transformation
//                break;
//            }
//            case 1:
//            {
//                transform.translation() << static_cast<float>( TRAN_RS1_TO_RS0_X_MM/1000 ),
//                        static_cast<float>( TRAN_RS1_TO_RS0_Y_MM/1000 ),
//                        static_cast<float>( TRAN_RS1_TO_RS0_Z_MM/1000 );
//                transform.rotate (Eigen::AngleAxisf (static_cast<float>( degreesToRadians(ROT_RS1_TO_RS0_X_ANG) ),
//                                                     Eigen::Vector3f::UnitX()));
//                pcl::transformPointCloud (*tmp_pc, *tmp_pc, transform);
//                break;
//            }
//            case 2:
//            {
//                transform.translation() << static_cast<float>( TRAN_RS2_TO_RS0_X_MM/1000 ),
//                        static_cast<float>( TRAN_RS2_TO_RS0_Y_MM/1000 ),
//                        static_cast<float>( TRAN_RS2_TO_RS0_Z_MM/1000 );
//                transform.rotate (Eigen::AngleAxisf (static_cast<float>( degreesToRadians(ROT_RS2_TO_RS0_X_ANG) ),
//                                                     Eigen::Vector3f::UnitX()));
//                pcl::transformPointCloud (*tmp_pc, *tmp_pc, transform);
//                break;
//            }
//            default:
//                std::cerr << "(Converter) Invalid device index" << std::endl;
//            }
//#if (VERBOSE > 1)
//                std::cout << "(Converter) Tranformation matrix for device " << i
//                          << std::endl << transform.matrix();
//#endif
//            *pcloud += *tmp_pc;
//        }
//        // old try ....
//        //        std::vector<const rs2::vertex*> vertices;
//        //        vertices.resize(pointsvec->size());
//        //        for (size_t i = 0;i < pointsvec->size();i++) {
//        //            auto Vertex = pointsvec->at(i).get_vertices();
//        //            vertices.push_back(Vertex);
//        //        }
//        //        for (size_t i=0; i<pcloud->points.size(); i++)
//        //        {
//        //            auto pclpoint = pcloud->points.at(i);
//        //            // auto test = pcloud->at(i);
//        //            if (i < FRAME_POINTS_COUNT)
//        //            {
//        //                pclpoint.x = vertices.at(0)->x;
//        //                pclpoint.y = vertices.at(0)->y;
//        //                pclpoint.z = vertices.at(0)->z;
//        //            }
//        //            else if (i >= FRAME_POINTS_COUNT && i < FRAME_POINTS_COUNT*2)
//        //            {
//        //                pclpoint.x = vertices.at(1)->x;
//        //                pclpoint.y = vertices.at(1)->y;
//        //                pclpoint.z = vertices.at(1)->z;
//        //            }
//        //            else if (i >= FRAME_POINTS_COUNT*2)
//        //            {
//        //                pclpoint.x = vertices.at(2)->x;
//        //                pclpoint.y = vertices.at(2)->y;
//        //                pclpoint.z = vertices.at(2)->z;
//        //            }
//        //        }
//#if (VERBOSE > 1)
//        std::cout << "(Converter) Converted " << pcloud->points.size() << " elements, took " << std::chrono::duration_cast
//                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() << " ms" << std::endl;
//#endif
//    }






#endif // RS2_PCL_CONVERTER_H

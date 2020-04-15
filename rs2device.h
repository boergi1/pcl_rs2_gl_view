#ifndef RS2DEVICE_H
#define RS2DEVICE_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <thread>
#include <mutex>
//#include <deviceinterface.h>

#include "format.h"
#include "customtypes.h"



class Rs2Device
{
private:
    //  std::mutex* m_mutex_ref = nullptr;

    // size_t* m_points_write_idx_ref = nullptr;

    shared_references_t m_ref_RS_to_interface;

    //   void*               start_flag;

    void start_device_thread();



    rs2::device m_rs2_dev;
    std::string m_serial_num;
    std::vector<rs2::sensor> m_rs2_sensors;



    rs2::colorizer m_rs2_colorizer;
    rs2::rates_printer m_rs2_printer;
    rs2::pipeline m_rs2_pipe;
    std::string m_name;

    rs2::pointcloud m_curr_rs2_pc_cpu;
    rs2::points m_curr_rs2_points_cpu;

    // filters
    rs2::decimation_filter m_dec_filter = rs2::decimation_filter(2.0f);                 // Decimation - reduces depth frame density
    rs2::threshold_filter m_thr_filter = rs2::threshold_filter(0.4f, 8.0f);             // Threshold  - removes values outside recommended range
    rs2::spatial_filter m_spat_filter = rs2::spatial_filter(0.65f, 29.0f, 2.0f, 2.0f);  // Spatial    - edge-preserving spatial smoothing


    bool m_running = true;
    bool m_use_polling = false;
    bool m_use_gpu_capture = false;

    std::string get_device_name(const rs2::device& dev);
    std::string get_device_id(const rs2::device &dev);

    std::string get_sensor_name(const rs2::sensor& sensor);

    void print_sensor_options(const rs2::sensor& sensor);

    void streamVideo();

    void print_device_information(const rs2::device& dev);



public:
    Rs2Device(rs2::device &dev, shared_references_t data_ref);

    void captureClient(bool running);
    //  rs2::points* m_rs2_points_buf_ref;


    std::function<void (rs2::frame)> depth_callback = [&](const rs2::frame& frame)
    {


        // std::cout << "callback thread # " << std::this_thread::get_id() << std::endl;
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
#if (VERBOSE > 0)
            auto start = std::chrono::high_resolution_clock::now();
#endif
            if (fs.size() > 1)
                std::cerr << "(Realsense device) Multiple frames arrived: " << fs.size() << std::endl;
            const rs2::frame& depth_tmp = fs.get_depth_frame();
            if (!depth_tmp)
            {
                std::cerr << "(Realsense device) No depth frame" << std::endl;
                return;
            }


            //            m_dec_filter.process(depth_tmp);
            //            m_thr_filter.process(depth_tmp);
            // m_spat_filter.process(depth_tmp);




            rs2::pointcloud rs2_pc_cpu;
            rs2_pc_cpu.map_to(depth_tmp);
            m_curr_rs2_points_cpu = rs2_pc_cpu.calculate(depth_tmp);


            m_ref_RS_to_interface.mtx_ref->lock();
            static_cast<rs2::points*>( m_ref_RS_to_interface.buf_ref )[ *m_ref_RS_to_interface.w_idx_ref ]
            = m_curr_rs2_points_cpu;
            *m_ref_RS_to_interface.w_idx_ref = *m_ref_RS_to_interface.w_idx_ref + 1;
            std::cout << "(Realsense device) Increased write index (cpu): " << *m_ref_RS_to_interface.w_idx_ref
                      << " size " << m_curr_rs2_points_cpu.size() << std::endl;
            if (*m_ref_RS_to_interface.w_idx_ref == BUF_SIZE_POINTS-1)
                *m_ref_RS_to_interface.w_idx_ref = 0;
            m_ref_RS_to_interface.mtx_ref->unlock();






            //            m_mutex_ref->lock();
            //            m_rs2_points_buf_ref[*m_points_write_idx_ref] = m_curr_rs2_points_cpu;
            //            *m_points_write_idx_ref = *m_points_write_idx_ref + 1;
            //            std::cout << "(Frames) Increased write index (cpu): " << *m_points_write_idx_ref << " size " << m_curr_rs2_points_cpu.size() << std::endl;
            //            if (*m_points_write_idx_ref == BUF_SIZE_POINTS-1)
            //                *m_points_write_idx_ref = 0;
            //            m_mutex_ref->unlock();

            //            // With callbacks, all synchronized stream will arrive in a single frameset
            //            for (const rs2::frame& f : fs)
            //            {
            //               // counters[f.get_profile().unique_id()]++;
            //            }
#if (VERBOSE > 1)
            std::cout << "(Realsense device) Callback took " << std::chrono::duration_cast<std::chrono::milliseconds>
                         (std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
        }
        else
        {
            std::cerr << "(Realsense device) Unhandled frame arrived: " << fs.size() << std::endl;
            // Stream that bypass synchronization (such as IMU) will produce single frames
            //  counters[frame.get_profile().unique_id()]++;
        }



    };

};


#endif // RS2DEVICE_H

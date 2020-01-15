#include "rs2device.h"


std::string Rs2Device::get_device_name(const rs2::device &dev)
{
    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);
    //    std::string sn = "########";
    //    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
    //        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    return name;
}

std::string Rs2Device::get_device_id(const rs2::device &dev)
{
    std::string sn = "Unknown ID";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    return sn;
}

std::string Rs2Device::get_sensor_name(const rs2::sensor &sensor)
{
    if (sensor.supports(RS2_CAMERA_INFO_NAME))
        return sensor.get_info(RS2_CAMERA_INFO_NAME);
    else
        return "Unknown Sensor";
}

void Rs2Device::print_sensor_options(const rs2::sensor &sensor)
{
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        if (sensor.supports(option_type))
        {
            std::cout << "    " << i << " : " << option_type << std::endl;
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;
        }
    }
}

void Rs2Device::print_device_information(const rs2::device &dev)
{
    std::cout << "Device information: " << std::endl;
    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
    {
        rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
        std::cout << "  " << info_type << " : ";
        if (dev.supports(info_type))
            std::cout << dev.get_info(info_type) << std::endl;
        else
            std::cout << "N/A" << std::endl;
    }
}

void Rs2Device::captureClient(bool running)
{
    if (running)
    {
        rs2::config rs2_cfg;
        rs2_cfg.enable_device(m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        rs2_cfg.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);
        std::cout << "Starting pipe of " << m_serial_num << std::endl;
        m_rs2_pipe = rs2::pipeline();
        rs2::pipeline_profile pipe_profile = m_rs2_pipe.start(rs2_cfg, depth_callback );
        std::cout << "Enabled streams:";
        for (auto p : pipe_profile.get_streams())
            std::cout << " " << p.stream_name();
        std::cout << std::endl;
    }
    else {
        m_rs2_pipe.stop();
    }




    //    while (m_running) {
    //        auto start = std::chrono::steady_clock::now();
    //        // Any new frames?
    //        rs2::frameset frames;
    //        std::cout << "before frame" << std::endl;
    //        if (!m_use_polling)
    //            frames = m_rs2_pipe.wait_for_frames().apply_filter(m_rs2_printer);
    //        else
    //            m_rs2_pipe.poll_for_frames(&frames);
    //        std::cout << "after frame" << std::endl;

    //        rs2::depth_frame depth_tmp  = frames.get_depth_frame();
    //        rs2::pointcloud rs2_pc_cpu;
    //        rs2_pc_cpu.map_to(depth_tmp);
    //        m_curr_rs2_points_cpu = rs2_pc_cpu.calculate(depth_tmp);
    //        std::string rs2_ts_domain;
    //        switch( depth_tmp.get_frame_timestamp_domain() )
    //        {
    //        case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
    //            rs2_ts_domain = "HW_CLK"; break;
    //        case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
    //            rs2_ts_domain = "SYS_TIME"; break;
    //        case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
    //            rs2_ts_domain = "GLO_TIME"; break;
    //        default: rs2_ts_domain = "NONE"; break;
    //        }

    //        std::cout << "Frame: #" << depth_tmp.get_frame_number() << ": " << depth_tmp.get_timestamp() << " size: "
    //                  << depth_tmp.get_data_size() << " domain: " << rs2_ts_domain << std::endl;

    //        rs2points_buf_mtx.lock();
    //        rs2_points_buf[point_write_index++] = m_curr_rs2_points_cpu;
    //        std::cout << "(Frames) Increased write index (cpu): " << point_write_index << std::endl;
    //        if (point_write_index == POINT_BUF_SIZE-1)
    //            point_write_index = 0;
    //        rs2points_buf_mtx.unlock();

    //        std::cout << "Device thread " << m_name << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>
    //                     (std::chrono::steady_clock::now()-start).count() << " ms" << std::endl;

    //    }
    //    m_rs2_pipe.stop();
    //    std::cout << "Device thread" << m_name << " ended" << std::endl;




}

Rs2Device::Rs2Device(rs2::device &dev, std::mutex* mutex, rs2::points *buffer, size_t& write_idx_ref)
{
    // std::map <size_t, size_t> hi;
    //  m_parent_ref = parent_ref;
    m_mutex_ref = mutex;
    //  start_flag = t_start_flag;

    m_rs2_dev = dev;
    //m_rs2_pipe_ptr = pipe;
    m_name = get_device_name(m_rs2_dev);
    m_rs2_points_buf_ref = buffer; //new rs2::points[POINT_BUF_SIZE];
    m_points_write_idx_ref = &write_idx_ref;
    * m_points_write_idx_ref = 0;
    // m_rs2_sensors = dev.query_sensors();
    m_use_polling = false;
    m_use_gpu_capture = false;

    m_serial_num = get_device_id(m_rs2_dev);

    std::cout << "New Realsense device instance: " << m_name << " id: " << m_serial_num
              << " buffer: " << m_rs2_points_buf_ref << " mutex: " << m_mutex_ref << std::endl;


    //    std::cout << "Sensors:" << std::endl;
    //    for (uint32_t i=0;i<m_rs2_sensors.size();i++)
    //    {
    //        std::cout << "  #" << i << " : " <<  get_sensor_name(m_rs2_sensors[i]) << std::endl;
    //        print_sensor_options(m_rs2_sensors[i]);
    //    }
    print_device_information(m_rs2_dev);


    if (!m_rs2_dev)
    {
        std::cout << "Not valid, returning" << std::endl;
        return;
    }


    captureClient(true);
    //   rs2_pipe.start(rs2_cfg);
    // rs2::pipeline_profile rs2_pipe_profile = rs2_pipe.start(rs2_cfg);




    //    std::cout << "Starting device thread: " << m_name << std::endl;
    //    m_dev_thread = std::thread(&Rs2Device::captureClient, this);

    // m_dev_thread.join();


}

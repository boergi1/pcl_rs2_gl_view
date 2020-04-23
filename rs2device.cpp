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

void Rs2Device::setCaptureEnabled(bool running)
{    
    std::cout << std::endl << "setCaptureEnabled begin " << getPositionType() << std::endl << std::endl;
    if (running)
    {
        std::string rs2_serial = m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "(Rs2Device) Enabling capture of " << getPositionType() << " #" << rs2_serial << std::endl;


//        m_rs2_dev.hardware_reset();
//        std::this_thread::sleep_for(std::chrono::seconds(3));


#if (RS_MASTER_SLAVE_CONFIG == 1)
        auto advanced_dev = m_rs2_dev.as<rs400::advanced_mode>();
        auto advanced_sensors = advanced_dev.query_sensors();
        rs2::sensor depth_sensor;

        for (auto&& sensor : advanced_sensors) {
            std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
            if (module_name.compare("Stereo Module") == 0)
            {
                depth_sensor = sensor;
            }
        }

        if (depth_sensor)
        {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500); // microseconds
            depth_sensor.set_option(RS2_OPTION_GAIN, 16);
            depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
        }
        else {
            std::cerr << "(Rs2Device) No depth sensor found #" << rs2_serial << std::endl;
            return;
        }
        // Master/slave sync configurations
        if (m_ref_RS_to_interface.pos_type == Rs2Position_t::CENTRAL)
        {
            // master
            depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
            std::cout << "(Rs2Device) Camera " << getPositionType() << " #" << rs2_serial << " set to MASTER (val="
                      << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
        }
        else
        {
            //slave
            depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
            std::cout << "(Rs2Device) Camera " << getPositionType() << " #" << rs2_serial << " set to SLAVE (val="
                      << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
        }
#endif
        m_rs2_pipe = rs2::pipeline();
        // rs2::pipeline rs2_pipe;
        rs2::config rs2_cfg;
        rs2_cfg.enable_device(rs2_serial);
        rs2_cfg.enable_stream(RS2_STREAM_DEPTH, RS_FRAME_WIDTH, RS_FRAME_HEIGHT, RS2_FORMAT_Z16, RS_FRAME_RATE);
        rs2_cfg.disable_stream(RS2_STREAM_COLOR);

        rs2::pipeline_profile rs2_profile = m_rs2_pipe.start(rs2_cfg, depth_callback);

        //        rs2::pipeline_profile pipe_profile = m_rs2_pipe.start(rs2_cfg);
        //        while (true) {
        //            auto frames = m_rs2_pipe.wait_for_frames();
        //            std::cerr << "DEBUG frames arrived: " << frames.size() << std::endl;
        //        }


#if (VERBOSE > 3)
        std::cout << std::endl << "(Rs2Device) Enabled streams:" << std::endl;
        for (auto p : rs2_profile.get_streams())
        {
            // auto sp = p.as<rs2::stream_profile>();
            auto vsp = p.as<rs2::video_stream_profile>();
            auto name = vsp.stream_name();
            auto width = vsp.width();
            auto height = vsp.height();
            auto fps = vsp.fps();
            auto format = vsp.format();
            auto intr = vsp.get_intrinsics();
            auto principal_point = std::make_pair(intr.ppx, intr.ppy);
            auto focal_length = std::make_pair(intr.fx, intr.fy);
            auto distortion_model = intr.model;
            auto distortion_coeffs = intr.coeffs;
            float fov[2]; // X, Y fov
            rs2_fov(&intr, fov);
            std::cout << name << ": " << width << "x" << height << "  fps: " << fps << "  format: " << format
                      << "  principal point: (" << principal_point.first << "," << principal_point.second
                      << ")  focal length: fx="<< focal_length.first << " fy=" << focal_length.second
                      << "  distortion model: " << distortion_model << "  distortion matrix: ["
                      << distortion_coeffs[0] << " " << distortion_coeffs[1] << " " << distortion_coeffs[2]
                      << " " << distortion_coeffs[3] << " " << distortion_coeffs[4]
                      << "]  FOV: x=" << fov[0] << "° y=" << fov[1] << "°" << std::endl << std::endl;
        }
#endif

    }
    else {
        std::cout << "(Rs2Device) Stopping pipe" << std::endl;
        m_rs2_pipe.stop();
    }

    m_active = running;



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



    std::cout << std::endl << "setCaptureEnabled end " << getPositionType() << std::endl << std::endl;
}

bool Rs2Device::isActive() { return m_active; }

Rs2Device::Rs2Device(rs2::device &dev, shared_references_s data_ref)
{

    m_ref_RS_to_interface = data_ref;
    m_rs2_dev = dev;
    std::cout << "New Realsense device, type: "<< getPositionType() << " #" << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    // print_device_information(m_rs2_dev);

    // setCaptureEnabled(true);



    // m_rs2_sensors = dev.query_sensors();
    //    std::cout << "Sensors:" << std::endl;
    //    for (uint32_t i=0;i<m_rs2_sensors.size();i++)
    //    {
    //        std::cout << "  #" << i << " : " <<  get_sensor_name(m_rs2_sensors[i]) << std::endl;
    //        print_sensor_options(m_rs2_sensors[i]);
    //    }

    //   rs2_pipe.start(rs2_cfg);
    // rs2::pipeline_profile rs2_pipe_profile = rs2_pipe.start(rs2_cfg);




    //    std::cout << "Starting device thread: " << m_name << std::endl;
    //    m_dev_thread = std::thread(&Rs2Device::captureClient, this);

    // m_dev_thread.join();


}

Rs2Device::~Rs2Device()
{
    std::cout << "(Rs2Device) Deleting rs2 camera #"
              << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

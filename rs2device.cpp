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

void Rs2Device::rs2_capture_thread_func()
{
    std::string serial = m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "(Rs2Device) Capture thread started: #" << serial << " threadID: " << std::this_thread::get_id() << std::endl;

#if (RS_MASTER_SLAVE_CONFIG == 1)
    auto advanced_dev = m_rs2_dev.as<rs400::advanced_mode>();

    auto advanced_sensors = advanced_dev.query_sensors();

    bool depth_found = false;
    bool color_found = false;
    rs2::sensor depth_sensor;
    rs2::sensor color_sensor;
    for (auto&& sensor : advanced_sensors) {
        std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        if (module_name == "Stereo Module") {
            depth_sensor = sensor;
            depth_found = true;
        } else if (module_name == "RGB Camera") {
            color_sensor = sensor;
            color_found = true;
        }
    }

    if (depth_found)
    {
        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500); // microseconds
        depth_sensor.set_option(RS2_OPTION_GAIN, 16);
        depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
    }
    else if (color_found)
    {
        color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        color_sensor.set_option(RS2_OPTION_EXPOSURE, 100); // 1/10 ms (10)
        color_sensor.set_option(RS2_OPTION_GAIN, 64);
        color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
    }
    else {
        std::cerr << "(Rs2Device) No depth sensor found #" << serial << std::endl;
        return;
    }

    // RGB sync doesn't work, need to use depth as master.
    if (m_rs2_dev_id == 0) {
        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
        std::cout << "(Rs2Device) Camera " << getPositionTypeStr() << " #" << serial << " set to MASTER (val="
                  << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
    } else {
        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
        std::cout << "(Rs2Device) Camera " << getPositionTypeStr() << " #" << serial << " set to SLAVE (val="
                  << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
    }
#endif

    rs2::pipeline rs2_pipe;
    rs2::config rs2_cfg;
    rs2_cfg.enable_device(serial);
    rs2_cfg.enable_stream(RS2_STREAM_DEPTH, RS_FRAME_WIDTH, RS_FRAME_HEIGHT, RS2_FORMAT_Z16, RS_FRAME_RATE);
    rs2_cfg.disable_stream(RS2_STREAM_COLOR);

    rs2::pipeline_profile rs2_profile = rs2_pipe.start(rs2_cfg);

#if (VERBOSE > 2)
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

    m_capture_running = true;
    while (m_capture_running) {
        rs2::frameset frames = rs2_pipe.wait_for_frames();

        auto cap_start = std::chrono::high_resolution_clock::now();

        // rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        double drift = depth_frame.get_timestamp() - m_last_frame_time;
        m_last_frame_time = depth_frame.get_timestamp();


#if (VERBOSE > 2)

        std::cout << frames.size() << " frame(s) from #" << serial << ": ";
        std::cout.precision(std::numeric_limits<double>::max_digits10);
        std::cout << "Drift: " << drift << ", ";
        switch(depth_frame.get_frame_timestamp_domain()) {
        case (RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK):
            // Frame timestamp was measured in relation to the camera clock
            std::cout << "Hardware Clock ";
            break;
        case (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME):
            // Frame timestamp was measured in relation to the OS system clock
            std::cout << "System Time ";
            break;
        case (RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME):
            // Frame timestamp was measured in relation to the camera clock and converted to OS system clock by constantly measure the difference
            std::cout << "Global Time ";
            break;
        case (RS2_TIMESTAMP_DOMAIN_COUNT):
            // Number of enumeration values. Not a valid input: intended to be used in for-loops.
            std::cout << "Domain Count ";
            break;
        default:
            std::cout << "Unknown ";
            break;
        }
        std::cout << "TS: " << m_last_frame_time << " (" << depth_frame.get_frame_number() << "), size: " << depth_frame.get_data_size() << std::endl;



//        const rs2_frame* frame_handle = depth_frame.get();
//        if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_FRAME_TIMESTAMP, nullptr) )
//        {
//            rs2_metadata_type frame_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_FRAME_TIMESTAMP, nullptr);
//            std::cout << "DEBUG RS2_FRAME_METADATA_FRAME_TIMESTAMP " << frame_ts_meta << std::endl;
//        }
//        if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, nullptr) )
//        {
//            rs2_metadata_type sensor_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, nullptr);
//            std::cout << "DEBUG RS2_FRAME_METADATA_SENSOR_TIMESTAMP " << sensor_ts_meta << std::endl;
//        }
//        if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, nullptr) )
//        {
//            rs2_metadata_type arrival_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, nullptr);
//            std::cout << "DEBUG RS2_FRAME_METADATA_TIME_OF_ARRIVAL " << arrival_ts_meta << std::endl;
//        }
//        if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_BACKEND_TIMESTAMP, nullptr) )
//        {
//            rs2_metadata_type backend_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_BACKEND_TIMESTAMP, nullptr);
//            std::cout << "DEBUG RS2_FRAME_METADATA_BACKEND_TIMESTAMP " << backend_ts_meta << std::endl;
//        }

#endif
        // Filters
        m_dec_filter.process(depth_frame);
        m_thr_filter.process(depth_frame);
        m_spat_filter.process(depth_frame);

        m_frame_queue.enqueue(depth_frame);





        //        m_ref_RS_to_interface.mtx_ref->lock();
        //        static_cast<rs2::frame*>( m_ref_RS_to_interface.buf_ref )[ *m_ref_RS_to_interface.w_idx_ref ] = depth_frame;
        //        *m_ref_RS_to_interface.w_idx_ref = *m_ref_RS_to_interface.w_idx_ref + 1;
        //        std::cout << "(Rs2Device " << getPositionTypeStr() << ") Increased write index: " << *m_ref_RS_to_interface.w_idx_ref
        //                  << " size " << depth_frame.get_data_size() << std::endl;
        //        if (*m_ref_RS_to_interface.w_idx_ref == BUF_SIZE_RS2FRAMES-1)
        //            *m_ref_RS_to_interface.w_idx_ref = 0;
        //        m_ref_RS_to_interface.mtx_ref->unlock();




        auto cap_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-cap_start).count();
        if (cap_end >= drift)
            std::cerr << "(Rs2Device) Processing takes longer than capturing" << std::endl;
#if (VERBOSE > 2)
        std::cout << "(Rs2Device) " << getPositionTypeStr() << " capture took " << cap_end << " ms" << std::endl;
#endif
    }

    rs2_pipe.stop();

    std::cout << "(Rs2Device) Capture thread stopped: #" << serial << " threadID: " << std::this_thread::get_id() << std::endl;
}

void Rs2Device::setCaptureEnabled(bool running)
{    
    std::cout << std::endl << "DEBUG setCaptureEnabled start " << getPositionTypeStr() << std::endl << std::endl;
    std::string serial = m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (running)
    {
        std::cout << "(Rs2Device) Enabling capture of " << getPositionTypeStr() << " #" << serial << std::endl;
        m_capture_thread = std::thread(&Rs2Device::rs2_capture_thread_func, this);

        //#if (RS_MASTER_SLAVE_CONFIG == 1)
        //        auto advanced_dev = m_rs2_dev.as<rs400::advanced_mode>();
        //        auto advanced_sensors = advanced_dev.query_sensors();
        //        rs2::sensor depth_sensor;
        //        for (auto&& sensor : advanced_sensors) {
        //            std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        //            if (module_name.compare("Stereo Module") == 0)
        //            {
        //                depth_sensor = sensor;
        //            }
        //        }
        //        if (depth_sensor)
        //        {
        //            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        //            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500); // microseconds
        //            depth_sensor.set_option(RS2_OPTION_GAIN, 16);
        //            depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
        //        }
        //        else {
        //            std::cerr << "(Rs2Device) No depth sensor found #" << serial << std::endl;
        //            return;
        //        }
        //        // Master/slave sync configurations
        //        if (m_ref_RS_to_interface.pos_type == Rs2Position_t::CENTRAL)
        //        {
        //            // master
        //            depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
        //            std::cout << "(Rs2Device) Camera " << getPositionType() << " #" << serial << " set to MASTER (val="
        //                      << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
        //        }
        //        else
        //        {
        //            //slave
        //            depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
        //            std::cout << "(Rs2Device) Camera " << getPositionType() << " #" << serial << " set to SLAVE (val="
        //                      << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
        //        }
        //#endif
        //        m_rs2_pipe = rs2::pipeline();
        //        // rs2::pipeline rs2_pipe;
        //        rs2::config rs2_cfg;
        //        rs2_cfg.enable_device(serial);
        //        rs2_cfg.enable_stream(RS2_STREAM_DEPTH, RS_FRAME_WIDTH, RS_FRAME_HEIGHT, RS2_FORMAT_Z16, RS_FRAME_RATE);
        //        rs2_cfg.disable_stream(RS2_STREAM_COLOR);
        //        rs2::pipeline_profile rs2_profile = m_rs2_pipe.start(rs2_cfg, depth_callback);

    }
    else {
        std::cout << "(Rs2Device) Disabling capture of " << getPositionTypeStr() << " #" << serial << std::endl;
        m_capture_running = false;
        m_capture_thread.join();
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



    std::cout << std::endl << "DEBUG setCaptureEnabled end " << getPositionTypeStr() << std::endl << std::endl;
}

bool Rs2Device::isActive() { return m_active; }

Rs2Device::Rs2Device(rs2::device &dev, size_t dev_id, Rs2Position_t pos_id, rs2::frame_queue &framebuf)
{
    m_rs2_dev = dev;
    m_rs2_dev_id = dev_id;
    m_pos_id = pos_id;
    m_frame_queue = framebuf;
    std::cout << "New Realsense device, type: "<< getPositionTypeStr() << " #" << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    // m_ref_RS_to_interface = pos_id;
    // print_device_information(m_rs2_dev);



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

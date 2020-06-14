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
        depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
    }
    else if (color_found)
    {
        color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        color_sensor.set_option(RS2_OPTION_EXPOSURE, 100); // microseconds
        color_sensor.set_option(RS2_OPTION_GAIN, 64);
        color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
        color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
    }
    else {
        std::cerr << "(Rs2Device) No depth sensor found #" << serial << std::endl;
        return;
    }
#if (RS_MASTER_SLAVE_CONF_ENABLED == 1)
    // RGB sync doesn't work, need to use depth as master.
    if (m_rs2_dev_id == 0)
        m_sync_type = SyncType_t::MASTER;
    else
        m_sync_type = SyncType_t::SLAVE;
#else
    m_sync_type = SyncType_t::DEFAULT;
#endif
    depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, m_sync_type);
    std::cout << "(Rs2Device) Camera " << getCamTypeStr() << " #" << serial << " set to SyncMode " << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << std::endl;

    rs2::pipeline rs2_pipe;
    rs2::config rs2_cfg;
    rs2_cfg.enable_device(serial);
    rs2_cfg.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH_DEPTH, FRAME_HEIGHT_DEPTH, RS2_FORMAT_Z16, FRAME_RATE_DEPTH);
#if RS_COLOR_ENABLED
    rs2_cfg.enable_stream(RS2_STREAM_COLOR);
#else
    rs2_cfg.disable_stream(RS2_STREAM_COLOR);
#endif
    rs2::pipeline_profile rs2_profile = rs2_pipe.start(rs2_cfg);

    for (auto p : rs2_profile.get_streams())
    {
        rs2::video_stream_profile vsp = p.as<rs2::video_stream_profile>();
        auto intr = vsp.get_intrinsics();
        float fov[2]; // X, Y fov
        rs2_fov(&intr, fov);

        camera_intrinsics_t cam_params(intr.fx, intr.fy, intr.ppx, intr.ppy);
        cam_params.k1 = intr.coeffs[0]; cam_params.k2 = intr.coeffs[1];
        cam_params.p1 = intr.coeffs[2]; cam_params.p2 = intr.coeffs[3];
        cam_params.k3 = intr.coeffs[4];
        cam_params.fov_x = fov[0]; cam_params.fov_y = fov[1];

        if (vsp.stream_name() == "Depth")
            m_depth_frame_queue->setIntrinsics(cam_params);
        else if (vsp.stream_name() == "Color")
            m_color_frame_queue->setIntrinsics(cam_params);
#if (VERBOSE > 1)
            std::cout << getCamTypeStr() << " " << vsp.stream_name() << ": " << vsp.width() << "x" << vsp.height() << "  fps: " << vsp.fps()
                      << "  format: " << vsp.format() << "  principal point: cx=" << intr.ppx << " cy=" << intr.ppy << "  focal length: fx="
                      << intr.fx << " fy=" << intr.fy << "  distortion model: " << intr.model << "  distortion matrix: [" << intr.coeffs[0]
                      << " " << intr.coeffs[1] << " " << intr.coeffs[2] << " " << intr.coeffs[3] << " " << intr.coeffs[4]
                      << "]  FOV: x=" << fov[0] << "° y=" << fov[1] << "°" << std::endl << std::endl;
#endif
    }


    while (m_active) {
        rs2::frameset frameset = rs2_pipe.wait_for_frames();
        //  rs2::frameset fs = frame.as<rs2::frameset>()

        auto cap_start = std::chrono::high_resolution_clock::now();

        //        rs2::depth_frame depth_frame = frameset.get_depth_frame();
        //#if RS_COLOR_ENABLED
        //        rs2::frame color_frame = frameset.get_color_frame();
        //#endif

#if RS_DEPTH_ENABLED
        rs2_metadata_type depth_ts = 0;
        auto depth_frame = frameset.get_depth_frame();
        if (depth_frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
            depth_ts = depth_frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
        double drift = depth_ts - m_last_frame_time;
        m_last_frame_time = depth_ts;
#endif
#if RS_COLOR_ENABLED
        rs2_metadata_type color_ts = 0;
        auto color_frame = frameset.get_color_frame();
        if (color_frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
            color_ts = color_frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
#endif
        /* RS2_FRAME_METADATA_FRAME_TIMESTAMP
         * RS2_FRAME_METADATA_SENSOR_TIMESTAMP
         * RS2_FRAME_METADATA_TIME_OF_ARRIVAL
         * RS2_FRAME_METADATA_BACKEND_TIMESTAMP */

        if (m_recording)
        {

#if FILTER_DEPTH_RS_ENABLED && RS_DEPTH_ENABLED
#if RS_FILTER_DECIMATION_ENABLED
            depth_frame = m_dec_filter.process(depth_frame);
#endif
#if RS_FILTER_THRESHOLD_ENABLED
            depth_frame = m_thr_filter.process(depth_frame);
#endif
#if RS_FILTER_HOLEFILL_ENABLED
            depth_frame = m_hole_filter.process(depth_frame);
#endif
#if RS_FILTER_SPATIAL_ENABLED
            depth_frame = m_spat_filter.process(depth_frame);
#endif
#endif

#if RS_DEPTH_ENABLED
            depth_frame.keep();
            m_depth_frame_queue->addFrame(depth_frame);
#endif
#if RS_COLOR_ENABLED
            color_frame.keep();
            m_color_frame_queue->addFrame(color_frame);
#endif
#if VERBOSE
            auto cap_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-cap_start).count();
            if (cap_end >= drift)
                std::cerr << "(Rs2Device) Processing takes longer than capturing" << std::endl;
#endif
#if (VERBOSE > 1)
#if RS_COLOR_ENABLED
            std::cout << "(Rs2Device) Color frame from cam " << getCamTypeStr() << " (" << color_frame.get_frame_number() << "): "
                      << color_frame.get_width() << "x" << color_frame.get_height() << " ts: " << color_ts << std::endl;
#endif
#if RS_DEPTH_ENABLED
            std::cout << "(Rs2Device) Depth frame from cam " << getCamTypeStr() << " (" << depth_frame.get_frame_number() << "): "
                      << depth_frame.get_width() << "x" << depth_frame.get_height() << " ts: " << depth_ts << std::endl;
#endif
            std::cout << "(Rs2Device) " << getCamTypeStr() << " capture took " << cap_end << " ms" << std::endl;
            std::setprecision(2);
#endif

        }
    }
    rs2_pipe.stop();
    std::cout << "(Rs2Device) Capture thread stopped: #" << serial << " threadID: " << std::this_thread::get_id() << std::endl;
}

void Rs2Device::setCaptureEnabled(bool running)
{    
    std::string serial = m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (running && !m_capture_thread.joinable())
    {
        std::cout << "(Rs2Device) Enabling capture of " << getCamTypeStr() << " #" << serial << std::endl;
        m_active = true;
        m_capture_thread = std::thread(&Rs2Device::rs2_capture_thread_func, this);
    }
    else {
        std::cout << "(Rs2Device) Disabling capture of " << getCamTypeStr() << " #" << serial << std::endl;
        m_active = false;
        m_capture_thread.join();
    }
}

bool Rs2Device::isActive() { return m_active; }


Rs2Device::Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, FrameQueue* depth_frames)
{
    m_rs2_dev = dev;
    m_rs2_dev_id = dev_id;
    m_pos_id = pos_id;
    m_depth_frame_queue = depth_frames;
    std::cout << "New Realsense device, type: "<< getCamTypeStr() << " #" << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

Rs2Device::Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, FrameQueue* depth_frames, FrameQueue* color_frames)
{
    m_rs2_dev = dev;
    m_rs2_dev_id = dev_id;
    m_pos_id = pos_id;
    m_depth_frame_queue = depth_frames;
    m_color_frame_queue = color_frames;
    std::cout << "New Realsense device, type: "<< getCamTypeStr() << " #" << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

Rs2Device::~Rs2Device()
{
    std::cout << "(Rs2Device) Deleting rs2 camera #"
              << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

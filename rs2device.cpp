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
#if (RS_MASTER_SLAVE_CONFIG == 1)
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
#else
    depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 0);
    std::cout << "(Rs2Device) Camera " << getPositionTypeStr() << " #" << serial << " set to DEFAULT (val="
              << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE) << ")" << std::endl;
#endif


    rs2::pipeline rs2_pipe;
    rs2::config rs2_cfg;
    rs2_cfg.enable_device(serial);
    rs2_cfg.enable_stream(RS2_STREAM_DEPTH, RS_FRAME_WIDTH, RS_FRAME_HEIGHT, RS2_FORMAT_Z16, RS_FRAME_RATE);
    rs2_cfg.disable_stream(RS2_STREAM_COLOR);

    rs2::pipeline_profile rs2_profile = rs2_pipe.start(rs2_cfg);

#if (VERBOSE > 1)
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

    while (m_active) {
        rs2::frameset frames = rs2_pipe.wait_for_frames();
#if (VERBOSE > 0)
        auto cap_start = std::chrono::high_resolution_clock::now();
#endif
        rs2::frame depth_frame = frames.get_depth_frame();
        rs2_metadata_type sensor_ts = 0;

        if (depth_frame.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
        {
            sensor_ts = depth_frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        }

        double drift = sensor_ts - m_last_frame_time;
        m_last_frame_time = sensor_ts;

        if (m_recording)
        {
#if (RS_FILTER_FRAMES == 1)
            m_dec_filter.process(depth_frame);
            m_thr_filter.process(depth_frame);
            m_spat_filter.process(depth_frame);
#endif
            m_frame_queue.enqueue(depth_frame);
#if (VERBOSE > 0)
            std::cout << "(Rs2Device) " << frames.size() << " frame(s) from #" << serial << ": ";
            std::cout.precision(std::numeric_limits<double>::max_digits10);
            std::cout << "Drift: " << drift << ", ";
            switch(depth_frame.get_frame_timestamp_domain()) {
            case (RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK):
                // Frame timestamp was measured in relation to the camera clock
                std::cout << "Hardware Clock";
                break;
            case (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME):
                // Frame timestamp was measured in relation to the OS system clock
                std::cout << "System Time";
                break;
            case (RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME):
                // Frame timestamp was measured in relation to the camera clock and converted to OS system clock by constantly measure the difference
                std::cout << "Global Time";
                break;
            case (RS2_TIMESTAMP_DOMAIN_COUNT):
                // Number of enumeration values. Not a valid input: intended to be used in for-loops.
                std::cout << "Domain Count";
                break;
            default:
                std::cout << "Unknown";
                break;
            }
            std::cout << " TS: " << depth_frame.get_timestamp() << "  Sensor TS: " << sensor_ts << " (" << depth_frame.get_frame_number() << "), size: "
                      << depth_frame.get_data_size() << " bytes" << std::endl;

            //            const rs2_frame* frame_handle = depth_frame.get();
            //            if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_FRAME_TIMESTAMP, nullptr) ) // device ts at data readout/transmission
            //            {
            //                rs2_metadata_type frame_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_FRAME_TIMESTAMP, nullptr);
            //                std::cout << "DEBUG RS2_FRAME_METADATA_FRAME_TIMESTAMP (usec) " << frame_ts_meta << std::endl;
            //            }
            //            if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, nullptr) ) // device ts at middle of exposure
            //            {
            //                rs2_metadata_type sensor_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, nullptr);
            //                std::cout << "DEBUG RS2_FRAME_METADATA_SENSOR_TIMESTAMP (usec) " << sensor_ts_meta << std::endl;
            //            }
            //            if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, nullptr) ) // system ts
            //            {
            //                rs2_metadata_type arrival_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, nullptr);
            //                std::cout << "DEBUG RS2_FRAME_METADATA_TIME_OF_ARRIVAL " << arrival_ts_meta << std::endl;
            //            }
            //            if ( rs2_supports_frame_metadata(frame_handle, RS2_FRAME_METADATA_BACKEND_TIMESTAMP, nullptr) ) // ts from uvc
            //            {
            //                rs2_metadata_type backend_ts_meta = rs2_get_frame_metadata(frame_handle, RS2_FRAME_METADATA_BACKEND_TIMESTAMP, nullptr);
            //                std::cout << "DEBUG RS2_FRAME_METADATA_BACKEND_TIMESTAMP (usec) " << backend_ts_meta << std::endl;
            //            }

            auto cap_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-cap_start).count();
            if (cap_end >= drift)
                std::cerr << "(Rs2Device) Processing takes longer than capturing" << std::endl;
#endif
#if (VERBOSE > 1)
            std::cout << "(Rs2Device) " << getPositionTypeStr() << " capture took " << cap_end << " ms" << std::endl;
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
        std::cout << "(Rs2Device) Enabling capture of " << getPositionTypeStr() << " #" << serial << std::endl;
        m_active = true;
        m_capture_thread = std::thread(&Rs2Device::rs2_capture_thread_func, this);
    }
    else {
        std::cout << "(Rs2Device) Disabling capture of " << getPositionTypeStr() << " #" << serial << std::endl;
        m_active = false;
        m_capture_thread.join();
    }
}

bool Rs2Device::isActive() { return m_active; }

Rs2Device::Rs2Device(rs2::device &dev, size_t dev_id, CameraType_t pos_id, rs2::frame_queue &framebuf)
{
    m_rs2_dev = dev;
    m_rs2_dev_id = dev_id;
    m_pos_id = pos_id;
    m_frame_queue = framebuf;
    std::cout << "New Realsense device, type: "<< getPositionTypeStr() << " #" << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

Rs2Device::~Rs2Device()
{
    std::cout << "(Rs2Device) Deleting rs2 camera #"
              << m_rs2_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

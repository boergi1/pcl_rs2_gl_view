#include <iostream>
#include "deviceinterface.h"
#include "rs2_pcl_converter.h"
#include "pclinterface.h"
#include "glgraphics.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

int main() try
{
    std::cout << "Main thread started # " << std::this_thread::get_id() << std::endl;

    DeviceInterface* device_interface = new DeviceInterface;
    PclInterface* pcl_interface = nullptr;
    Rs2_PCL_Converter* rs2_pcl_conv = nullptr;

    auto rs2_device_types = device_interface->connectRealSenseDevices();
    if (rs2_device_types.size() > 0)
    {
        std::cout << "RealSense devices: " << rs2_device_types.size() << std::endl;
#if (RS_DEPTH_ENABLED > 0)
        pcl_interface = new PclInterface(rs2_device_types);
        rs2_pcl_conv = new Rs2_PCL_Converter(device_interface, pcl_interface, rs2_device_types);
        rs2_pcl_conv->init(); // start multiple workerthreads
#endif
        device_interface->startRecordingRs2Devices();
#if (RS_DEPTH_ENABLED > 0)
        rs2_pcl_conv->setActive(true);
        pcl_interface->setActive(true);
#endif
    }
    else std::cerr << "No Realsense device found" << std::endl;


    // device_interface->connectVideoDevice(2);


    std::map<std::string, rs2::colorizer> colorizers;
    for (auto camtype : rs2_device_types)
    {
        switch (camtype) {
        case CameraType_t::CENTRAL:
            colorizers[RS_CENTRAL_SERIAL] = rs2::colorizer();
            break;
        case CameraType_t::FRONT:
            colorizers[RS_FRONT_SERIAL] = rs2::colorizer();
            break;
        case CameraType_t::REAR:
            colorizers[RS_REAR_SERIAL] = rs2::colorizer();
            break;
        default: break;
        }
    }

    std::map<int, rs2::frame> render_frames;

    window app(1280, 960, "CPP Multi-Camera Example");

    while (app) {
        //  std::cout << "Main thread" << std::endl;
#if (VERBOSE > 0)
        auto draw_start = std::chrono::high_resolution_clock::now();
#endif

        std::vector<rs2::frame> new_frames;

#if (RS_DEPTH_ENABLED > 0)
        auto depth_data = device_interface->getDepthFrameData();
        for (FrameQueue* queue_d : *depth_data)
            if ( !queue_d->isEmpty() ) new_frames.emplace_back(queue_d->readFrame());
#endif
#if (RS_COLOR_ENABLED > 0)
        auto color_data = device_interface->getColorFrameData();
        for (FrameQueue* queue_c : *color_data)
            if ( !queue_c->isEmpty() ) new_frames.emplace_back(queue_c->getFrame());
#endif

        // Convert the newly-arrived frames to render-friendly format
        for (const auto& frame : new_frames)
        {
            // Get the serial number of the current frame's device
            auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            // Apply the colorizer of the matching device and store the colorized frame
            render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
        }

        // Present all the collected frames with openGl mosaic
        app.show(render_frames);
#if (VERBOSE > 0)
        auto draw_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-draw_start).count();
        std::cout << "Main thread took " << draw_end << " ms" << std::endl;
#endif
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));

    }

    delete pcl_interface;
    delete rs2_pcl_conv;
    delete device_interface;

    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

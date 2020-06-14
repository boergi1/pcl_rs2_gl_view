#include <iostream>
#include "deviceinterface.h"
#include "rs2_pcl_converter.h"
#include "processinginterface.h"
#include "mainwindowgl.h"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

int main() try
{
    std::cout << "Main thread started # " << std::this_thread::get_id() << std::endl;

    MainWindowGL* MainWindow = new MainWindowGL;
    ;
    //        MainWindow->drawPointClouds();
    //    else
    //        MainWindow->exit();


    DeviceInterface* device_interface = new DeviceInterface;
    ProcessingInterface* pcl_interface = nullptr;
    Rs2_PCL_Converter* rs2_pcl_conv = nullptr;

    auto rs2_device_types = device_interface->connectRealSenseDevices();
    if (rs2_device_types.size() > 0)
    {
        std::cout << "RealSense devices: " << rs2_device_types.size() << std::endl;
#if (RS_DEPTH_ENABLED > 0)
        pcl_interface = new ProcessingInterface(rs2_device_types, MainWindow);
        rs2_pcl_conv = new Rs2_PCL_Converter(device_interface, pcl_interface, rs2_device_types);
        rs2_pcl_conv->init(CONV_THREAD_POOL_SIZE); // start multiple workerthreads
#endif
        device_interface->startRecordingRs2Devices();
#if (RS_DEPTH_ENABLED > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(FRAME_PERIOD_MS));
        rs2_pcl_conv->setActive(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(FRAME_PERIOD_MS));
        pcl_interface->setActive(true);
#endif
    }
    else std::cerr << "No Realsense device found" << std::endl;

    // device_interface->connectVideoDevice(2);

    if ( !MainWindow->initialize() )
    {
        std::cerr << "Could not initialize the Main Window." << std::endl;
        MainWindow->exit();
        return EXIT_FAILURE;
    }
    else MainWindow->drawPointClouds();

    // Window was closed, terminate.
    device_interface->disconnectRealSenseDevices();
    rs2_pcl_conv->setActive(false);
    pcl_interface->setActive(false);

    delete device_interface;
    delete rs2_pcl_conv;
    delete pcl_interface;
    delete MainWindow;

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

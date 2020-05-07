#include <iostream>
#include "deviceinterface.h"
#include "rs2_pcl_converter.h"
#include "pclinterface.h"


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
    PclInterface* pcl_interface;
    Rs2_PCL_Converter* rs2_pcl_conv;

    auto rs2_device_types = device_interface->connectRealSenseDevices();
    if (rs2_device_types.size() > 0)
    {
        std::cout << "RealSense devices: " << rs2_device_types.size() << std::endl;
        pcl_interface = new PclInterface(rs2_device_types);
        rs2_pcl_conv = new Rs2_PCL_Converter(device_interface, pcl_interface, rs2_device_types);
        rs2_pcl_conv->init(); // start multiple workerthreads

        device_interface->startRecordingRs2Devices();
        rs2_pcl_conv->setActive(true);
        pcl_interface->setActive(true);
    }
    else std::cerr << "No Realsense device found" << std::endl;

    // device_interface->connectVideoDevice(2);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Main thread" << std::endl;
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

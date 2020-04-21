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
    DeviceInterface* device_interface;
    PclInterface* pcl_interface;
    Rs2_PCL_Converter* rs2_pcl_conv;

    device_interface = new DeviceInterface;
    size_t rs2_device_count = device_interface->connectRealSenseDevices();
    if (rs2_device_count > 0)
    {
        pcl_interface = new PclInterface(rs2_device_count);
        rs2_pcl_conv = new Rs2_PCL_Converter(device_interface, pcl_interface, rs2_device_count);
        rs2_pcl_conv->startThread();
        pcl_interface->startThread();
    }
    else std::cerr << "No Realsense device found" << std::endl;

    // device_interface->connectVideoDevice(2);

//    std::this_thread::sleep_for(std::chrono::seconds(2));
//    delete device_interface;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Main thread" << std::endl;


    }

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

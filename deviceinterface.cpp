#include "deviceinterface.h"

//DeviceInterface::DeviceInterface()
//{

//}

DeviceInterface::DeviceInterface()
{
    std::cout << "Created instance of DeviceInterface" << std::endl;
    //  connectRealSenseDevices();
}

size_t DeviceInterface::connectRealSenseDevices()
{
    m_RS_data.clear();
    //  m_rs2_dev_mtxs.clear();
    m_rs2_devices.clear();
    //  m_rs2_points_buffers.clear();
    //  m_pcl_clouds_buffers.clear();
    //   m_points_write_indexes.clear();
    //   m_points_read_indexes.clear();
    size_t device_count = 0;
    //        m_deviceNames.clear();
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;
    // Using the context we can get all connected devices in a device list
    m_rs2_device_list = ctx.query_devices();

    if (m_rs2_device_list.size() == 0)
    {
        if ((false))
        {
            std::cout << "No device connected, please connect a RealSense device" << std::endl;
            rs2::device_hub device_hub(ctx);
            device_hub.wait_for_device(); // blocking until device connects
            std::cout << "Device connected" << std::endl;
            connectRealSenseDevices();
        }
        else return 0;
    }
    else
    {
        device_count = m_rs2_device_list.size();
        std::cout << "Creating " << device_count << " instances of Rs2Device" << std::endl;
        // Creating pipelines
        m_rs2_pipelines.reserve(device_count);
        for  (uint32_t i=0; i < device_count; i++) {
            m_rs2_pipelines.push_back(rs2::pipeline());
        }

        m_rs2_devices.reserve(device_count);

        // Creating threaded devices for acquisition
        //   m_rs2_dev_mtxs.reserve(device_count);
        m_RS_data.resize(device_count);


        //  m_rs2_points_buffers.reserve(device_count);
        //  m_pcl_clouds_buffers.reserve(device_count);
        //  m_points_write_indexes.resize(device_count, 0);
        //  m_points_read_indexes.resize(device_count, 0);

        for (uint32_t i=0; i < device_count; i++) {
            rs2::device rs2_device =  m_rs2_device_list[i];

            m_RS_data.at(i).buf_ref = new rs2::points[BUF_SIZE_POINTS];
            m_RS_data.at(i).mtx_ref = new std::mutex();
            m_RS_data.at(i).w_idx_ref = new size_t(0);
            m_RS_data.at(i).r_idx_ref = new size_t(0);
            m_rs2_devices.push_back(new Rs2Device(rs2_device, m_RS_data.at(i) ));

            //  m_rs2_points_buffers[i] = new rs2::points[BUF_SIZE_POINTS];
            //  m_pcl_clouds_buffers[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr();
            // m_rs2_dev_mtxs[i] = new std::mutex();
            //  m_rs2_write_indexes[i] = new size_t(0);
            //            m_rs2_write_indexes.push_back(0);
            //  m_rs2_write_indexes.at(i) = 0;

            //            m_rs2_devices.push_back(new Rs2Device(rs2_device, m_rs2_dev_mtxs[i], m_rs2_points_buffers[i],
            //                                                  m_points_write_indexes[i] ));
        }

        std::cout << "connectRealSenseDevices end" << std::endl;

    }

    return device_count;
}

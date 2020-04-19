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
    rs2::context ctx;
    rs2::device_list rs2_device_list = ctx.query_devices();
    size_t device_count = rs2_device_list.size();

    if (device_count == 0)
        return 0;
    else
    {
        // Creating threaded devices for acquisition
        m_rs2_devices.resize(device_count);
        m_RS_data.resize(device_count);

        for (uint32_t i=0; i < device_count; i++) {
            rs2::device rs2_device =  rs2_device_list[i];
            std::string serial_num = getRs2DeviceSerialNum(rs2_device);

            size_t id;

            if (serial_num == RS0_CENTRAL_SERIAL)
            {
                id = 0;
                std::cout << "Creating CENTRAL instance of Rs2Device at idx " << id << std::endl;
            }
            else if (serial_num == RS1_FRONT_SERIAL)
            {
                id = 1;
                std::cout << "Creating FRONT instances of Rs2Device at idx " << id << std::endl;
            }
            else if (serial_num == RS2_REAR_SERIAL)
            {
                id = 2;
                std::cout << "Creating REAR instances of Rs2Device at idx " << id << std::endl;
            }
            else
            {
                std::cerr << "(DeviceInterface) No matching RS2 serial number" << std::endl;
                continue;
            }



            m_RS_data.at(id).buf_ref = new rs2::points[BUF_SIZE_POINTS];
            m_RS_data.at(id).mtx_ref = new std::mutex();
            m_RS_data.at(id).w_idx_ref = new size_t(0);
            m_RS_data.at(id).r_idx_ref = new size_t(0);
            m_rs2_devices.at(id) = new Rs2Device( rs2_device, m_RS_data.at(id) );


            //  m_rs2_points_buffers[i] = new rs2::points[BUF_SIZE_POINTS];
            //  m_pcl_clouds_buffers[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr();
            // m_rs2_dev_mtxs[i] = new std::mutex();
            //  m_rs2_write_indexes[i] = new size_t(0);
            //            m_rs2_write_indexes.push_back(0);
            //  m_rs2_write_indexes.at(i) = 0;

            //            m_rs2_devices.push_back(new Rs2Device(rs2_device, m_rs2_dev_mtxs[i], m_rs2_points_buffers[i],
            //                                                  m_points_write_indexes[i] ));
        }

    }

    return device_count;
}

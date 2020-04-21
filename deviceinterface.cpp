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
                id = 0;
            else if (serial_num == RS1_FRONT_SERIAL)
                id = 1;
            else if (serial_num == RS2_REAR_SERIAL)
                id = 2;
            else
            {
                std::cerr << "(DeviceInterface) No matching RS2 serial number" << std::endl;
                continue;
            }

#if (VERBOSE > 1)
            switch (id) {
            case 0: {
                std::cout << "Creating CENTRAL instance of Rs2Device at idx " << id << std::endl; break;
            }
            case 1: {
                std::cout << "Creating FRONT instances of Rs2Device at idx " << id << std::endl; break;
            }
            case 2: {
                std::cout << "Creating REAR instances of Rs2Device at idx " << id << std::endl; break;
            }
            }
#endif
            m_RS_data.at(id).buf_ref = new rs2::points[BUF_SIZE_POINTS];
            m_RS_data.at(id).mtx_ref = new std::mutex();
            m_RS_data.at(id).w_idx_ref = new size_t(0);
            m_RS_data.at(id).r_idx_ref = new size_t(0);
            m_rs2_devices.at(id) = new Rs2Device( rs2_device, m_RS_data.at(id) );
        }

        // Start acquisition and check for success
        std::vector<Rs2Device*>::iterator iter = m_rs2_devices.begin();
        while( iter != m_rs2_devices.end() )
        {
            if ((*iter)->setCaptureEnabled(true) < 0 )
            {
                size_t idx = static_cast<size_t>( iter - m_rs2_devices.begin() );
                delete[] static_cast<rs2::points*>( m_RS_data.at(idx).buf_ref );
                delete m_RS_data.at(idx).mtx_ref;
                delete m_RS_data.at(idx).w_idx_ref;
                delete m_RS_data.at(idx).r_idx_ref;
                m_RS_data.erase(m_RS_data.begin() + static_cast<long>(idx));

                delete m_rs2_devices.at(idx);
                m_rs2_devices.at(idx) = nullptr;
                iter = m_rs2_devices.erase(iter);
                device_count--;
            }
            else ++iter;
        }
    }

    return device_count;
}

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
    size_t device_id = 0;

    for (auto&& dev : m_ctx.query_devices())
    {
        Rs2Position_t pos_id;
        std::string serial_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        if (serial_num == RS0_CENTRAL_SERIAL)
            pos_id = Rs2Position_t::CENTRAL;
        else if (serial_num == RS1_FRONT_SERIAL)
            pos_id = Rs2Position_t::FRONT;
        else if (serial_num == RS2_REAR_SERIAL)
            pos_id = Rs2Position_t::REAR;
        else
        {
            std::cerr << "(DeviceInterface) No matching RS2 serial number: " << serial_num << std::endl;
            pos_id = Rs2Position_t::OTHER;
            //continue;
        }
        
        m_depth_frames.push_back(rs2::frame_queue(BUF_SIZE_RS2FRAMES, true));

        m_rs2_devices.push_back(new Rs2Device( dev, device_id, pos_id, m_depth_frames.back()));
        

//        m_RS_data.push_back(rs2_references_t(new rs2::frame[BUF_SIZE_RS2FRAMES], new std::mutex(),
//                                                new size_t(0), new size_t(0), pos_id));
//        m_rs2_devices.push_back(new Rs2Device( dev, device_id, m_RS_data.back()));

        m_rs2_devices.back()->setCaptureEnabled(true);

        device_id++;
        std::this_thread::sleep_for(std::chrono::seconds(5));

    }

    return device_id;

//    rs2::context ctx;
//    rs2::device_list rs2_device_list = ctx.query_devices();
//    size_t device_ct = rs2_device_list.size();
//    std::cerr << "(DeviceInterface) Found rs2 devices: " << device_id << std::endl;S
//    if (device_ct)
//    {
//        // Creating threaded devices for acquisition
//        //        m_rs2_devices.clear();
//        //        m_RS_data.clear();
//        for (uint32_t i=0; i < device_ct; i++) {
//            // use only registered devices
//            rs2::device rs2_device = rs2_device_list[i];
//            std::string serial_num = getRs2DeviceSerialNum(rs2_device);
//            Rs2Position_t id;
//            if (serial_num == RS0_CENTRAL_SERIAL)
//                id = Rs2Position_t::CENTRAL;
//            else if (serial_num == RS1_FRONT_SERIAL)
//                id = Rs2Position_t::FRONT;
//            else if (serial_num == RS2_REAR_SERIAL)
//                id = Rs2Position_t::REAR;
//            else
//            {
//                std::cerr << "(DeviceInterface) No matching RS2 serial number: " << serial_num << std::endl;
//                id = Rs2Position_t::OTHER;
//                //continue;
//            }
//            // test usb connection
//            rs2_camera_info cam_info_usb = static_cast<rs2_camera_info>(9);
//            if (rs2_device.supports(cam_info_usb))
//            {
//                double usb_type = std::atof( rs2_device.get_info(cam_info_usb) );
//                if (usb_type < 3.0)
//                {
//                    std::cerr << "(DeviceInterface) USB type below 3.0: " << serial_num << std::endl;
//                    continue;
//                }
//            }
//            else
//            {
//                std::cerr << "(DeviceInterface) Couldn't read USB type: " << serial_num << std::endl;
//                continue;
//            }
//#if (VERBOSE > 3)
//            switch (id) {
//            case Rs2Position_t::CENTRAL: {
//                std::cout << "(DeviceInterface) Creating CENTRAL instance of Rs2Device at idx "
//                          << m_rs2_devices.size() << std::endl; break;
//            }
//            case Rs2Position_t::FRONT: {
//                std::cout << "(DeviceInterface) Creating FRONT instance of Rs2Device at idx "
//                          << m_rs2_devices.size() << std::endl; break;
//            }
//            case Rs2Position_t::REAR: {
//                std::cout << "(DeviceInterface) Creating REAR instance of Rs2Device at idx "
//                          << m_rs2_devices.size() << std::endl; break;
//            }
//            default: break;
//            }
//#endif
////            m_RS_data.push_back(rs2_references_t(new rs2::frame[BUF_SIZE_RS2FRAMES], new std::mutex(),
////                                                    new size_t(0), new size_t(0), id));
////            m_rs2_devices.push_back(new Rs2Device( rs2_device, i, m_RS_data.back()));
//        }
//        for (size_t i = 0; i < m_rs2_devices.size(); i++) {
//            m_rs2_devices.at(i)->setCaptureEnabled(true);
//        }
//        std::vector<Rs2Device*>::iterator iter = m_rs2_devices.begin();
//        while( iter != m_rs2_devices.end() )
//        {
//            if ( ! (*iter)->isActive() )
//                device_ct--;
//            ++iter;
//        }
//    }
//    return device_ct;
}

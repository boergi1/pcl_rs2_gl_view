#include "deviceinterface.h"

DeviceInterface::DeviceInterface()
{
    std::cout << "Created instance of DeviceInterface" << std::endl;
}

std::vector<CameraType_t> DeviceInterface::connectRealSenseDevices()
{    
    size_t device_id = 0;
    std::vector<CameraType_t> devices;

    for (auto&& dev : m_ctx.query_devices())
    {
        CameraType_t pos_id;
        std::string serial_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        // test usb connection
        rs2_camera_info cam_info_usb = static_cast<rs2_camera_info>(9);
        if (dev.supports(cam_info_usb))
        {
            double usb_type = std::atof( dev.get_info(cam_info_usb) );
            if (usb_type < 3.0)
            {
                std::cerr << "(DeviceInterface) USB type below 3.0: " << serial_num << std::endl;
                continue;
            }
        }
        else
        {
            std::cerr << "(DeviceInterface) Couldn't read USB type: " << serial_num << std::endl;
            continue;
        }

        // get position by serial number
        if (serial_num == RS0_CENTRAL_SERIAL)
            pos_id = CameraType_t::CENTRAL;
        else if (serial_num == RS1_FRONT_SERIAL)
            pos_id = CameraType_t::FRONT;
        else if (serial_num == RS2_REAR_SERIAL)
            pos_id = CameraType_t::REAR;
        else
        {
            std::cerr << "(DeviceInterface) No matching RS2 serial number: " << serial_num << std::endl;
            // pos_id = CamPosition_t::OTHER;
            continue;
        }
        
        m_depth_frames.push_back(rs2::frame_queue(QUE_SIZE_RS2FRAMES, true));

        m_rs2_devices.push_back(new Rs2Device( dev, device_id, pos_id, m_depth_frames.back()));

        m_rs2_devices.back()->setCaptureEnabled(true);

        devices.push_back(pos_id);

        device_id++;
        std::this_thread::sleep_for(std::chrono::seconds(3));

    }

    std::vector<Rs2Device*>::iterator iter = m_rs2_devices.begin();
    while( iter != m_rs2_devices.end() )
    {
        if ( ! (*iter)->isActive() )
        {
            std::cerr << "(DeviceInterface) Rs2Device not active: " << (*iter)->getPositionTypeStr() << std::endl;
            device_id--;
            for (size_t i = 0; i < devices.size(); i++) {
                if (devices.at(i) == (*iter)->getPositionType())
                {
                    devices.erase(devices.begin() + i);
                    break;
                }
            }
        }
        ++iter;
    }

    return devices;

}

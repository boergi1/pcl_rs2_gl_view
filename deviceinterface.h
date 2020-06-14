#ifndef DEVICEINTERFACE_H
#define DEVICEINTERFACE_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>

//#include <opencv2/core/utility.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/tracking/tracker.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/common/common_headers.h>

#include <opencv2/core/core.hpp>

#include "rs2device.h"
#include "ocvdevice.h"
#include "customtypes.h"

class DeviceInterface
{
private:
    rs2::context m_ctx;
    std::vector<Rs2Device*> m_rs2_devices;

    std::vector<FrameQueue*> m_depth_frame_queues;
    std::vector<FrameQueue*> m_color_frame_queues;


    std::vector<OcvDevice*> m_opencv_devices;
    std::vector<std::mutex *> m_opencv_devices_dev_mtxs;
    std::vector<cv::Mat*> m_ocv_mat_buffers;
    std::vector<size_t> m_ocv_write_indexes;

public:
    DeviceInterface();

    ~DeviceInterface()
    {
        std::cout << "(DeviceInterface) Destructor " << std::endl;
        while (m_rs2_devices.size())
        {
            std::cout << "(DeviceInterface) Freeing Rs2Device memory " << m_rs2_devices.size() << std::endl;
            delete m_rs2_devices.back();
            m_rs2_devices.back() = nullptr;
            m_rs2_devices.pop_back();
        }
    }

    std::vector<Rs2Device *>* getRs2Devices() { return &m_rs2_devices; }

    int connectVideoDevice(int idx)
    {
        std::cout << "Connecting OpenCV device" << std::endl;
        m_opencv_devices_dev_mtxs.push_back(new std::mutex);
        m_ocv_mat_buffers.push_back(new cv::Mat[BUF_SIZE_MATS]);
        m_ocv_write_indexes.push_back(0);
        m_opencv_devices.push_back(new OcvDevice(idx, m_opencv_devices_dev_mtxs.back(),
                                                 m_ocv_mat_buffers.back(), m_ocv_write_indexes.back()));
        return static_cast<int>( m_opencv_devices.size() );
    }

    std::vector<CameraType_t> connectRealSenseDevices();

    void disconnectRealSenseDevices()
    {
        std::cout << "DEBUG disconnectRealSenseDevices " << m_rs2_devices.size() << std::endl;
        for(size_t i=0; i < m_rs2_devices.size(); i++)
        {
            m_rs2_devices.at(i)->setCaptureEnabled(false);
            if (m_rs2_devices.at(i)->isActive()) std::cerr << "Error: Rs2 Device not disabled: " << i << std::endl;
        }
    }

    void startRecordingRs2Devices()
    {
        std::cout << std::endl << "(DeviceInterface) Recording start" << std::endl << std::endl;
        for(auto device : m_rs2_devices)
        {
            device->setRecordingEnabled(true);
        }
    }

    void stopRecordingRs2Devices()
    {
        std::cout << std::endl << "(DeviceInterface) Recording stop" << std::endl << std::endl;
        for(auto device : m_rs2_devices)
        {
            device->setRecordingEnabled(false);
        }
    }

    std::string getRs2DeviceSerialNum(const rs2::device &dev)
    {
        std::string sn = "Unknown ID";
        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
            sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        return sn;
    }


    std::vector<FrameQueue*>* getDepthFrameData()
    {
        return &m_depth_frame_queues;
    }
    std::vector<FrameQueue*>* getColorFrameData()
    {
        return &m_color_frame_queues;
    }



};

#endif // DEVICEINTERFACE_H

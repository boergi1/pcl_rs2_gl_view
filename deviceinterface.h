#ifndef DEVICEINTERFACE_H
#define DEVICEINTERFACE_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>

#include <pcl/common/common_headers.h>

#include <opencv2/core/core.hpp>

#include "rs2device.h"
#include "ocvdevice.h"
#include "customtypes.h"


class DeviceInterface
{
private:

    rs2::context m_ctx;

    std::vector<Rs2Device *> m_rs2_devices;

    //  std::vector<rs2_references_t> m_RS_data;

    std::vector<rs2::frame_queue> m_depth_frames;



    // std::vector<rs2::sensor> m_sensors;
    // rs2::sensor m_sensors;
    //   std::vector< std::vector<rs2::sensor> > m_sensors; // -> to std::map

    //  QList<std::string> m_deviceNames;
    // std::vector<rs2::points*> m_rs2_points_buffers;
    //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_pcl_clouds_buffers;


    std::vector<OcvDevice*> m_opencv_devices;
    std::vector<std::mutex *> m_opencv_devices_dev_mtxs;
    std::vector<cv::Mat*> m_ocv_mat_buffers;
    std::vector<size_t> m_ocv_write_indexes;




public:
    DeviceInterface();

    //    std::mutex getPointsBufferMutex(size_t device_index) {
    //        //  return  m_rs2_devices.at(device_index)->rs2points_buf_mtx;
    //    }

    ~DeviceInterface()
    {
        std::cout << "(DeviceInterface) Destructor " << std::endl;
        disconnectRealSenseDevices();
    }


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


    size_t connectRealSenseDevices();

    void disconnectRealSenseDevices()
    {
        while (m_rs2_devices.size())
        {
            std::cout << "(DeviceInterface) Freeing Rs2Device memory " << m_rs2_devices.size() << std::endl;
            m_rs2_devices.back()->setCaptureEnabled(false);
            if (m_rs2_devices.back()->isActive()) std::cerr << "Still active" << std::endl;
            delete m_rs2_devices.back();
            m_rs2_devices.pop_back();
        }

        //        while (m_RS_data.size()) {
        //            std::cout << "(DeviceInterface) Freeing memory of shared references: " << m_RS_data.size() << std::endl;
        //            delete[] static_cast<rs2::points*>( m_RS_data.back().buf_ref );
        //            delete m_RS_data.back().mtx_ref;
        //            delete m_RS_data.back().w_idx_ref;
        //            delete m_RS_data.back().r_idx_ref;
        //            m_RS_data.back().buf_ref = nullptr;
        //            m_RS_data.back().mtx_ref = nullptr;
        //            m_RS_data.back().w_idx_ref = nullptr;
        //            m_RS_data.back().r_idx_ref = nullptr;
        //            m_RS_data.pop_back();
        //        }
    }


    std::string getRs2DeviceSerialNum(const rs2::device &dev)
    {
        std::string sn = "Unknown ID";
        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
            sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        return sn;
    }

    std::vector<rs2::frame_queue>* getDepthFrameData()
    {
        return &m_depth_frames;
    }

    //    std::vector<rs2_references_t> get_rs_data_refs()
    //    {
    //        return m_RS_data;
    //    }



};

#endif // DEVICEINTERFACE_H

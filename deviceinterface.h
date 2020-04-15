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

    std::vector<Rs2Device *> m_rs2_devices;
    std::vector<rs2::pipeline> m_rs2_pipelines;


    //  std::vector<std::mutex *> m_rs2_dev_mtxs;
    //  std::vector<size_t> m_points_write_indexes;
    //  std::vector<size_t> m_points_read_indexes;

    std::vector<shared_references_t> m_RS_data;


    rs2::device_list m_rs2_device_list;
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

    std::vector<shared_references_t> get_rs_data_refs()
    {
        return m_RS_data;
    }

    //    std::mutex* getPointsBufferMutex(size_t i)
    //    {
    //        return m_rs2_dev_mtxs[i];
    //    }

    //    rs2::points* getPointsBufferRef(size_t i)
    //    {
    //        return m_rs2_points_buffers[i];
    //    }

    //    size_t& getPointsWriteIndexRef(size_t i)
    //    {
    //        return  m_points_write_indexes[i];
    //    }
    //    size_t& getPointsReadIndexRef(size_t i)
    //    {
    //        return  m_points_read_indexes[i];
    //    }

};

#endif // DEVICEINTERFACE_H

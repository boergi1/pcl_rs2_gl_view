#ifndef DEVICEINTERFACE_H
#define DEVICEINTERFACE_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>
#include "rs2device.h"

#include <pcl/common/common_headers.h>




class DeviceInterface
{
private:

    std::vector<Rs2Device *> m_rs2_devices;
    std::vector<rs2::pipeline> m_rs2_pipelines;

    //  std::vector<std::mutex> m_rs2_dev_mtxs;
    std::vector<std::mutex *> m_rs2_dev_mtxs;
    std::vector<size_t> m_points_write_indexes;
     std::vector<size_t> m_points_read_indexes;


    rs2::device_list m_rs2_device_list;
    // std::vector<rs2::sensor> m_sensors;
    // rs2::sensor m_sensors;
    //   std::vector< std::vector<rs2::sensor> > m_sensors; // -> to std::map

    //  QList<std::string> m_deviceNames;
    std::vector<rs2::points*> m_rs2_points_buffers;
  //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_pcl_clouds_buffers;


public:
    DeviceInterface();

//    std::mutex getPointsBufferMutex(size_t device_index) {
//        //  return  m_rs2_devices.at(device_index)->rs2points_buf_mtx;
//    }


    size_t connectRealSenseDevices();

    std::mutex* getPointsBufferMutex(size_t i)
    {
        return m_rs2_dev_mtxs[i];
    }

    rs2::points* getPointsBufferRef(size_t i)
    {
        return m_rs2_points_buffers[i];
    }

    size_t& getPointsWriteIndexRef(size_t i)
    {
        return  m_points_write_indexes[i];
    }
    size_t& getPointsReadIndexRef(size_t i)
    {
        return  m_points_read_indexes[i];
    }

};

#endif // DEVICEINTERFACE_H

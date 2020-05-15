#include "rs2_pcl_converter.h"

void Rs2_PCL_Converter::converter_thread_func()
{
    std::cout << "(Converter) Thread started, id: " << std::this_thread::get_id() << std::endl;

    //    m_active = true;
    while (m_active) {
        bool idle = true;
        // Read from RealSense2 Devices and generate tasks
        for (size_t i = 0; i < m_ref_to_depth_queues->size(); i++)
        {
            //            auto &queue = m_ref_to_rs2_queues->at(i);
            auto queue_depth = m_ref_to_depth_queues->at(i);
#if RS_COLOR_ENABLED
            auto queue_color = m_ref_to_color_queues->at(i);
#endif
            while ( !queue_depth->isEmpty() )
            {
                idle = false;
                FrameToPointsTask* task_f2p = new FrameToPointsTask();
                task_f2p->setTaskType(TaskType_t::TSKTYPE_F2P);
                task_f2p->setTaskId(m_cam_positions.at(i));
                task_f2p->setTaskStatus(BaseTask::WORK_TO_DO);
                task_f2p->in.push_back(queue_depth->getFrame());
#if RS_COLOR_ENABLED
                task_f2p->in.push_back(queue_color->getFrame());
#endif
                this->addTask(task_f2p);
#if (VERBOSE > 1)
                std::cout << "(Converter) Added TASK \"F2P\" (" << tmp_rs2_frame.get_frame_number() << ") size in: "
                          << task_f2p->in.size() << "x" << tmp_rs2_frame.get_data_size() << " addr: " << &task_f2p << std::endl;
#endif
            }
        }

        // Read tasks from output
        // std::cout << "(Converter) Handle " << this->OutputQueue.size() << " finished tasks" << std::endl;
        while(this->OutputQueue.size())
        {
            idle = false;
            BaseTask* tmp_task = getTask();
            auto taskid = tmp_task->getTaskId();
            if (tmp_task->getTaskStatus() == BaseTask::TaskStatus_t::TASK_DONE)
            {
                switch (tmp_task->getTaskType()) {
                // Read converted points and create new tasks
                case TaskType_t::TSKTYPE_F2P:
                {
                    switch (taskid) {
                    case CameraType_t::CENTRAL:
                    {
                        for (auto tuple : static_cast<FrameToPointsTask*>(tmp_task)->out)
                        {
                            PointsToCloudTask* task_p2c = new PointsToCloudTask();
                            task_p2c->setTaskType(TaskType_t::TSKTYPE_P2C);
                            task_p2c->setTaskId(taskid);
                            task_p2c->setTaskStatus(BaseTask::WORK_TO_DO);
                            task_p2c->in.push_back(tuple);
                            this->addTask(task_p2c);
#if (VERBOSE > 1)
                            std::cout << "(Converter) Added TASK \"P2C\" CENTRAL (" << std::get<0>(tuple).get_frame_number() << ") size in: "
                                      << task_p2c->in.size() << " addr: " << &task_p2c << std::endl;
#endif
                        }
                        break;
                    }
                    case CameraType_t::FRONT:
                    case CameraType_t::REAR:
                    {
#if (CONV_SPLIT_DATA == 1)
                        // dividing data in multiple tasks
                        for (rs2::points points : static_cast<FrameToPointsTask*>(tmp_task)->out)
                        {
                            auto data_raw = static_cast<const char*>(points.get_data());
                            auto data_size = points.get_data_size(); // 18432000
                            // rs2::points first, second; // todo: split the data

                            PointsToCloudTask* task_p2c = new PointsToCloudTask();
                            task_p2c->setTaskType(TaskType_t::TSKTYPE_P2C);
                            task_p2c->setTaskId(taskid);
                            task_p2c->setTaskStatus(BaseTask::WORK_TO_DO);
                            task_p2c->in.push_back(points);
                            this->addTask(task_p2c);
                            std::cout << "(Converter) Added TASK \"P2C\" FRONT/REAR (" << points.get_frame_number() << ") size in: "
                                      << task_p2c->in.size() << " addr: " << &task_p2c << std::endl;
                        }

#else
                        for (auto tuple : static_cast<FrameToPointsTask*>(tmp_task)->out)
                        {
                            PointsToCloudTask* task_p2c = new PointsToCloudTask();
                            task_p2c->setTaskType(TaskType_t::TSKTYPE_P2C);
                            task_p2c->setTaskId(taskid);
                            task_p2c->setTaskStatus(BaseTask::WORK_TO_DO);
                            task_p2c->in.push_back(tuple);
                            this->addTask(task_p2c);
#if (VERBOSE > 1)
                            std::cout << "(Converter) Added TASK \"P2C\" FRONT/REAR (" << std::get<0>(tuple).get_frame_number() << ") size in: "
                                      << task_p2c->in.size() << " addr: " << &task_p2c << std::endl;
#endif
                        }
#endif
                        break;
                    }
                    default: break;
                    }
                    break;
                }
                    // Receiving pcl::PointClouds here. Pass it to the PclInterface.
                case TaskType_t::TSKTYPE_P2C:
                {
                    //  pcl::PointCloud<pcl::PointXYZ> tmp_pc(RS_FRAME_POINTS_COUNT,1);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_POINTS_COUNT,1));
#if (CONV_SPLIT_DATA == 1)

                    // combining clouds from tasks

#else
                    switch (taskid) {
                    case CameraType_t::CENTRAL:
                    case CameraType_t::FRONT:
                    case CameraType_t::REAR:
                    {
                        for (auto cloudtuple : static_cast<PointsToCloudTask*>(tmp_task)->out)
                        {
                            for(auto& cloudqueue : *m_ref_to_pcl_queues)
                            {
                                if ( taskid == cloudqueue->getCameraType())
                                {
                                    cloudqueue->addCloudT(cloudtuple);
#if (VERBOSE > 1)
                                    std::cout << "(Converter) Added Cloud to queue " << taskid << " (" << std::get<2>(cloudtuple) << ") size: " << std::get<0>(cloudtuple)->size() << std::endl;
#endif
                                }
                            }
                        }
                        break;
                    }
                    default: break;
                    }
#endif
                    break;
                }
                default: break;

                }
                delete tmp_task;
            }
            else std::cerr << "(Converter) Task in output queue not ready" << std::endl;

        }
        if (idle) std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_CONV_POLL_NS));
    }
}


Rs2_PCL_Converter::Rs2_PCL_Converter(DeviceInterface *in_interface_ref, PclInterface *out_interface_ref, std::vector<CameraType_t> camera_types)
{
    m_ref_to_depth_queues = in_interface_ref->getDepthFrameData();
#if RS_COLOR_ENABLED
    m_ref_to_color_queues = in_interface_ref->getColorFrameData();
#endif

    m_cam_positions = camera_types;

    //    for (auto device : *in_interface_ref->getRs2Devices())
    //        m_cam_positions.push_back(device->getPositionType());

    m_ref_to_pcl_queues = out_interface_ref->getInputCloudsRef();

}

Rs2_PCL_Converter::~Rs2_PCL_Converter()
{

}

void Rs2_PCL_Converter::init(int ThreadPoolSize)
{
#if (VERBOSE > 0)
    std::cout << std::endl <<"(Converter) Initializing " << ThreadPoolSize << " threads"<<std::endl;
#endif
    ThreadController::init(ThreadPoolSize);
}

bool Rs2_PCL_Converter::isActive() { return m_active; }

PointsToCloudTask::PointsToCloudTask()
{
    float rad_front = static_cast<float>(degreesToRadians(ROT_FRONT_TO_CENTRAL_ANG));
    float rad_rear = static_cast<float>(degreesToRadians(ROT_REAR_TO_CENTRAL_ANG));
    if (CONV_RS_ROTATION_AXIS == "X" || CONV_RS_ROTATION_AXIS == "x")
    {
        m_extrinsics_front.rotation[0] = 1; m_extrinsics_front.rotation[3] = 0;             m_extrinsics_front.rotation[6] = 0;
        m_extrinsics_front.rotation[1] = 0; m_extrinsics_front.rotation[4] = std::cos(rad_front); m_extrinsics_front.rotation[7] = -std::sin(rad_front);
        m_extrinsics_front.rotation[2] = 0; m_extrinsics_front.rotation[5] = std::sin(rad_front); m_extrinsics_front.rotation[8] = std::cos(rad_front);
        m_extrinsics_rear.rotation[0] = 1; m_extrinsics_rear.rotation[3] = 0;             m_extrinsics_rear.rotation[6] = 0;
        m_extrinsics_rear.rotation[1] = 0; m_extrinsics_rear.rotation[4] = std::cos(rad_rear); m_extrinsics_rear.rotation[7] = -std::sin(rad_rear);
        m_extrinsics_rear.rotation[2] = 0; m_extrinsics_rear.rotation[5] = std::sin(rad_rear); m_extrinsics_rear.rotation[8] = std::cos(rad_rear);
    }
    else if (CONV_RS_ROTATION_AXIS == "Y" || CONV_RS_ROTATION_AXIS == "y")
    {
        m_extrinsics_front.rotation[0] = std::cos(rad_front);  m_extrinsics_front.rotation[3] = 0; m_extrinsics_front.rotation[6] = std::sin(rad_front);
        m_extrinsics_front.rotation[1] = 0;                    m_extrinsics_front.rotation[4] = 1; m_extrinsics_front.rotation[7] = 0;
        m_extrinsics_front.rotation[2] = -std::sin(rad_front); m_extrinsics_front.rotation[5] = 0; m_extrinsics_front.rotation[8] = std::cos(rad_front);
        m_extrinsics_rear.rotation[0] = std::cos(rad_rear);  m_extrinsics_rear.rotation[3] = 0; m_extrinsics_rear.rotation[6] = std::sin(rad_rear);
        m_extrinsics_rear.rotation[1] = 0;                   m_extrinsics_rear.rotation[4] = 1; m_extrinsics_rear.rotation[7] = 0;
        m_extrinsics_rear.rotation[2] = -std::sin(rad_rear); m_extrinsics_rear.rotation[5] = 0; m_extrinsics_rear.rotation[8] = std::cos(rad_rear);
    }
    else if (CONV_RS_ROTATION_AXIS == "Z" || CONV_RS_ROTATION_AXIS == "z")
    {
        m_extrinsics_front.rotation[0] = std::cos(rad_front); m_extrinsics_front.rotation[3] = -std::sin(rad_front); m_extrinsics_front.rotation[6] = 0;
        m_extrinsics_front.rotation[1] = std::sin(rad_front); m_extrinsics_front.rotation[4] = std::cos(rad_front);  m_extrinsics_front.rotation[7] = 0;
        m_extrinsics_front.rotation[2] = 0;                   m_extrinsics_front.rotation[5] = 0;                    m_extrinsics_front.rotation[8] = 1;
        m_extrinsics_rear.rotation[0] = std::cos(rad_rear); m_extrinsics_rear.rotation[3] = -std::sin(rad_rear); m_extrinsics_rear.rotation[6] = 0;
        m_extrinsics_rear.rotation[1] = std::sin(rad_rear); m_extrinsics_rear.rotation[4] = std::cos(rad_rear);  m_extrinsics_rear.rotation[7] = 0;
        m_extrinsics_rear.rotation[2] = 0;                  m_extrinsics_rear.rotation[5] = 0;                   m_extrinsics_rear.rotation[8] = 1;
    }
    else
    {
        std::cerr << "(Converter) Invalid rotation axis specified" << std::endl;
        m_extrinsics_front.rotation[0] = 1; m_extrinsics_front.rotation[3] = 0; m_extrinsics_front.rotation[6] = 0;
        m_extrinsics_front.rotation[1] = 0; m_extrinsics_front.rotation[4] = 1; m_extrinsics_front.rotation[7] = 0;
        m_extrinsics_front.rotation[2] = 0; m_extrinsics_front.rotation[5] = 0; m_extrinsics_front.rotation[8] = 1;
        m_extrinsics_front.translation[0] = 0;
        m_extrinsics_front.translation[1] = 0;
        m_extrinsics_front.translation[2] = 0;
        m_extrinsics_rear.rotation[0] = 1; m_extrinsics_rear.rotation[3] = 0; m_extrinsics_rear.rotation[6] = 0;
        m_extrinsics_rear.rotation[1] = 0; m_extrinsics_rear.rotation[4] = 1; m_extrinsics_rear.rotation[7] = 0;
        m_extrinsics_rear.rotation[2] = 0; m_extrinsics_rear.rotation[5] = 0; m_extrinsics_rear.rotation[8] = 1;
        m_extrinsics_rear.translation[0] = 0;
        m_extrinsics_rear.translation[1] = 0;
        m_extrinsics_rear.translation[2] = 0;
        return;
    }

    m_extrinsics_front.translation[0] = static_cast<float>(TRAN_FRONT_TO_CENTRAL_X_M);
    m_extrinsics_front.translation[1] = static_cast<float>(TRAN_FRONT_TO_CENTRAL_Y_M);
    m_extrinsics_front.translation[2] = static_cast<float>(TRAN_FRONT_TO_CENTRAL_Z_M);
    m_extrinsics_rear.translation[0] = static_cast<float>(TRAN_REAR_TO_CENTRAL_X_M);
    m_extrinsics_rear.translation[1] = static_cast<float>(TRAN_REAR_TO_CENTRAL_Y_M);
    m_extrinsics_rear.translation[2] = static_cast<float>(TRAN_REAR_TO_CENTRAL_Z_M);

#if (VERBOSE > 1)
    std::cout << "Transformation matrix REAR:" << std::endl
              << m_extrinsics_rear.rotation[0] << " " << m_extrinsics_rear.rotation[3] << " " << m_extrinsics_rear.rotation[6] << " " << m_extrinsics_rear.translation[0] << std::endl
              << m_extrinsics_rear.rotation[1] << " " << m_extrinsics_rear.rotation[1] << " " << m_extrinsics_rear.rotation[7] << " " << m_extrinsics_rear.translation[1] << std::endl
              << m_extrinsics_rear.rotation[2] << " " << m_extrinsics_rear.rotation[5] << " " << m_extrinsics_rear.rotation[8] << " " << m_extrinsics_rear.translation[2] << std::endl;
    std::cout << "Transformation matrix FRONT:" << std::endl
              << m_extrinsics_front.rotation[0] << " " << m_extrinsics_front.rotation[3] << " " << m_extrinsics_front.rotation[6] << " " << m_extrinsics_front.translation[0] << std::endl
              << m_extrinsics_front.rotation[1] << " " << m_extrinsics_front.rotation[1] << " " << m_extrinsics_front.rotation[7] << " " << m_extrinsics_front.translation[1] << std::endl
              << m_extrinsics_front.rotation[2] << " " << m_extrinsics_front.rotation[5] << " " << m_extrinsics_front.rotation[8] << " " << m_extrinsics_front.translation[2] << std::endl;
#endif
}

PointsToCloudTask::~PointsToCloudTask()
{
}

void PointsToCloudTask::process()
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_POINTS_COUNT,1));
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_WIDTH_DEPTH, RS_FRAME_HEIGHT_DEPTH));
    size_t i = 0;
    out.resize(in.size());
#if (VERBOSE > 1)
    auto start = std::chrono::high_resolution_clock::now();
#endif
    while (in.size())
    {
        auto tuple = in.front();
        in.pop_front();
        rs2::points tmp_points = std::get<0>(tuple);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>());
        points_to_pcl(tmp_points, tmp_pc);
        out.at(i++) = std::make_tuple(tmp_pc, std::get<1>(tuple), std::get<2>(tuple));
    }
#if (VERBOSE > 1)
    std::cout << "(Converter) PointsToCloudTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
                 <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
    this->setTaskStatus(TASK_DONE);

}

void PointsToCloudTask::rs2_transform_point_to_point_custom(float *to_point, const rs2_extrinsics *extrin, const float *from_point)
{
    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}

void PointsToCloudTask::points_to_pcl(const rs2::points &points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud)
{
    int skipped = 0;
    const rs2::vertex* rs_vertex = points.get_vertices();
    size_t pt_count = points.size();
    pcloud->reserve(pt_count);

    switch (this->getTaskId()) {
    case CameraType_t::CENTRAL: // no transformation
        for (size_t i=0; i<pt_count; i++)
        {
            if ( !areSameF(rs_vertex->z, 0.0f) )
            {
                pcloud->push_back(pcl::PointXYZ(rs_vertex->x, rs_vertex->y, rs_vertex->z));
            }
            else skipped++;
            rs_vertex++;
        }
        break;
    case CameraType_t::FRONT:
        for (size_t i=0; i<pt_count; i++)
        {
            if ( !areSameF(rs_vertex->z, 0.0f) )
            {
                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
                float target[3];
                rs2_transform_point_to_point_custom(target, &m_extrinsics_front, origin);
                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
            }
            else skipped++;
            rs_vertex++;
        }
        break;
    case CameraType_t::REAR:
        for (size_t i=0; i<pt_count; i++)
        {
            if ( !areSameF(rs_vertex->z, 0.0f) )
            {
                float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
                float target[3];
                rs2_transform_point_to_point_custom(target, &m_extrinsics_rear, origin);
                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
            }
            else skipped++;
            rs_vertex++;
        }
        break;
    }
}

FrameToPointsTask::FrameToPointsTask()
{

}

FrameToPointsTask::~FrameToPointsTask()
{

}

void FrameToPointsTask::process()
{
    size_t i = 0;
    out.resize(in.size());
    rs2::pointcloud tmp_rs2_pc;
#if (VERBOSE > 1)
    auto start = std::chrono::high_resolution_clock::now();
#endif
    while (in.size())
    {
        rs2::frame tmp_frame_depth = in.front();
        in.pop_front();
#if RS_COLOR_ENABLED
        rs2::frame tmp_frame_color = in.front();
        in.pop_front();
#endif
        rs2_metadata_type sensor_ts = 0;
        if (tmp_frame_depth.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP))
            sensor_ts = tmp_frame_depth.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
        else std::cerr << "(FrameToPointsTask) Error retrieving frame timestamp" << std::endl;
#if RS_COLOR_ENABLED
        tmp_rs2_pc.map_to(tmp_frame_color);
        std::cout << "DEBUG F2P depth size:" << tmp_frame_depth.get_data_size() << " color size: " << tmp_frame_color.get_data_size() << std::endl;
#else
        tmp_rs2_pc.map_to(tmp_frame_depth);
#endif
        out.at(i++) = std::make_tuple(tmp_rs2_pc.calculate(tmp_frame_depth), sensor_ts, tmp_frame_depth.get_frame_number());
    }
#if (VERBOSE > 1)
    std::cout << "(Converter) FrameToPointsTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
                 <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
    this->setTaskStatus(TASK_DONE);
}

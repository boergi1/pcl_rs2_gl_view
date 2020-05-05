#include "rs2_pcl_converter.h"

void Rs2_PCL_Converter::converter_thread_func()
{
    std::cout << "(Converter) Thread started, id: " << std::this_thread::get_id() << std::endl;

    //    m_active = true;
    while (m_active) {
        // Read from RealSense2 Devices and generate tasks
        for (size_t i = 0; i < m_ref_to_rs2_frames->size(); i++)
        {
            auto &queue = m_ref_to_rs2_frames->at(i);
            rs2::frame tmp_rs2_frame;
            while ( queue.poll_for_frame(&tmp_rs2_frame) )
            {
                FrameToPointsTask* task_f2p = new FrameToPointsTask();
                task_f2p->setTaskType(TaskType_t::TSKTYPE_F2P);
                task_f2p->setTaskId(m_cam_positions.at(i));
                task_f2p->setTaskStatus(BaseTask::WORK_TO_DO);
                task_f2p->in.push_back(tmp_rs2_frame);
                this->addTask(task_f2p);
                std::cout << "(Converter) Added TASK \"F2P\" (" << tmp_rs2_frame.get_frame_number() << ") size in: "
                          << task_f2p->in.size() << " addr: " << &task_f2p << std::endl;
            }
        }

        // Read tasks from output
        std::cout << "(Converter) Handle " << this->OutputQueue.size() << " finished tasks" << std::endl;
        while(this->OutputQueue.size())
        {
            BaseTask* tmp_task = getTask();
            auto taskid = tmp_task->getTaskId();
            if (tmp_task->getTaskStatus() == BaseTask::TaskStatus_t::TASK_DONE)
            {
                switch (tmp_task->getTaskType()) {
                // Read converted points and create new tasks
                case TaskType_t::TSKTYPE_F2P:
                {
                    switch (taskid) {
                    case CamPosition_t::CENTRAL:
                    {
                        for (auto points : static_cast<FrameToPointsTask*>(tmp_task)->out)
                        {
                            PointsToCloudTask* task_p2c = new PointsToCloudTask();
                            task_p2c->setTaskType(TaskType_t::TSKTYPE_P2C);
                            task_p2c->setTaskId(taskid);
                            task_p2c->setTaskStatus(BaseTask::WORK_TO_DO);
                            task_p2c->in.push_back(points);
                            this->addTask(task_p2c);
                            std::cout << "(Converter) Added TASK \"P2C\" CENTRAL (" << points.get_frame_number() << ") size in: "
                                      << task_p2c->in.size() << " addr: " << &task_p2c << std::endl;
                        }
                        break;
                    }
                    case CamPosition_t::FRONT:
                    case CamPosition_t::REAR:
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
                        for (rs2::points points : static_cast<FrameToPointsTask*>(tmp_task)->out)
                        {
                            PointsToCloudTask* task_p2c = new PointsToCloudTask();
                            task_p2c->setTaskType(TaskType_t::TSKTYPE_P2C);
                            task_p2c->setTaskId(taskid);
                            task_p2c->setTaskStatus(BaseTask::WORK_TO_DO);
                            task_p2c->in.push_back(points);
                            this->addTask(task_p2c);
                            std::cout << "(Converter) Added TASK f2p FRONT/REAR (" << points.get_frame_number() << ") size in: "
                                      << task_p2c->in.size() << " addr: " << &task_p2c << std::endl;
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
                    case CamPosition_t::CENTRAL:
                    case CamPosition_t::FRONT:
                    case CamPosition_t::REAR:
                    {
                        for (auto cloudtuple : static_cast<PointsToCloudTask*>(tmp_task)->out)
                            m_ref_to_pcl_queues->at(static_cast<size_t>(taskid))->addCloudT(cloudtuple);
                        break;
                    }
                    default: break;
                    }
#endif
                    break;
                }
                default: break;

                }
                //  delete tmp_task; // -> ??????
            }
            else std::cerr << "(Converter) Task in output queue not ready" << std::endl;

        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_CONV_POLL_NS));
    }
}


Rs2_PCL_Converter::Rs2_PCL_Converter(DeviceInterface *in_interface_ref, PclInterface *out_interface_ref)
{
    m_ref_to_rs2_frames = in_interface_ref->getDepthFrameData();

    for (auto device : *in_interface_ref->getRs2Devices())
        m_cam_positions.push_back(device->getPositionType());

    m_ref_to_pcl_queues = out_interface_ref->getInputCloudsRef();

}

Rs2_PCL_Converter::~Rs2_PCL_Converter()
{

}

void Rs2_PCL_Converter::init()
{
    ThreadController::init();
}

bool Rs2_PCL_Converter::isActive() { return m_active; }

PointsToCloudTask::PointsToCloudTask()
{

}

PointsToCloudTask::~PointsToCloudTask()
{

}

void PointsToCloudTask::process()
{
    std::cout<<"(ProcessTask) Points -> PointCloud conversion: " << in.size() <<std::endl;
    size_t i = 0;
    //  pcl::PointCloud<pcl::PointXYZ> tmp_pc(RS_FRAME_POINTS_COUNT,1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_POINTS_COUNT,1));

    out.resize(in.size());
    while (in.size())
    {
        rs2::points tmp_points = in.front();
        in.pop_front();
        // points_to_pcl(tmp_points, pcl::PointCloud<pcl::PointXYZ>::Ptr(&tmp_pc));
        points_to_pcl(tmp_points, tmp_pc);
        out.at(i++) = std::make_tuple(tmp_pc, tmp_points.get_timestamp(), tmp_points.get_frame_number());
    }
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
#if (VERBOSE > 2)
    auto start = std::chrono::high_resolution_clock::now();
#endif
    int skipped = 0;
    auto rs_vertex = points.get_vertices();

    switch (this->getTaskId()) {
    case CamPosition_t::CENTRAL: // no transformation
        for (auto& pcl_point : pcloud->points)
        {
            pcl_point.x = rs_vertex->x;
            pcl_point.y = rs_vertex->y;
            pcl_point.z = rs_vertex->z;
            rs_vertex++;
        }
        break;
    case CamPosition_t::FRONT:
        //  case TaskType_t::TID_P2C_F_2:
    {
        float rad = static_cast<float>(degreesToRadians(ROT_RS1_TO_RS0_X_ANG));
        rs2_extrinsics extrinsics_T_Rx;
        extrinsics_T_Rx.rotation[0] = 1; extrinsics_T_Rx.rotation[3] = 0;             extrinsics_T_Rx.rotation[6] = 0;
        extrinsics_T_Rx.rotation[1] = 0; extrinsics_T_Rx.rotation[4] = std::cos(rad); extrinsics_T_Rx.rotation[7] = -std::sin(rad);
        extrinsics_T_Rx.rotation[2] = 0; extrinsics_T_Rx.rotation[5] = std::sin(rad); extrinsics_T_Rx.rotation[8] = std::cos(rad);
        extrinsics_T_Rx.translation[0] = static_cast<float>(TRAN_RS1_TO_RS0_X_M);
        extrinsics_T_Rx.translation[1] = static_cast<float>(TRAN_RS1_TO_RS0_Y_M);
        extrinsics_T_Rx.translation[2] = static_cast<float>(TRAN_RS1_TO_RS0_Z_M);
#if (VERBOSE > 3)
        std::cerr << "Transformation matrix:" << std::endl
                  << extrinsics_T_Rx.rotation[0] << " " << extrinsics_T_Rx.rotation[3] << " " << extrinsics_T_Rx.rotation[6] << " " << extrinsics_T_Rx.translation[0] << std::endl
                  << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[7] << " " << extrinsics_T_Rx.translation[1] << std::endl
                  << extrinsics_T_Rx.rotation[2] << " " << extrinsics_T_Rx.rotation[5] << " " << extrinsics_T_Rx.rotation[8] << " " << extrinsics_T_Rx.translation[2] << std::endl;
#endif
        for (auto& pcl_point : pcloud->points)
        {
            if (areSameF(0.0f, rs_vertex->z))
            {
                skipped++;
                pcl_point.x = rs_vertex->x;
                pcl_point.y = rs_vertex->y;
                pcl_point.z = rs_vertex->z;
                rs_vertex++;
                continue;
            }
            float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
            float target[3];
            rs2_transform_point_to_point_custom(target, &extrinsics_T_Rx, origin);
            pcl_point.x = target[0];
            pcl_point.y = target[1];
            pcl_point.z = target[2];
            rs_vertex++;
#if (VERBOSE > 4)
            std::cerr << "Point before: (" << origin[0] << "," << origin[1] << "," << origin[2] << ")\t" <<
                         "Point after: (" << target[0] << "," << target[1] << "," << target[2] << ")" << std::endl;
#endif
        }
        break;
    }
    case CamPosition_t::REAR:
        // case TaskType_t::TID_P2C_R_2:
    {
        float rad = static_cast<float>(degreesToRadians(ROT_RS2_TO_RS0_X_ANG));
        rs2_extrinsics extrinsics_T_Rx;
        extrinsics_T_Rx.rotation[0] = 1; extrinsics_T_Rx.rotation[3] = 0;             extrinsics_T_Rx.rotation[6] = 0;
        extrinsics_T_Rx.rotation[1] = 0; extrinsics_T_Rx.rotation[4] = std::cos(rad); extrinsics_T_Rx.rotation[7] = -std::sin(rad);
        extrinsics_T_Rx.rotation[2] = 0; extrinsics_T_Rx.rotation[5] = std::sin(rad); extrinsics_T_Rx.rotation[8] = std::cos(rad);
        extrinsics_T_Rx.translation[0] = static_cast<float>(TRAN_RS2_TO_RS0_X_M);
        extrinsics_T_Rx.translation[1] = static_cast<float>(TRAN_RS2_TO_RS0_Y_M);
        extrinsics_T_Rx.translation[2] = static_cast<float>(TRAN_RS2_TO_RS0_Z_M);
#if (VERBOSE > 3)
        std::cerr << "Transformation matrix:" << std::endl
                  << extrinsics_T_Rx.rotation[0] << " " << extrinsics_T_Rx.rotation[3] << " " << extrinsics_T_Rx.rotation[6] << " " << extrinsics_T_Rx.translation[0] << std::endl
                  << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[1] << " " << extrinsics_T_Rx.rotation[7] << " " << extrinsics_T_Rx.translation[1] << std::endl
                  << extrinsics_T_Rx.rotation[2] << " " << extrinsics_T_Rx.rotation[5] << " " << extrinsics_T_Rx.rotation[8] << " " << extrinsics_T_Rx.translation[2] << std::endl;
#endif
        for (auto& pcl_point : pcloud->points)
        {
            if (areSameF(0.0f, rs_vertex->z))
            {
                skipped++;
                pcl_point.x = rs_vertex->x;
                pcl_point.y = rs_vertex->y;
                pcl_point.z = rs_vertex->z;
                rs_vertex++;
                continue;
            }
            float origin[3] { rs_vertex->x, rs_vertex->y, rs_vertex->z };
            float target[3];
            rs2_transform_point_to_point_custom(target, &extrinsics_T_Rx, origin);
            pcl_point.x = target[0];
            pcl_point.y = target[1];
            pcl_point.z = target[2];
            rs_vertex++;
#if (VERBOSE > 4)
            std::cerr << "Point before: (" << origin[0] << "," << origin[1] << "," << origin[2] << ")\t" <<
                         "Point after: (" << target[0] << "," << target[1] << "," << target[2] << ")" << std::endl;
#endif
        }
        break;
    }
    default:
        std::cerr << "(Converter) Error: invalid device index" << std::endl;
    }
    //#if (VERBOSE > 2)
    //    std::cout << "(Converter) Task (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") converted "
    //              << pcloud->points.size() << " points to clouds, skipped " << skipped << ", took " << std::chrono::duration_cast
    //                 <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
    //#endif
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
    while (in.size())
    {
        rs2::frame tmp_frame = in.front();
        in.pop_front();
        //        rs2::pointcloud tmp_rs2_pc;
        tmp_rs2_pc.map_to(tmp_frame);
        out.at(i++) = tmp_rs2_pc.calculate(tmp_frame);
    }
    this->setTaskStatus(TASK_DONE);
}

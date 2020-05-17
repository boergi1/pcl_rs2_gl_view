#include "rs2_pcl_converter.h"

void Rs2_PCL_Converter::converter_thread_func()
{
    std::cout << "(Converter) Thread started, id: " << std::this_thread::get_id() << std::endl;

    //    m_active = true;
    while (m_active) {
        bool idle = true;
        // Read from RealSense2 Devices and generate tasks
        //        for (size_t i = 0; i < m_ref_to_depth_queues->size(); i++)
        for (size_t i = 0; i < m_ref_to_framesets->size(); i++)
        {
            //            auto queue_depth = m_ref_to_depth_queues->at(i);
            //#if RS_COLOR_ENABLED
            //            auto queue_color = m_ref_to_color_queues->at(i);
            //#endif
            auto fs_deque = m_ref_to_framesets->at(i);
            //            while ( !queue_depth->isEmpty() )
            while ( !fs_deque->isEmpty() )
            {
                idle = false;
                FrameToPointsTask* task_f2p = new FrameToPointsTask();
                task_f2p->setTaskType(TaskType_t::TSKTYPE_F2P);
                task_f2p->setTaskId(m_cam_positions.at(i));
                task_f2p->setTaskStatus(BaseTask::WORK_TO_DO);
                //                task_f2p->in.push_back(queue_depth->getFrame());
                //#if RS_COLOR_ENABLED
                //                task_f2p->in.push_back(queue_color->getFrame());
                //#endif
                task_f2p->in.push_back(fs_deque->getFrame());
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
                                    auto pc = std::get<0>(cloudtuple);
#if RS_COLOR_ENABLED
                                    //  cloudqueue->addCloudT(cloudtuple);
#else
                                    cloudqueue->addCloudT(cloudtuple);
#endif
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
    //    m_ref_to_depth_queues = in_interface_ref->getDepthFrameData();
    //#if RS_COLOR_ENABLED
    //    m_ref_to_color_queues = in_interface_ref->getColorFrameData();
    //#endif
    m_ref_to_framesets = in_interface_ref->getFrameSetData();

    m_cam_positions = camera_types;

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
        rs2::depth_frame tmp_frame_depth = in.front().get_depth_frame();
#if RS_COLOR_ENABLED
        rs2::video_frame tmp_frame_color = in.front().get_color_frame();
#endif
        in.pop_front();
        rs2_metadata_type sensor_ts = 0;
        if (tmp_frame_depth.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP))
            sensor_ts = tmp_frame_depth.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
        else std::cerr << "(FrameToPointsTask) Error retrieving frame timestamp" << std::endl;
#if RS_COLOR_ENABLED
        tmp_rs2_pc.map_to(tmp_frame_color);
        out.at(i++) = std::make_tuple(tmp_rs2_pc.calculate(tmp_frame_depth), tmp_frame_color, sensor_ts, tmp_frame_depth.get_frame_number());
#else
        tmp_rs2_pc.map_to(tmp_frame_depth);
        out.at(i++) = std::make_tuple(tmp_rs2_pc.calculate(tmp_frame_depth), sensor_ts, tmp_frame_depth.get_frame_number());
#endif
    }
#if (VERBOSE > 1)
    std::cout << "(Converter) FrameToPointsTask (type:" << this->getTaskType() << " id:" << this->getTaskId() << ") took " << std::chrono::duration_cast
                 <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
    this->setTaskStatus(TASK_DONE);
}


PointsToCloudTask::~PointsToCloudTask()
{
}

void PointsToCloudTask::process()
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_POINTS_COUNT,1));
#if PCL_CLOUD_ORGANIZED
#if RS_COLOR_ENABLED
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>(RS_FRAME_WIDTH_DEPTH/RS_FILTER_DEC_MAG, RS_FRAME_HEIGHT_DEPTH/RS_FILTER_DEC_MAG));
#else
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>(RS_FRAME_WIDTH_DEPTH/RS_FILTER_DEC_MAG, RS_FRAME_HEIGHT_DEPTH/RS_FILTER_DEC_MAG));
#endif
#endif
    size_t i = 0;
    out.resize(in.size());
#if (VERBOSE > 1)
    auto start = std::chrono::high_resolution_clock::now();
#endif
    while (in.size())
    {
        auto tuple = in.front();
        in.pop_front();
#if !PCL_CLOUD_ORGANIZED
#if RS_COLOR_ENABLED
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
#else
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>());
#endif
#endif
        rs2::points tmp_points = std::get<0>(tuple);
#if RS_COLOR_ENABLED
        rs2::frame color = std::get<1>(tuple);
        auto ts =  std::get<2>(tuple);
        auto ctr =  std::get<3>(tuple);
        points_to_pcl_rgb(tmp_points, color, tmp_pc);
#else
        auto ts =  std::get<1>(tuple);
        auto ctr =  std::get<2>(tuple);
        points_to_pcl(tmp_points, tmp_pc);
#endif


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
#if !PCL_CLOUD_ORGANIZED
    pcloud->reserve(pt_count);
#endif

    switch (this->getTaskId()) {
    case CameraType_t::CENTRAL: // no transformation
        for (size_t i=0; i<pt_count; i++)
        {
            if ( !areSameF(rs_vertex->z, 0.0f) )
            {
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = rs_vertex->x;
                pcloud->points.at(i).y = rs_vertex->y;
                pcloud->points.at(i).z = rs_vertex->z;
#else
                pcloud->push_back(pcl::PointXYZ(rs_vertex->x, rs_vertex->y, rs_vertex->z));
#endif
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
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = target[0];
                pcloud->points.at(i).y = target[1];
                pcloud->points.at(i).z = target[2];
#else
                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
#endif
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
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = target[0];
                pcloud->points.at(i).y = target[1];
                pcloud->points.at(i).z = target[2];
#else
                pcloud->push_back(pcl::PointXYZ(target[0], target[1], target[2]));
#endif
            }
            else skipped++;
            rs_vertex++;
        }
        break;
    }
}

inline void PointsToCloudTask::textureToColor(const rs2::video_frame* rgb_frame, const rs2::texture_coordinate* texture_uv, std::tuple<uint8_t, uint8_t, uint8_t>* output)
{
    // Get Width and Height coordinates of texture
    int width  = rgb_frame->get_width();  // Frame width in pixels
    int height = rgb_frame->get_height(); // Frame height in pixels
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(texture_uv->u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(texture_uv->v * height + .5f), 0), height - 1);
    int bytes = x_value * rgb_frame->get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * rgb_frame->get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);
    const auto New_Texture = reinterpret_cast<const uint8_t*>(rgb_frame->get_data());
    // RGB components to save in tuple
    *output = std::make_tuple<int, int, int>(New_Texture[Text_Index], New_Texture[Text_Index+1], New_Texture[Text_Index+2]);
}


void PointsToCloudTask::points_to_pcl_rgb(const rs2::points &points, const rs2::video_frame& color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud)
{

    int skipped = 0;
    const rs2::vertex* rs_vertex = points.get_vertices();
    size_t pt_count = points.size();
#if !PCL_CLOUD_ORGANIZED
    pcloud->reserve(pt_count);
#endif

    std::tuple<uint8_t, uint8_t, uint8_t> rgb;
    auto Texture_Coord = points.get_texture_coordinates();


    switch (this->getTaskId()) {
    case CameraType_t::CENTRAL: // no transformation
        for (size_t i=0; i<pt_count; i++)
        {
            if ( !areSameF(rs_vertex->z, 0.0f) )
            {
                textureToColor(&color, &Texture_Coord[i], &rgb);
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = rs_vertex->x;
                pcloud->points.at(i).y = rs_vertex->y;
                pcloud->points.at(i).z = rs_vertex->z;
                pcloud->points.at(i).r = std::get<0>(rgb);
                pcloud->points.at(i).g = std::get<1>(rgb);
                pcloud->points.at(i).b = std::get<2>(rgb);
#else
                pcl::PointXYZRGB pt = pcl::PointXYZRGB(std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb));
                pt.x = rs_vertex->x;
                pt.y = rs_vertex->y;
                pt.z = rs_vertex->z;
                pcloud->push_back(pt);
#endif
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
                textureToColor(&color, &Texture_Coord[i], &rgb);
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = target[0];
                pcloud->points.at(i).y = target[1];
                pcloud->points.at(i).z = target[2];
                pcloud->points.at(i).r = std::get<0>(rgb);
                pcloud->points.at(i).g = std::get<1>(rgb);
                pcloud->points.at(i).b = std::get<2>(rgb);
#else
                pcl::PointXYZRGB pt = pcl::PointXYZRGB(std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb));
                pt.x = target[0];
                pt.y = target[1];
                pt.z = target[2];
                pcloud->push_back(pt);
#endif
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
                textureToColor(&color, &Texture_Coord[i], &rgb);
#if PCL_CLOUD_ORGANIZED
                pcloud->points.at(i).x = target[0];
                pcloud->points.at(i).y = target[1];
                pcloud->points.at(i).z = target[2];
                pcloud->points.at(i).r = std::get<0>(rgb);
                pcloud->points.at(i).g = std::get<1>(rgb);
                pcloud->points.at(i).b = std::get<2>(rgb);
#else
                pcl::PointXYZRGB pt = pcl::PointXYZRGB(std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb));
                pt.x = target[0];
                pt.y = target[1];
                pt.z = target[2];
                pcloud->push_back(pt);
#endif
            }
            else skipped++;
            rs_vertex++;
        }
        break;
    }

}

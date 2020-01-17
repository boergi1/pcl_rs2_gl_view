#ifndef FORMAT_H
#define FORMAT_H

#define FRAME_WIDTH 640//1280
#define FRAME_HEIGHT 480//720
#define FRAME_RATE 15 // due rs2_pipeline_start_with_config_and_callback_cpp
#define FRAME_PERIOD 1000/FRAME_RATE

#define CONV_DELAY FRAME_PERIOD/2 // rs2 to pcl converter race condition
#define PROC_DELAY CONV_DELAY

#define POINT_BUF_SIZE 100
#define CLOUD_BUF_SIZE 100

#endif // FORMAT_H

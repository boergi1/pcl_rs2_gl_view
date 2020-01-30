#ifndef OCVDEVICE_H
#define OCVDEVICE_H

#include "format.h"

#include <iostream>
#include <thread>
#include <mutex>

#include <opencv2/core/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking.hpp>



class OcvDevice
{
private:
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_cap;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_seg;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_tra;


    bool m_capture_running = true;
    std::thread m_capture_thread;
    std::thread m_segmentation_thread;
    std::thread m_tracking_thread;

    const double mm_per_pix = static_cast<double>(REF_SIZE_MM)/static_cast<double>(REF_PIXEL);

    cv::VideoCapture* m_capture = nullptr;

    std::vector<std::string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

    // internal buffers and mutexes
    std::mutex m_marker_mutex;
    cv::Mat** m_capture_buf;
    size_t m_capture_write_idx, m_marker_read_idx;

    // external refs
    std::mutex* m_mutex_ref;
    int m_cam_idx;
    cv::Mat* m_ocv_mat_buf_ref;
    size_t* m_mat_write_idx_ref;

    cv::Ptr<cv::Tracker> createTrackerByName(std::string trackerType)
    {

        cv::Ptr<cv::Tracker> tracker;
        if (trackerType ==  trackerTypes[0])
            tracker = cv::TrackerBoosting::create();
        else if (trackerType == trackerTypes[1])
            tracker = cv::TrackerMIL::create();
        else if (trackerType == trackerTypes[2])
            tracker = cv::TrackerKCF::create();
        else if (trackerType == trackerTypes[3])
            tracker = cv::TrackerTLD::create();
        else if (trackerType == trackerTypes[4])
            tracker = cv::TrackerMedianFlow::create();
        else if (trackerType == trackerTypes[5])
            tracker = cv::TrackerGOTURN::create();
        else if (trackerType == trackerTypes[6])
            tracker = cv::TrackerMOSSE::create();
        else if (trackerType == trackerTypes[7])
            tracker = cv::TrackerCSRT::create();
        else {
            std::cerr << "Incorrect tracker name" << std::endl;
        }
        return tracker;
    }

    void capture_thread_func()
    {
        std::cout << "OpenCV capture thread started # " << std::this_thread::get_id() << " device: " << m_cam_idx << std::endl;
        cv::Mat image;
        m_capture = new cv::VideoCapture(m_cam_idx); // cv::CAP_V4L2
        m_capture->set(cv::CAP_PROP_FRAME_WIDTH, CV_FRAME_WIDTH);
        m_capture->set(cv::CAP_PROP_FRAME_HEIGHT, CV_FRAME_HEIGHT);
        m_capture->set(cv::CAP_PROP_FPS, CV_FRAME_RATE);
        std::cout << "Capturing with resolution " << m_capture->get(cv::CAP_PROP_FRAME_WIDTH) << " x "
                  << m_capture->get(cv::CAP_PROP_FRAME_HEIGHT) << " at " << m_capture->get(cv::CAP_PROP_FPS) << " fps" << std::endl;

        if (m_capture->isOpened())
            m_segmentation_thread = std::thread(&OcvDevice::segmentation_thread_func, this);
#ifdef IMSHOW_CAP
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {
            // next image
            m_capture->read(image);
            m_start_time_cap = std::chrono::steady_clock::now();
            // calculate markers
            cv::Mat markers = cv::Mat::zeros(image.size(), CV_32S);
#ifdef IMSHOW_CAP
            cv::imshow("Input", image);
#endif
            prepareObjectMarkers(image, markers);
            // write to buffer
            m_marker_mutex.lock();
            m_capture_buf[m_capture_write_idx][0] = image;
            m_capture_buf[m_capture_write_idx++][1] = markers;
            if (m_capture_write_idx == CAP_BUF_SIZE)
                m_capture_write_idx = 0;
            std::cout << "(OpenCV capture) Increased write index: " << m_capture_write_idx << " size0 " << image.size() << " type0 " << image.type() << " size1 "
                      << markers.size() << " type1 " << markers.type() << std::endl;
            m_marker_mutex.unlock();
            std::cout << "(OpenCV capture) Total took: " << std::chrono::duration_cast
                         <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
        }
        std::cout << "Capture thread is exiting" << std::endl;
    }

    void prepareObjectMarkers(cv::Mat& colorimg, cv::Mat& markers)
    {
        double dist_trans_thresh = 0.1;
        cv::Mat background, dt, tmp;
        // pre processing
        cv::cvtColor(colorimg, tmp, cv::COLOR_BGR2GRAY);
        cv::threshold(tmp, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // filtering noise with opening
        cv::morphologyEx(tmp, tmp, cv::MORPH_OPEN, cv::Mat::ones(3 , 3, CV_8U));
        // extract bg and invert
        std::cout << "(OpenCV capture) After preprocessing: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 3); // 8-14 ms
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5)), cv::Point(-1,-1), 3); // 10 - 16 ms
        cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 5); // 10-12 ms

        cv::threshold(background, background, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        // distance transformation
        cv::distanceTransform(tmp, tmp, cv::DIST_L2, 3);
        std::cout << "(OpenCV capture) After distance transformation: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
        //  cv::imshow("Normalized Distance Transformation", tmp);
        if ((false))
        { // 5-7 ms
            cv::normalize(tmp, tmp, 0, 1.0, cv::NORM_MINMAX);
            cv::threshold(tmp, tmp, dist_trans_thresh, 1, cv::THRESH_BINARY);
        }
        else {
            if ((false))
            {
                double max;
                cv::minMaxLoc(tmp, nullptr, &max);
                cv::threshold(tmp, tmp, dist_trans_thresh*max, max, cv::THRESH_BINARY);
            }
            else
            {
                cv::threshold(tmp, tmp, dist_trans_thresh*255, 255, cv::THRESH_BINARY);
            }
        }
        //   cv::imshow("Normalized Distance Transformation Thresh", tmp);
        std::cout << "(OpenCV capture) After normalize thresh: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
        // segmenting contours
        std::vector<std::vector<cv::Point>> contours;
        tmp.convertTo(tmp, CV_8U);
        findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++)
            drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
        std::cout << "(OpenCV capture) After find&draw contours: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
        // manual marker operations
        markers.convertTo(tmp, CV_8U);
        cv::bitwise_or(tmp, background, tmp);
#ifdef IMSHOW_CAP
        cv::imshow("Watershed markers", tmp*(255/contours.size())-10);
#endif
        tmp.convertTo(markers, CV_32S);
    }

    void segmentation_thread_func()
    {
        bool tracker = false;
        std::cout << "OpenCV segmentation thread started # " << std::this_thread::get_id() << std::endl;
        cv::Ptr<cv::MultiTracker> multiTracker = cv::MultiTracker::create();


        while (m_marker_read_idx == m_capture_write_idx) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        //        std::cout << "init tracker" << std::endl;
        //        m_marker_mutex.lock();
        //        //  cv::Mat color = m_capture_buf[m_marker_read_idx][0];
        //        cv::Mat tracker_init;
        //        m_capture_buf[m_marker_read_idx][0].convertTo(tracker_init, CV_8U);
        //        //        if (m_marker_read_idx == POINT_BUF_SIZE)
        //        //            m_marker_read_idx = 0;
        //        m_marker_mutex.unlock();




        //    multiTracker->add(createTrackerByName("CSRT"), cv::Mat::zeros(CV_FRAME_WIDTH, CV_FRAME_HEIGHT, CV_8UC1), cv::Rect2d(0, 0, CV_FRAME_WIDTH, CV_FRAME_HEIGHT));


        //   multiTracker->add(createTrackerByName("CSRT"), tracker_init, cv::Rect2d(0, 0, CV_FRAME_WIDTH, CV_FRAME_HEIGHT));

        // std::cout << "tracker inited" << std::endl;

#ifdef IMSHOW_SEG
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {

            if (m_marker_read_idx != m_capture_write_idx)
            {
                m_start_time_seg = std::chrono::steady_clock::now();
                // Read
                m_marker_mutex.lock();
                cv::Mat color = m_capture_buf[m_marker_read_idx][0];
                cv::Mat marker = m_capture_buf[m_marker_read_idx++][1];
                if (m_marker_read_idx == POINT_BUF_SIZE)
                    m_marker_read_idx = 0;
                std::cout << "(OpenCV segmentation) Increased read index: " << m_marker_read_idx << " size0 " << color.size() << " type0 " << color.type() << " size1 "
                          << marker.size() << " type1 " << marker.type() << std::endl;
                m_marker_mutex.unlock();




                // Segmenting by watershed algorithm
                marker.convertTo(marker, CV_32S);
                cv::watershed(color, marker); // 6-20 ms
                marker.convertTo(marker, CV_8U);
                std::cout << "After watershed: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
                // Get bounding boxes
                cv::Mat labels, stats, centroids;
                int marker_count = cv::connectedComponentsWithStats(marker, labels, stats, centroids, 4, CV_32S); // 10-15 ms

#if defined(IMSHOW_SEG) || defined(IMSHOW_CAP)
        for (int i = 0; i < marker_count; i++)
        {


            //        testvec[i] = rect;
            //           testvec.push_back(rect);


            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            cv::Rect rect = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                                     stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));

            cv::rectangle(markers, rect, cv::Scalar(0), 1);
            cv::putText(markers, std::to_string((int)(mm_per_pix*rect.width))+" x "+std::to_string((int)(mm_per_pix*rect.height)),
                        cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
            if (i > 0)
            {
                std::cout << "Label " << i << " - pos: (" << rect.x << "," << rect.y << ") center: (" << cx << "," << cy
                          << ") size: " << rect.width << " x " << rect.height << " area: " << area << std::endl;
            }
            else {
                std::cout << "Background - pos: (" << rect.x << "," << rect.y << ") center: (" << cx << "," << cy
                          << ") size: " << rect.width << " x " << rect.height << " area: " << area << std::endl;
            }
        }
#endif



//                // Update Multitracker
//                if (!tracker)
//                {
//                    for(int i=0; i < marker_count; i++)
//                    {
//                        std::cout << "tracker added " << i << std::endl;
//                        multiTracker->add(createTrackerByName("CSRT"), marker, cv::Rect2d(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
//                                                                                         stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT) ));
//                    }
//                }

//                if (!multiTracker->update(color))
//                {
//                    std::cout << "tracker not updated" << std::endl;
//                }
//                else
//                {
//                    std::cout << "tracker updated" << std::endl;
//                    // draw tracked objects
//                    for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
//                    {
//                        std::cout << "tracker added rect " << i << ": " << multiTracker->getObjects()[i] << std::endl;
//                        rectangle(color, multiTracker->getObjects()[i], cv::Scalar(0), 10);
//                    }
//                }
//                cv::imshow("tracker", color);
//                cv::waitKey(1);





                //    int elements = segmentObjects(color, marker);


#ifdef IMSHOW_SEG
                cv::imshow("Segmented image", marker*(255/elements));
#endif
                // Write
                std::cout << "(OpenCV segmentation) Total took: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;

            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(SEG_DELAY));
            }


            //            if (*m_points_read_idx_ref != *m_points_write_idx_ref)
            //            {
            //                // Read from rs2::points buffer
            //                m_points_mutex_ref->lock();
            //                rs2::points points = m_points_buf_ref[ *m_points_read_idx_ref ];
            //                *m_points_read_idx_ref = *m_points_read_idx_ref + 1;
            //                if (*m_points_read_idx_ref == POINT_BUF_SIZE-1)
            //                    *m_points_read_idx_ref = 0;
            //                cout << "(Converter) Increased read index: " << *m_points_read_idx_ref << " size " << points.size() << endl;
            //                m_points_mutex_ref->unlock();

            //                // Convert to pcl::PointCloud cloud
            //                pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud = points_to_pcl(points);

            //                // Write to pcl::PointCloud buffer
            //                m_clouds_mutex_ref->lock();
            //                 m_clouds_buf_ref->at(*m_clouds_write_idx_ref) = converted_cloud; // m_current_cloud->makeShared()
            //                auto cloud_size =  m_clouds_buf_ref->at(*m_clouds_write_idx_ref)->size();
            //                *m_clouds_write_idx_ref = *m_clouds_write_idx_ref + 1;
            //                if (*m_clouds_write_idx_ref == POINT_BUF_SIZE-1)
            //                    *m_clouds_write_idx_ref = 0;
            //                cout << "(Converter) Increased write index: " << *m_clouds_write_idx_ref << " size " << cloud_size << endl;
            //                m_clouds_mutex_ref->unlock();

            //            }




        }
        std::cout << "Segmentation thread is exiting" << std::endl;
    }


    int segmentObjects(cv::Mat& colorimg, cv::Mat& markers)
    {
        // Segmenting by watershed algorithm
        markers.convertTo(markers, CV_32S);
        cv::watershed(colorimg, markers); // 6-20 ms
        markers.convertTo(markers, CV_8U);
        // Segmenting with stats for bounding boxes

        cv::Mat labels(markers.size(), CV_32S);
        cv::Mat stats, centroids;
        std::cout << "After watershed: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
        int marker_count = cv::connectedComponentsWithStats(markers, labels, stats, centroids, 4, CV_32S); // 10-15 ms

        //        std::vector<cv::Rect> testvec;
        //        testvec.resize(marker_count);
        //        std::vector<cv::Rect> bboxes;
        //        bool showCrosshair = true;
        //        bool fromCenter = false;

        // cv::selectROIs("MultiTracker", markers, bboxes, showCrosshair, fromCenter);



        //  return marker_count; // 25-30 ms
#if defined(IMSHOW_SEG) || defined(IMSHOW_CAP)
        for (int i = 0; i < marker_count; i++)
        {


            //        testvec[i] = rect;
            //           testvec.push_back(rect);


            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            cv::Rect rect = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                                     stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));

            cv::rectangle(markers, rect, cv::Scalar(0), 1);
            cv::putText(markers, std::to_string((int)(mm_per_pix*rect.width))+" x "+std::to_string((int)(mm_per_pix*rect.height)),
                        cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
            if (i > 0)
            {
                std::cout << "Label " << i << " - pos: (" << rect.x << "," << rect.y << ") center: (" << cx << "," << cy
                          << ") size: " << rect.width << " x " << rect.height << " area: " << area << std::endl;
            }
            else {
                std::cout << "Background - pos: (" << rect.x << "," << rect.y << ") center: (" << cx << "," << cy
                          << ") size: " << rect.width << " x " << rect.height << " area: " << area << std::endl;
            }
        }
#endif
        return marker_count;
    }

    void tracking_thread_func()
    {

    }


public:
    //     Rs2Device(rs2::device &dev, std::mutex* mutex, rs2::points* buffer, size_t& write_idx_ref);
    OcvDevice(int idx, std::mutex* mutex, cv::Mat* buffer, size_t& write_idx_ref)
    {
        m_cam_idx = idx;
        m_mutex_ref = mutex;
        m_ocv_mat_buf_ref = buffer;
        m_mat_write_idx_ref = &write_idx_ref;

        std::cout << "New OpenCV device instance: " << m_cam_idx << " buffer: " << m_ocv_mat_buf_ref << " mutex: " << m_mutex_ref << std::endl;

        m_capture_buf = new cv::Mat*[CAP_BUF_SIZE];
        for (int i=0; i<CAP_BUF_SIZE; i++)
        {
            m_capture_buf[i] = new cv::Mat[2];
        }
        //        cv::Mat* twodim [MARKER_BUF_SIZE][2];
        m_capture_write_idx = 0;
        m_marker_read_idx = 0;


        m_capture_thread = std::thread(&OcvDevice::capture_thread_func, this);
        //  m_segmentation_thread = std::thread(&OcvDevice::segmentation_thread_func, this);

    }
    ~OcvDevice()
    {
        for (int i=0; i<CAP_BUF_SIZE; i++)
        {
            delete[] m_capture_buf[i];
        }
        delete[] m_capture_buf;
    }
};

#endif // OCVDEVICE_H

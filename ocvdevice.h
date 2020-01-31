#ifndef OCVDEVICE_H
#define OCVDEVICE_H

#include "format.h"
#include "customtypes.h"

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

    cv::VideoCapture* m_capture;

    std::vector<std::string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

    // internal buffers and mutexes
    cv::Mat** m_capture_buf;
    size_t m_capture_write_idx, m_marker_read_idx;
    std::mutex m_marker_mutex;

    tracked_object** m_tracked_objects_buf;
    size_t m_objects_write_idx, m_objects_read_idx;
    std::mutex m_objects_mutex;

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
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After preprocessing: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 3); // 8-14 ms
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5)), cv::Point(-1,-1), 3); // 10 - 16 ms
        cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 5); // 10-12 ms

        cv::threshold(background, background, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        // distance transformation
        cv::distanceTransform(tmp, tmp, cv::DIST_L2, 3);
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After distance transformation: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
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
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After normalize thresh: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        // segmenting contours
        std::vector<std::vector<cv::Point>> contours;
        tmp.convertTo(tmp, CV_8U);
        findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++)
            drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After find&draw contours: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
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
        std::cout << "OpenCV segmentation thread started # " << std::this_thread::get_id() << std::endl;

        //        while (m_marker_read_idx == m_capture_write_idx) {
        //            std::this_thread::sleep_for(std::chrono::seconds(1));
        //        }

#ifdef IMSHOW_SEG
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {
            if (m_marker_read_idx != m_capture_write_idx)
            {
                m_start_time_seg = std::chrono::steady_clock::now();

                // Get segmentation markers
                m_marker_mutex.lock();
                cv::Mat color = m_capture_buf[m_marker_read_idx][0];
                cv::Mat marker = m_capture_buf[m_marker_read_idx++][1];
                if (m_marker_read_idx == POINT_BUF_SIZE)
                    m_marker_read_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased read index: " << m_marker_read_idx << " size0 " << color.size() << " type0 " << color.type() << " size1 "
                          << marker.size() << " type1 " << marker.type() << std::endl;
#endif
                m_marker_mutex.unlock();

#if (VERBOSE > 1)
                std::cout << "Before watershed: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
                // Segmenting by watershed algorithm
                marker.convertTo(marker, CV_32S);
                cv::watershed(color, marker); // 6-20 ms
                marker.convertTo(marker, CV_8U);
#if (VERBOSE > 1)
                std::cout << "After watershed: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
                // Get objects position
                cv::Mat labels, stats, centroids;
                int marker_count = cv::connectedComponentsWithStats(marker, labels, stats, centroids, 4, CV_32S); // 10-15 ms
                tracked_object* obj_ptr_arr = new tracked_object[marker_count];

                for (int i = 0; i < marker_count; i++)
                {
                    obj_ptr_arr[i].x = stats.at<int>(i, cv::CC_STAT_LEFT);
                    obj_ptr_arr[i].y = stats.at<int>(i, cv::CC_STAT_TOP);
                    obj_ptr_arr[i].w = stats.at<int>(i, cv::CC_STAT_WIDTH);
                    obj_ptr_arr[i].h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
                    obj_ptr_arr[i].area = stats.at<int>(i, cv::CC_STAT_AREA);
                    obj_ptr_arr[i].cx = centroids.at<double>(i, 0);
                    obj_ptr_arr[i].cy = centroids.at<double>(i, 1);
#if (VERBOSE > 2)
                    if (i > 0)
                        std::cout << "Label " << i << " - pos: (" << obj_ptr_arr[i].x << "," << obj_ptr_arr[i].y << ") center: (" << obj_ptr_arr[i].cx << ","
                                  << obj_ptr_arr[i].cy << ") size: " << obj_ptr_arr[i].w << " x " << obj_ptr_arr[i].h << " area: " << obj_ptr_arr[i].area << std::endl;
                    else
                        std::cout << "Background - pos: (" << obj_ptr_arr[i].x << "," << obj_ptr_arr[i].y << ") center: (" << obj_ptr_arr[i].cx << ","
                                  << obj_ptr_arr[i].cy << ") size: " << obj_ptr_arr[i].w << " x " << obj_ptr_arr[i].h << " area: " << obj_ptr_arr[i].area << std::endl;
#endif
#if defined(IMSHOW_SEG) || defined(IMSHOW_CAP)
                    cv::rectangle(marker, cv::Rect(obj_ptr_arr[i].x, obj_ptr_arr[i].y, obj_ptr_arr[i].w, obj_ptr_arr[i].h), cv::Scalar(0), 1);
                    cv::putText(marker, std::to_string((int)(mm_per_pix * obj_ptr_arr[i].w))+" x "+std::to_string((int)(mm_per_pix * obj_ptr_arr[i].h)),
                                cv::Point(obj_ptr_arr[i].x, obj_ptr_arr[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
#endif
                }

                // Write
                m_objects_mutex.lock();
                m_tracked_objects_buf[m_objects_write_idx++] = obj_ptr_arr;
                if (m_objects_write_idx == TOBJ_BUF_SIZE)
                    m_objects_write_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased write index: " << m_objects_write_idx << " size array of "
                          << marker_count << " structs" << std::endl;
#endif
                m_objects_mutex.unlock();

#ifdef IMSHOW_SEG
                cv::imshow("Segmented image", marker*(255/elements));
#endif

                std::cout << "(OpenCV segmentation) Total took: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;

            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(SEG_DELAY));
            }

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
        m_capture_write_idx = 0;
        m_marker_read_idx = 0;

        m_tracked_objects_buf = new tracked_object*[TOBJ_BUF_SIZE];
        for (int i=0; i<TOBJ_BUF_SIZE; i++)
        {
            m_tracked_objects_buf[i] = new tracked_object;
        }
        m_objects_write_idx = 0;


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

        for (int i=0; i<TOBJ_BUF_SIZE; i++)
        {
            delete[] m_tracked_objects_buf[i];
        }
        delete[] m_tracked_objects_buf;
    }
};

#endif // OCVDEVICE_H

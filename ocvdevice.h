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
    cv::VideoCapture* m_capture;
    const double mm_per_pix = static_cast<double>(REF_SIZE_MM)/static_cast<double>(REF_PIXEL);

    // internal buffers and mutexes
    cv::Mat** m_cvcap_buf;
    size_t m_cvcap_write_idx, m_cvcap_read_idx;
    std::mutex m_marker_mutex;


    // tracked_object** m_tracked_objects_buf;
    TrackedObjects* m_tracked_objects_buf;


    size_t m_objects_write_idx, m_objects_read_idx;
    std::mutex m_objects_mutex;

    // external refs
    std::mutex* m_mutex_ref;
    int m_cam_idx;
    cv::Mat* m_ocv_mat_buf_ref;
    size_t* m_mat_write_idx_ref;

    // threads
    std::thread m_capture_thread;
    std::thread m_segmentation_thread;
    std::thread m_tracking_thread;

    // tracked objects
    //    TrackedObjects m_obj_centroids;
    //    TrackedObjectArr m_in_centroids;
    //    struct tracked_object* m_obj_centroids;
    //    struct tracked_object* m_in_centroids;
    TrackedObjects m_obj_centroids;
    TrackedObjects m_in_centroids;
    int m_max_disappeared = 0;

    // debug variables
#if (VERBOSE > 0)
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_cap;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_seg;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_tra;
#endif

    void capture_thread_func()
    {
        std::cout << "OpenCV capture thread started # " << std::this_thread::get_id() << " device: " << m_cam_idx << std::endl;
        cv::Mat image;
        m_capture = new cv::VideoCapture(m_cam_idx); // cv::CAP_V4L2
        m_capture->set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH_CV);
        m_capture->set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT_CV);
        m_capture->set(cv::CAP_PROP_FPS, FRAME_RATE_CV);
        std::cout << "Capturing with resolution " << m_capture->get(cv::CAP_PROP_FRAME_WIDTH) << " x "
                  << m_capture->get(cv::CAP_PROP_FRAME_HEIGHT) << " at " << m_capture->get(cv::CAP_PROP_FPS) << " fps" << std::endl;

        if (m_capture->isOpened())
        {
            m_segmentation_thread = std::thread(&OcvDevice::segmentation_thread_func, this);
            m_tracking_thread = std::thread(&OcvDevice::tracking_thread_func, this);
        }
#if (IMSHOW_CAP > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {
            // next image
            m_capture->read(image);
#if (VERBOSE > 0)
            m_start_time_cap = std::chrono::steady_clock::now();
#endif
            // calculate markers
            cv::Mat markers = cv::Mat::zeros(image.size(), CV_32S);
#if (IMSHOW_CAP > 0)
            cv::imshow("Input", image);
#endif
            prepareObjectMarkers(image, markers);
            // write to buffer
            m_marker_mutex.lock();
            m_cvcap_buf[m_cvcap_write_idx][0] = image;
            m_cvcap_buf[m_cvcap_write_idx++][1] = markers;
            if (m_cvcap_write_idx == BUF_SIZE_CVCAP)
                m_cvcap_write_idx = 0;
#if (VERBOSE > 0)
            std::cout << "(OpenCV capture) Increased write index: " << m_cvcap_write_idx << " size0 " << image.size() << " type0 " << image.type() << " size1 "
                      << markers.size() << " type1 " << markers.type() << std::endl;
#endif
            m_marker_mutex.unlock();
#if (VERBOSE > 0)
            std::cout << "(OpenCV capture) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                         (std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        }
        std::cout << "Capture thread is exiting" << std::endl;
    }

    void segmentation_thread_func()
    {
        std::cout << "OpenCV segmentation thread started # " << std::this_thread::get_id() << std::endl;
#if (IMSHOW_SEG > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {
            if (m_cvcap_read_idx != m_cvcap_write_idx)
            {
#if (VERBOSE > 0)
                m_start_time_seg = std::chrono::steady_clock::now();
#endif
                // Get segmentation markers
                m_marker_mutex.lock();
                cv::Mat color = m_cvcap_buf[m_cvcap_read_idx][0];
                cv::Mat marker = m_cvcap_buf[m_cvcap_read_idx++][1];
                if (m_cvcap_read_idx == BUF_SIZE_POINTS)
                    m_cvcap_read_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased read index: " << m_cvcap_read_idx << " size0 " << color.size() << " type0 " << color.type() << " size1 "
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
                // Get object positions
                cv::Mat labels, stats, centroids;
                int marker_count = cv::connectedComponentsWithStats(marker, labels, stats, centroids, 4, CV_32S); // 10-15 ms




                // heap
                std::vector<tracked_object> *objects = new std::vector<tracked_object>();
                objects->reserve(marker_count);

                //  tracked_object* objects_ptr = new tracked_object[marker_count];





                for (int i = 0; i < marker_count; i++)
                {
                    objects->push_back({stats.at<int>(i, cv::CC_STAT_LEFT),stats.at<int>(i, cv::CC_STAT_TOP),stats.at<int>(i, cv::CC_STAT_WIDTH),
                                        stats.at<int>(i, cv::CC_STAT_HEIGHT),stats.at<int>(i, cv::CC_STAT_AREA),centroids.at<double>(i, 0),centroids.at<double>(i, 1), -1, 0});



                    //                    objects_ptr[i].x = stats.at<int>(i, cv::CC_STAT_LEFT);
                    //                    objects_ptr[i].y = stats.at<int>(i, cv::CC_STAT_TOP);
                    //                    objects_ptr[i].w = stats.at<int>(i, cv::CC_STAT_WIDTH);
                    //                    objects_ptr[i].h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
                    //                    objects_ptr[i].area = stats.at<int>(i, cv::CC_STAT_AREA);
                    //                    objects_ptr[i].cx = centroids.at<double>(i, 0);
                    //                    objects_ptr[i].cy = centroids.at<double>(i, 1);
                    //#if (IMSHOW_SEG > 0) || (IMSHOW_CAP > 0)
                    //                    cv::rectangle(marker, cv::Rect(objects_ptr[i].x, objects_ptr[i].y, objects_ptr[i].w, objects_ptr[i].h), cv::Scalar(0), 1);
                    //                    cv::putText(marker, std::to_string((int)(mm_per_pix * objects_ptr[i].w))+" x "+std::to_string((int)(mm_per_pix * objects_ptr[i].h)),
                    //                                cv::Point(objects_ptr[i].x, objects_ptr[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                    //#endif
                }
#if (VERBOSE > 0)
                for (int i = 0; i < (int)objects->size(); i++) {
                    if (i > 0)
                        std::cout << "Label " << i << " - pos: (" << objects->at(i).x << "," << objects->at(i).y << ") center: (" << objects->at(i).cx << ","
                                  << objects->at(i).cy << ") size: " << objects->at(i).w << " x " << objects->at(i).h << " area: " << objects->at(i).area << std::endl;
                    else
                        std::cout << "Background - pos: (" << objects->at(i).x << "," << objects->at(i).y << ") center: (" << objects->at(i).cx << ","
                                  << objects->at(i).cy << ") size: " << objects->at(i).w << " x " << objects->at(i).h << " area: " << objects->at(i).area << std::endl;
                }
#endif



                // Write tracked objects
                m_objects_mutex.lock();



                // m_tracked_objects_buf[m_objects_write_idx++] = TrackedObject(new std::vector<tracked_object>(objects));
                //  m_tracked_objects_buf[m_objects_write_idx++] = TrackedObjectArr(marker_count, objects_ptr);
                //  m_tracked_objects_buf[m_objects_write_idx++] = objects->data();
                m_tracked_objects_buf[m_objects_write_idx++] = TrackedObjects(objects->data(), marker_count);



                if (m_objects_write_idx == BUF_SIZE_TOBJ)
                    m_objects_write_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased write index: " << m_objects_write_idx << " size array of "
                          << marker_count << std::endl;
#endif
                m_objects_mutex.unlock();



#if (IMSHOW_SEG > 0)
                cv::imshow("Segmented image", marker*(255/marker_count));
#endif
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_SEGM));
        }

        std::cout << "Segmentation thread is exiting" << std::endl;
    }

    void tracking_thread_func()
    {
        std::cout << "OpenCV tracking thread started # " << std::this_thread::get_id() << std::endl;

        //   cv::Mat curr_centroids, object_centroids = cv::Mat::zeros(0, 2, CV_64F);
        //  cv::Mat prev_centroids;
        //  return;

        m_obj_centroids = new tracked_object;

        while( m_capture->isOpened() )
        {
            //  cv::Mat centroids, prev_centroids;

            if (m_objects_read_idx != m_objects_write_idx)
            {
#if (VERBOSE > 0)
                m_start_time_tra = std::chrono::steady_clock::now();
#endif
                // Read tracked objects
                m_objects_mutex.lock();
                m_in_centroids = m_tracked_objects_buf[m_objects_read_idx++]; // delete ptr later
                if (m_objects_read_idx == BUF_SIZE_POINTS)
                    m_objects_read_idx = 0;


                //                std::vector<tracked_object> *obj_ptr = m_obj_centroids.getTrackedObjects();
                //                tracked_object *inp_ptr = m_in_centroids.getTrackedObjects();
#if (VERBOSE > 0)
                //  std::cout << "(OpenCV tracking) Increased read index: " << m_objects_read_idx << " size " << m_in_centroids.getSize() << std::endl;
                std::cout << "(OpenCV tracking) Increased read index: " << m_objects_read_idx << " size " << m_in_centroids.getSize() << std::endl;
#endif
                m_objects_mutex.unlock();










                //                if (obj_ptr->empty())
                //                {
                //                    std::cout << "DEBUGUGUGUGUGU" << std::endl;
                //                    tracked_object test;
                //                    test.x = 0;
                //                    obj_ptr->push_back(test);
                //                    std::cout << obj_ptr->size() << std::endl;
                //                }
                //                else std::cout << "NOT DEBUGUGUGUGUGU" << std::endl;





                // Track objects
                tracked_object* inp_ptr = m_in_centroids.getTrackedObjectsPtr();


#if (VERBOSE > 0)
                for (int i = 0; i < m_in_centroids.getSize(); i++) {
                    if (i > 0)
                        std::cout << "Label " << i << " - pos: (" << inp_ptr[i].x << "," << inp_ptr[i].y << ") center: (" << inp_ptr[i].cx << ","
                                  << inp_ptr[i].cy << ") size: " << inp_ptr[i].w << " x " << inp_ptr[i].h << " area: " << inp_ptr[i].area << std::endl;
                    else
                        std::cout << "Background - pos: (" << inp_ptr[i].x << "," << inp_ptr[i].y << ") center: (" << inp_ptr[i].cx << ","
                                  << inp_ptr[i].cy << ") size: " << inp_ptr[i].w << " x " << inp_ptr[i].h << " area: " << inp_ptr[i].area << std::endl;
                }
#endif



#ifdef Geci
                double min, max;
                cv::Point minLoc, maxLoc;

                // Compute centroid distances

                std::vector<std::tuple<double,int,int>> min_distances, max_distances; // value, row, col
                cv::Mat distances = centroidDistances(obj_ptr, &m_in_centroids);



#if (VERBOSE > 0)
                std::cout << "distance matrix (object x input):" << std::endl << distances << std::endl;
#endif
                // Extract min and max distances

                if ( !distances.empty() )
                {
                    // find min/max distances of known objects (rows)
                    for (int i = 0; i < distances.rows; i++)
                    {
                        cv::Mat row = distances.row(i);
                        cv::minMaxLoc(row, &min, &max, &minLoc, &maxLoc);
                        minLoc.y = i;
                        min_distances.push_back(std::make_tuple(min, i, minLoc.x));
                        max_distances.push_back(std::make_tuple(max, i, maxLoc.x));
#if (VERBOSE > 0)
                        std::cout << "min " << min << " at row,col: (" << i << "," << minLoc.x << ")" << std::endl;
                        std::cout << "max " << max << " at row,col: (" << i << "," << maxLoc.x << ")" << std::endl;
#endif
                    }
                    // sort found distances
                    std::sort(min_distances.begin(), min_distances.end());
                    std::sort(max_distances.begin(), max_distances.end(), std::greater<std::tuple<double,int,int>>());
#if (VERBOSE > 0)
                    std::cout<<"mins"<<std::endl;
                    for (int i=0;i<(int)min_distances.size();i++) {
                        std::cout << std::get<0>(min_distances.at(i)) << std::endl;
                    }
                    std::cout<<"maxs"<<std::endl;
                    for (int i=0;i<(int)max_distances.size();i++) {
                        std::cout << std::get<0>(max_distances.at(i)) << std::endl;
                    }
#endif
                }


                // deregister lost objects

                if (m_obj_centroids.getSize() > m_in_centroids.getSize())
                {
                    // increment deregister counters
                    std::vector<int> lost_objects;

                    if (m_in_centroids.getSize() == 0)
                    {
                        for (int i = 0; i < (int)obj_ptr->size(); i++)
                        {
                            obj_ptr->at(i).lost_ctr++;
                            if (obj_ptr->at(i).lost_ctr >= m_max_disappeared)
                            {
                                lost_objects.push_back(i);
                                std::cout << "lost object:" << lost_objects.back() << std::endl;
                            }
                        }
                    }
                    else {
                        int to_delete = obj_ptr->size() - m_in_centroids.getSize();
                        for (int i = 0; i < to_delete; i++)
                        {
                            int obj_idx = std::get<1>(max_distances[i])/*row*/;
                            obj_ptr->at(obj_idx).lost_ctr++;
                            if (obj_ptr->at(obj_idx).lost_ctr >= m_max_disappeared)
                            {
                                lost_objects.push_back(obj_idx);
                                std::cout << "lost object:" << lost_objects.back() << std::endl;
                            }
                        }
                    }
                    // deregister marked objects
                    if (!lost_objects.empty()) {
                        deregisterObjects(&lost_objects, &m_obj_centroids);
                        std::cout << std::endl;
                        std::cout << "Deregistered lost object centroids:" << std::endl;
                        for (int i = 0; i < m_obj_centroids.getSize(); i++){
                            std::cout << " #" << obj_ptr->at(i).unique_id << " (" << obj_ptr->at(i).cx << "," << obj_ptr->at(i).cy << ")" << std::endl;
                        }

                        // recall the whole function here?
                    }
                }
#endif // Geci





#if (VERBOSE > 0)
                std::cout << "(OpenCV tracking) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_tra).count() << " ms" << std::endl;
#endif

#if (IMSHOW_TRA > 0)
#endif

            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TRAC));
        }
    }

    void deregisterObjects(std::vector<int> *obj_idxs, TrackedObjects *obj_centr)
    {
        std::sort(obj_idxs->begin(), obj_idxs->end(), std::greater<int>());
        auto obj_ptr = obj_centr->getTrackedObjectsPtr();

        //        for (int i = 0; i < (int)obj_idxs->size(); i++) {
        //            int idx = obj_idxs->at(i);
        //            std::cout << "deleting object idx " << idx << " #" << obj_ptr->at(idx).unique_id << std::endl;
        //            obj_ptr->erase(obj_ptr->begin() + idx);
        //        }
    }

    cv::Mat centroidDistances(std::vector<tracked_object> *obj_centr, TrackedObjectArr *inp_centr)
    {
        cv::Mat result = cv::Mat::zeros(obj_centr->size(), inp_centr->getSize(), CV_64F);
        for (int i = 0, total = (int)obj_centr->size(); i < total; ++i)
        {
            double x_o = obj_centr->at(i).cx;
            double y_o = obj_centr->at(i).cy;
            for (int j = 0, total = (int)inp_centr->getSize(); j < total; ++j)
            {
                auto inp_ptr = inp_centr->getTrackedObjects()[i];
                double dx = inp_ptr.cx - x_o;
                double dy = inp_ptr.cy - y_o;
                result.at<double>(i, j) = std::sqrt(dx * dx + dy * dy);
            }
        }
        return result;
    }

    //    cv::Mat computeDistances(cv::Mat& from, cv::Mat& to)
    //    {
    //        // awaiting cv::Mats with n x 2 (x,y) dimension
    //        cv::Mat result = cv::Mat::zeros(from.rows, to.rows, CV_64F);
    //        for (int i = 0; i < from.rows; i++)
    //        {
    //            double x, y, x_t ,y_t;
    //            x = from.at<double>(i, 0);
    //            y = from.at<double>(i, 1);
    //            for (int j = 0; j < to.rows; j++)
    //            {
    //                x_t = to.at<double>(j, 0);
    //                y_t = to.at<double>(j, 1);
    //                double dx = x_t - x;
    //                double dy = y_t - y;
    //                result.at<double>(i, j) = std::sqrt(dx * dx + dy * dy);
    //            }
    //        }
    //        return result;
    //    }

public:
    OcvDevice(int idx, std::mutex* mutex, cv::Mat* buffer, size_t& write_idx_ref)
    {
        m_cam_idx = idx;
        m_mutex_ref = mutex;
        m_ocv_mat_buf_ref = buffer;
        m_mat_write_idx_ref = &write_idx_ref;

        std::cout << "New OpenCV device instance: " << m_cam_idx << " buffer: " << m_ocv_mat_buf_ref << " mutex: " << m_mutex_ref << std::endl;

        // initialize buffers
        m_cvcap_buf = new cv::Mat*[BUF_SIZE_CVCAP];
        for (int i=0; i<BUF_SIZE_CVCAP; i++)
        {
            m_cvcap_buf[i] = new cv::Mat[2];
        }
        m_cvcap_write_idx = m_cvcap_read_idx = 0;

        //   m_tracked_objects_buf = new TrackedObjectArr[BUF_SIZE_TOBJ]();
        // m_tracked_objects_buf = new tracked_object*[BUF_SIZE_TOBJ];

        m_tracked_objects_buf = new TrackedObjects[BUF_SIZE_TOBJ];
        m_objects_write_idx = m_objects_read_idx = 0;
        // Start capturing, which starts the following threads
        m_capture_thread = std::thread(&OcvDevice::capture_thread_func, this);

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
#if (IMSHOW_CAP > 0)
        cv::imshow("Watershed markers", tmp*(255/contours.size())-10);
#endif
        tmp.convertTo(markers, CV_32S);
    }

    ~OcvDevice()
    {
        for (int i=0; i<BUF_SIZE_CVCAP; i++)
        {
            delete[] m_cvcap_buf[i];
        }
        delete[] m_cvcap_buf;

        delete[] m_tracked_objects_buf;
    }
};

#endif // OCVDEVICE_H

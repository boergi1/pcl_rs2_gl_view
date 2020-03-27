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

    shared_data_t* m_tracked_objects_buf = new shared_data_t();


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
    std::thread m_imshow_thread;

    tracked_object_t *m_input_objects;
    std::vector<tracked_object_t> *m_tracked_objects;
    int m_max_disappeared = 5;
    uint16_t m_id_ctr = 0;

    // current mats for imshow
    cv::Mat m_current_mat_seg, m_current_mat_tra;
    std::mutex m_mat_seg_mtx, m_mat_tra_mtx;

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
#if (IMSHOW > 0 && VERBOSE > 0)
            m_imshow_thread = std::thread(&OcvDevice::imshow_thread_func, this);
#endif
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
        tracked_object_t* tmp_tr_obj;
        while( m_capture->isOpened() )
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
                tmp_tr_obj = new tracked_object_t[marker_count]();

                // Fill array with segmented objects
                for (int i = 0; i < marker_count; i++)
                {
                    tmp_tr_obj[i].x = stats.at<int>(i, cv::CC_STAT_LEFT);
                    tmp_tr_obj[i].y = stats.at<int>(i, cv::CC_STAT_TOP);
                    tmp_tr_obj[i].w = stats.at<int>(i, cv::CC_STAT_WIDTH);
                    tmp_tr_obj[i].h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
                    tmp_tr_obj[i].area = stats.at<int>(i, cv::CC_STAT_AREA);
                    tmp_tr_obj[i].cx = centroids.at<double>(i, 0);
                    tmp_tr_obj[i].cy = centroids.at<double>(i, 1);
                    tmp_tr_obj[i].unique_id = -1;
                    tmp_tr_obj[i].lost_ctr = 0;
#if (IMSHOW > 0 && VERBOSE > 0)
                    // Draw markers on cv window
                    cv::rectangle(marker, cv::Rect(tmp_tr_obj[i].x, tmp_tr_obj[i].y,
                                                   tmp_tr_obj[i].w, tmp_tr_obj[i].h), cv::Scalar(0), 1);
                    cv::putText(marker, std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].w))
                                + " x "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].h)),
                                cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y),
                                cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                    cv::circle(marker, cv::Point(roundToInt(tmp_tr_obj[i].cx), roundToInt(tmp_tr_obj[i].cy)),
                               10, cv::Scalar(255), -1);
#endif
                }
                // Write segmented objects
                m_tracked_objects_buf->mutex.lock();
                m_tracked_objects_buf->tobj_ptr_queue.push(tmp_tr_obj);
                m_tracked_objects_buf->arr_size_queue.push(static_cast<size_t>(marker_count));
                m_tracked_objects_buf->mutex.unlock();
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased queue size: " << m_tracked_objects_buf->tobj_ptr_queue.size() << " size: array of "
                          << marker_count << std::endl;
#endif
#if (IMSHOW > 0 && VERBOSE > 0)
                cv::putText(marker, std::to_string(marker_count),
                            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                m_mat_seg_mtx.lock();
                m_current_mat_seg = marker*(255/marker_count);
                m_mat_seg_mtx.unlock();
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
        m_tracked_objects = new std::vector<tracked_object_t>();
#if (IMSHOW_TRA > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
#else
        while( m_capture->isOpened() )
#endif
        {
            if ( !m_tracked_objects_buf->tobj_ptr_queue.empty() )
            {
                size_t input_objects_size, tracked_objects_size;
                cv::Mat distances;
                std::vector<size_t> lost_objects;
                double min, max;
                cv::Point minLoc, maxLoc;
                std::vector<std::tuple<double,int,int>> row_min_distances, col_min_distances, max_distances; // value, row, col
#if (VERBOSE > 0)
                m_start_time_tra = std::chrono::steady_clock::now();
#endif
                // Read segmented input objects
                m_tracked_objects_buf->mutex.lock();
                input_objects_size = m_tracked_objects_buf->arr_size_queue.front();
                m_tracked_objects_buf->arr_size_queue.pop();
                m_input_objects = new tracked_object_t[input_objects_size]();
                m_input_objects = m_tracked_objects_buf->tobj_ptr_queue.front();
                m_tracked_objects_buf->tobj_ptr_queue.pop();
                m_tracked_objects_buf->mutex.unlock();
                tracked_objects_size = m_tracked_objects->size();
#if (VERBOSE > 0)
                std::cout << "(OpenCV tracking) Reduced queue size: " << m_tracked_objects_buf->tobj_ptr_queue.size() << " size: array of " << input_objects_size << std::endl;
#endif
#if (VERBOSE > 1)
                std::cout << "Current object centroids: " << tracked_objects_size << std::endl;
                for (size_t i = 0; i < tracked_objects_size; i++){
                    std::cout << "idx "<< i <<" #" << m_tracked_objects->at(i).unique_id << " (" << m_tracked_objects->at(i).cx
                              << "," << m_tracked_objects->at(i).cy << ")" << std::endl;
                }
                std::cout << "Current input centroids: " << input_objects_size << std::endl;
                for (size_t i = 0; i < input_objects_size; i++) {
                    std::cout << "idx "<< i <<" #" << m_input_objects[i].unique_id << " (" << m_input_objects[i].cx
                              << "," << m_input_objects[i].cy << ")" << std::endl;
                }
#endif
                // Track objects
                if (tracked_objects_size == 0 && input_objects_size == 0) // ||
                {
                    // Do nothing
                    std::cout << "tracking thread break" << std::endl;
                    break;
                }
                else if (tracked_objects_size == 0 && input_objects_size > 0)
                {
                    // Register all input objects
                    for (size_t i = 0; i < input_objects_size; i++)
                    {
                        auto tmp_obj = m_input_objects[i];
                        tmp_obj.unique_id = m_id_ctr++;
                        m_tracked_objects->push_back(tmp_obj);
                    }
                }
                else if (tracked_objects_size > 0 && input_objects_size == 0)
                {
                    // Deregister all tracked objects
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        m_tracked_objects->at(i).lost_ctr++;
                        if (m_tracked_objects->at(i).lost_ctr >= m_max_disappeared)
                            lost_objects.push_back(i);
                    }
                }
                else {
                    // Compute centroid distances
                    distances = centroidDistances(m_tracked_objects, m_input_objects, input_objects_size);
#if (VERBOSE > 2)
                    std::cout << "Distance matrix ( " << tracked_objects_size << " x " << input_objects_size << " ):"<< std::endl << distances << std::endl;
#endif
                    // Find min/max distances of known objects (rows)
                    for (int i = 0; i < static_cast<int>(tracked_objects_size); i++)
                    {
                        cv::minMaxLoc(distances.row(i), &min, &max, &minLoc, &maxLoc);
                        row_min_distances.push_back(std::make_tuple(min, i, minLoc.x));
                        max_distances.push_back(std::make_tuple(max, i, maxLoc.x));
                    }

                    for (int i = 0; i < static_cast<int>(input_objects_size); i++)
                    {
                        cv::minMaxLoc(distances.col(i), &min, &max, &minLoc, &maxLoc);
                        col_min_distances.push_back(std::make_tuple(min, minLoc.y, i));
                    }

                    // Sort extreme values
                    std::sort(row_min_distances.begin(), row_min_distances.end());
                    std::sort(col_min_distances.begin(), col_min_distances.end());
                    std::sort(max_distances.begin(), max_distances.end(), std::greater<std::tuple<double,int,int>>());



#if (VERBOSE > 1)
                    std::cout<<"Minima rows: "<< row_min_distances.size() <<std::endl;
                    for (size_t i=0;i<row_min_distances.size();i++) {
                        std::cout << std::get<0>(row_min_distances.at(i)) << " at row,col: (" << std::get<1>(row_min_distances.at(i)) << ","
                                  << std::get<2>(row_min_distances.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Minima cols: "<< col_min_distances.size() <<std::endl;
                    for (size_t i=0;i<col_min_distances.size();i++) {
                        std::cout << std::get<0>(col_min_distances.at(i)) << " at row,col: (" << std::get<1>(col_min_distances.at(i)) << ","
                                  << std::get<2>(col_min_distances.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Maxima: "<< max_distances.size() << std::endl;
                    for (size_t i=0;i<max_distances.size();i++) {
                        std::cout << std::get<0>(max_distances.at(i)) << " at row,col: (" << std::get<1>(max_distances.at(i)) << ","
                                  << std::get<2>(max_distances.at(i)) << ")" << std::endl;
                    }
#endif
                    //                    // Update current objects
                    //                    for (size_t i = 0; i < tracked_objects_size; i++)
                    //                    {
                    //                        size_t obj_idx = static_cast<size_t>(std::get<1>(row_min_distances[i]))/*row*/;
                    //                        size_t inp_idx = static_cast<size_t>(std::get<2>(row_min_distances[i]))/*col*/;
                    //                        auto tmp_obj = m_input_objects[inp_idx];
                    //                        tmp_obj.unique_id = m_tracked_objects->at(obj_idx).unique_id;
                    //                        tmp_obj.lost_ctr = m_tracked_objects->at(obj_idx).lost_ctr;
                    //                        m_tracked_objects->at(obj_idx) = tmp_obj;
                    //#if (VERBOSE > 1)
                    //                        std::cout << "Updated object: idx " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                    //                                  << " x " << m_tracked_objects->at(obj_idx).cx << " y "
                    //                                  << m_tracked_objects->at(obj_idx).cy << " lost " << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;
                    //#endif
                    //                    }

                    if (tracked_objects_size > input_objects_size)
                    {

                        // Deregister specific objects
                        size_t to_delete = tracked_objects_size - input_objects_size;
                        for (size_t i = 0; i < to_delete; i++)
                        {
                            size_t obj_idx = static_cast<size_t>(std::get<1>(max_distances[i]))/*row*/;
                            if ( ++m_tracked_objects->at(obj_idx).lost_ctr >= m_max_disappeared)
                            {
                                lost_objects.push_back(obj_idx);
#if (VERBOSE > 1)
                                std::cout << "Deleting object: idx " << lost_objects.back() << " #" << m_tracked_objects->at(obj_idx).unique_id
                                          << " x " << m_tracked_objects->at(obj_idx).cx
                                          << " y " << m_tracked_objects->at(obj_idx).cy << std::endl;
#endif
                            }
#if (VERBOSE > 1)
                            else {
                                std::cout << "Lost object: idx " << obj_idx << " #"
                                          << m_tracked_objects->at(obj_idx).unique_id << " lost "
                                          << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;

                            }
#endif
                        }
                    }
                    else if (tracked_objects_size < input_objects_size)
                    {
                        // Register specific objects
                        for (size_t i=0; i<col_min_distances.size(); i++) {
                            bool new_object = true;
                            for (size_t j=0; j<row_min_distances.size(); j++) {
                                if ( std::get<2>(col_min_distances[i]) == std::get<2>(row_min_distances[j]))
                                    new_object = false;
                            }
                            if (new_object)
                            {
                                size_t inp_idx = static_cast<size_t>(std::get<2>(col_min_distances[i]));
                                auto tmp_obj = m_input_objects[inp_idx];
                                tmp_obj.unique_id = m_id_ctr++;
#if (VERBOSE >= 10)
                                // Error check if same point already exists
                                for (size_t j = 0; j < tracked_objects_size; j++)
                                {
                                    if ( areSame(tmp_obj.x, m_tracked_objects->at(j).x) && areSame(tmp_obj.y, m_tracked_objects->at(j).y) )
                                    {
                                        std::cerr << "ERROR Point match: idx "<< j << " #"
                                                  << m_tracked_objects->at(j).unique_id << std::endl;
                                    }
                                }
#endif
                                m_tracked_objects->push_back(tmp_obj);
#if (VERBOSE > 1)
                                std::cout << "New object: idx " << m_tracked_objects->size()-1 << " #" << m_tracked_objects->back().unique_id
                                          << " x " << m_tracked_objects->back().cx
                                          << " y " << m_tracked_objects->back().cy << std::endl;
#endif
                            }

                        }
                    }

                    //                Current object centroids: 3
                    //                idx 0 #0 (620.743,360.408)+
                    //                idx 1 #1 (644.825,358.997)+
                    //                idx 2 #2 (560.888,367.001)+
                    //                Current input centroids: 3
                    //                idx 0 #-1 (634.649,357.521)
                    //                idx 1 #-1 (640.836,359.964)++
                    //                idx 2 #-1 (620.714,352.996)+
                    //                Minima rows: 3
                    //                4.10396 at row,col: (1,1) <-
                    //                7.41184 at row,col: (0,2) <-
                    //                61.4432 at row,col: (2,2) --
                    //                Minima cols: 3
                    //                10.2826 at row,col: (1,0) <-
                    //                4.10396 at row,col: (1,1) ==
                    //                7.41184 at row,col: (0,2) ==

                    for (size_t i=0; i<col_min_distances.size(); i++) {
                        //                            bool new_object = true;
                        for (size_t j=0; j<row_min_distances.size(); j++) {

                            //                                if ( std::get<2>(col_min_distances[i]) == std::get<2>(row_min_distances[j]))
                            //                                    new_object = false;
                            //                            }
                            //                            if (new_object)
                            //                            {
                            //                                size_t inp_idx = static_cast<size_t>(std::get<2>(col_min_distances[i]));
                            //                                auto tmp_obj = m_input_objects[inp_idx];
                            //                                tmp_obj.unique_id = m_id_ctr++;
                            //                                m_tracked_objects->push_back(tmp_obj);
                            //#if (VERBOSE > 1)
                            //                                std::cout << "New object: idx " << m_tracked_objects->size()-1 << " #" << m_tracked_objects->back().unique_id
                            //                                          << " x " << m_tracked_objects->back().cx
                            //                                          << " y " << m_tracked_objects->back().cy << std::endl;
                            //#endif
                        }
                    }





                    // Update current objects
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        size_t obj_idx = static_cast<size_t>(std::get<1>(row_min_distances[i]))/*row*/;
                        size_t inp_idx = static_cast<size_t>(std::get<2>(row_min_distances[i]))/*col*/;
                        auto tmp_obj = m_input_objects[inp_idx];
                        tmp_obj.unique_id = m_tracked_objects->at(obj_idx).unique_id;
                        tmp_obj.lost_ctr = m_tracked_objects->at(obj_idx).lost_ctr;
                        m_tracked_objects->at(obj_idx) = tmp_obj;
#if (VERBOSE > 1)
                        std::cout << "Updated object: idx " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                                  << " x " << m_tracked_objects->at(obj_idx).cx << " y "
                                  << m_tracked_objects->at(obj_idx).cy << " lost " << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;
#endif
                    }
                }

#if (VERBOSE >= 0)
                // Error check if same point already exists
                for (size_t i=0; i<tracked_objects_size; i++) {
                    auto tmp_obj = m_tracked_objects->at(i);
                    for (size_t j = 0; j < tracked_objects_size; j++) {
                        if ( (i != j) && areSame(tmp_obj.cx, m_tracked_objects->at(j).cx)
                             && areSame(tmp_obj.cy, m_tracked_objects->at(j).cy) )
                            std::cerr << "ERROR Point match: obj idx "<< j << " #"
                                      << m_tracked_objects->at(j).unique_id << " ("
                                      << m_tracked_objects->at(j).cx << ","
                                      << m_tracked_objects->at(j).cy << ") " << std::endl;
                    }
                }
#endif
                // Deregister lost objects
                if ( !lost_objects.empty() )
                {
                    deregisterObjects(&lost_objects, m_tracked_objects);
                    tracked_objects_size = m_tracked_objects->size();
                }




                //                Current object centroids: 5
                //                idx 0 #0 (540.018,355.054)
                //                idx 1 #1 (666.886,359.583)
                //                idx 2 #2 (381.572,32.1588)
                //                idx 3 #3 (135.066,441.903)
                //                idx 4 #4 (110.48,377.865)

                //                Current input centroids: 4
                //                idx 0 #-1 (550.321,353.653)
                //                idx 1 #-1 (665.875,359.479)
                //                idx 2 #-1 (392.661,30.2895)
                //                idx 3 #-1 (146.059,433.473)

                //                Updated object: idx 1 #1 x 665.875 y 359.479 lost 0
                //                Updated object: idx 0 #0 x 550.321 y 353.653 lost 0
                //                Updated object: idx 2 #2 x 392.661 y 30.2895 lost 0
                //                Updated object: idx 3 #3 x 146.059 y 433.473 lost 0
                //                Updated object: idx 4 #4 x 146.059 y 433.473 lost 0

                //                Lost object: idx 4 #4 lost 1

                //                ERROR Point match: obj idx 4 #4 (146.059,433.473)
                //                ERROR Point match: obj idx 3 #3 (146.059,433.473)

                //                Current object centroids: 5
                //                idx 0 #0 (550.321,353.653)
                //                idx 1 #1 (665.875,359.479)
                //                idx 2 #2 (392.661,30.2895)
                //                idx 3 #3 (146.059,433.473)
                //                idx 4 #4 (146.059,433.473)






                //                Current object centroids: 7
                //                idx 0 #0 (536.458,326.587) +
                //                idx 1 #1 (664.18,367.749) +
                //                idx 2 #2 (82.7494,106.134) +
                //                idx 3 #3 (86.1865,202.376)
                //                idx 4 #4 (9.32895,241.02)
                //                idx 5 #5 (3.8,236.333)
                //                idx 6 #6 (60.7541,274.965)
                //                Current input centroids: 3
                //                idx 0 #-1 (579.723,338.537) +
                //                idx 1 #-1 (664.355,367.809) +
                //                idx 2 #-1 (82.3868,173.369) +
                //                Lost object: idx 5 #5 lost 1
                //                Lost object: idx 4 #4 lost 1
                //                Lost object: idx 2 #2 lost 1
                //                Lost object: idx 1 #1 lost 1
                //                Updated object: idx 1 #1 x 664.355 y 367.809 lost 1
                //                Updated object: idx 3 #3 x 82.3868 y 173.369 lost 0
                //                Updated object: idx 0 #0 x 579.723 y 338.537 lost 0
                //                Updated object: idx 2 #2 x 82.3868 y 173.369 lost 1
                //                Updated object: idx 4 #4 x 82.3868 y 173.369 lost 1
                //                Updated object: idx 5 #5 x 82.3868 y 173.369 lost 1
                //                Updated object: idx 6 #6 x 82.3868 y 173.369 lost 0
                //                (OpenCV tracking) Total took: 1 ms
                //                ERROR Point match: obj idx 3 #3 (82.3868,173.369)
                //                ERROR Point match: obj idx 4 #4 (82.3868,173.369)
                //                ERROR Point match: obj idx 5 #5 (82.3868,173.369)
                //                ERROR Point match: obj idx 6 #6 (82.3868,173.369)
                //                ERROR Point match: obj idx 2 #2 (82.3868,173.369)
                //                ERROR Point match: obj idx 4 #4 (82.3868,173.369)
                //                ERROR Point match: obj idx 5 #5 (82.3868,173.369)
                //                ERROR Point match: obj idx 6 #6 (82.3868,173.369)
                //                ERROR Point match: obj idx 2 #2 (82.3868,173.369)
                //                ERROR Point match: obj idx 3 #3 (82.3868,173.369)
                //                ERROR Point match: obj idx 5 #5 (82.3868,173.369)
                //                ERROR Point match: obj idx 6 #6 (82.3868,173.369)
                //                ERROR Point match: obj idx 2 #2 (82.3868,173.369)
                //                ERROR Point match: obj idx 3 #3 (82.3868,173.369)
                //                ERROR Point match: obj idx 4 #4 (82.3868,173.369)
                //                ERROR Point match: obj idx 6 #6 (82.3868,173.369)
                //                ERROR Point match: obj idx 2 #2 (82.3868,173.369)
                //                ERROR Point match: obj idx 3 #3 (82.3868,173.369)
                //                ERROR Point match: obj idx 4 #4 (82.3868,173.369)
                //                ERROR Point match: obj idx 5 #5 (82.3868,173.369)
                //                (OpenCV capture) After distance transformation: 19 ms
                //                (OpenCV capture) After normalize thresh: 19 ms
                //                (OpenCV capture) After find&draw contours: 21 ms
                //                (OpenCV capture) Increased write index: 3 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV capture) Total took: 22 ms
                //                (OpenCV segmentation) Increased read index: 3 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                Before watershed: 0 ms
                //                After watershed: 2 ms
                //                (OpenCV segmentation) Increased queue size: 1 size: array of 4
                //                (OpenCV segmentation) Total took: 14 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 4
                //                Current object centroids: 7
                //                idx 0 #0 (579.723,338.537)
                //                idx 1 #1 (664.355,367.809)
                //                idx 2 #2 (82.3868,173.369)
                //                idx 3 #3 (82.3868,173.369)
                //                idx 4 #4 (82.3868,173.369)
                //                idx 5 #5 (82.3868,173.369)
                //                idx 6 #6 (82.3868,173.369)



                //                Current object centroids: 3
                //                idx 0 #0 (620.743,360.408)+
                //                idx 1 #1 (644.825,358.997)+
                //                idx 2 #2 (560.888,367.001)+
                //                Current input centroids: 3
                //                idx 0 #-1 (634.649,357.521)
                //                idx 1 #-1 (640.836,359.964)++
                //                idx 2 #-1 (620.714,352.996)+
                //                Minima rows: 3
                //                4.10396 at row,col: (1,1)
                //                7.41184 at row,col: (0,2)
                //                61.4432 at row,col: (2,2)
                //                Minima cols: 3
                //                10.2826 at row,col: (1,0)
                //                4.10396 at row,col: (1,1)
                //                7.41184 at row,col: (0,2)
                //                Maxima: 3
                //                80.2575 at row,col: (2,1)
                //                24.8464 at row,col: (1,2)
                //                20.098 at row,col: (0,1)
                //                Updated object: idx 1 #1 x 640.836 y 359.964 lost 0
                //                Updated object: idx 0 #0 x 620.714 y 352.996 lost 0
                //                Updated object: idx 2 #2 x 620.714 y 352.996 lost 0
                //                (OpenCV tracking) Total took: 0 ms
                //                ERROR Point match: obj idx 2 #2 (620.714,352.996)
                //                ERROR Point match: obj idx 0 #0 (620.714,352.996)

                //                Current object centroids: 3
                //                idx 0 #0 (620.714,352.996)
                //                idx 1 #1 (640.836,359.964)
                //                idx 2 #2 (620.714,352.996)


#if (IMSHOW > 0 && VERBOSE > 0)
                cv::Mat tracking_mat = cv::Mat::zeros(FRAME_HEIGHT_CV, FRAME_WIDTH_CV, CV_8U);
                for (size_t i = 0; i < tracked_objects_size; i++){
                    if (i == 0)
                    {
                        cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                             m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(100), -1);
                        cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                           roundToInt(m_tracked_objects->at(i).cy)), 10, cv::Scalar(255), 2);
                        cv::putText(tracking_mat, std::to_string(tracked_objects_size),
                                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, 255, 3);
                    }
                    else
                    {
                        cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                           roundToInt(m_tracked_objects->at(i).cy)), 5, cv::Scalar(255), -1);
                        cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                             m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(255), 1);
                    }
                    //                   cv::putText(tracking_mat, std::to_string((int)(mm_per_pix * tmp_tr_obj[i].w))+" x "+std::to_string((int)(mm_per_pix * tmp_tr_obj[i].h)),
                    //                               cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                }
                m_mat_tra_mtx.lock();
                m_current_mat_tra = tracking_mat;
                m_mat_tra_mtx.unlock();
#endif



#if (VERBOSE > 0)
                std::cout << "(OpenCV tracking) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_tra).count() << " ms" << std::endl;
#endif
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TRAC));
        }
        std::cout << "Tracking thread is exiting" << std::endl;
    }

    void imshow_thread_func()
    {
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
        {
            m_mat_seg_mtx.lock();
            cv::Mat seg = m_current_mat_seg;
            m_mat_seg_mtx.unlock();
            m_mat_tra_mtx.lock();
            cv::Mat tra = m_current_mat_tra;
            m_mat_tra_mtx.unlock();
            if ( !seg.empty() && !tra.empty() )
            {
                cv::imshow("Segmentation", seg);
                cv::imshow("Tracking" , tra);
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_SHOW));
        }
    }


    void deregisterObjects(std::vector<size_t> *obj_idxs, std::vector<tracked_object_t> *obj_centr)
    {
        std::sort(obj_idxs->begin(), obj_idxs->end(), std::greater<size_t>());
        for (size_t i = 0; i < obj_idxs->size(); i++) {
            auto idx = obj_idxs->at(i);
            std::cout << "deleting object idx " << idx << " #" << obj_centr->at(idx).unique_id << std::endl;
            obj_centr->erase(obj_centr->begin() + static_cast<long>(idx));
        }
    }


    cv::Mat centroidDistances(std::vector<tracked_object_t> *obj_centr, tracked_object_t *inp_centr, size_t size_in)
    {
        size_t size_obj = obj_centr->size();
        cv::Mat result = cv::Mat::zeros(static_cast<int>(size_obj), static_cast<int>(size_in), CV_64F);
        for (size_t i = 0; i < size_obj; i++)
        {
            double x_o = obj_centr->at(i).cx;
            double y_o = obj_centr->at(i).cy;
            for (size_t j = 0; j < size_in; ++j)
            {
                double dx = inp_centr[j].cx - x_o;
                double dy = inp_centr[j].cy - y_o;
                result.at<double>(static_cast<int>(i), static_cast<int>(j)) = std::sqrt(dx * dx + dy * dy);
            }
        }
        return result;
    }

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

        //  m_tracked_objects_buf = new TrackedObjects[BUF_SIZE_TOBJ];
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
        std::cout << "OcvDevice destructor" << std::endl;
        for (int i=0; i<BUF_SIZE_CVCAP; i++)
        {
            delete[] m_cvcap_buf[i];
        }
        delete[] m_cvcap_buf;

        delete[] m_tracked_objects_buf;
    }
};

#endif // OCVDEVICE_H

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

    // tracked objects
    //    TrackedObjects m_obj_centroids;
    //    TrackedObjects m_in_centroids;

    tracked_object_t *m_input_objects;
    std::vector<tracked_object_t> *m_tracked_objects;
    int m_max_disappeared = 0;
    uint16_t m_id_ctr = 0;

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
        tracked_object_t* tmp_tr_obj;
#if (IMSHOW_SEG > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
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
#if (IMSHOW_SEG > 0) || (IMSHOW_CAP > 0)
                    // Draw markers on cv window
                    cv::rectangle(marker, cv::Rect(tmp_tr_obj[i].x, tmp_tr_obj[i].y, tmp_tr_obj[i].w, tmp_tr_obj[i].h), cv::Scalar(0), 1);
                    cv::putText(marker, std::to_string((int)(mm_per_pix * tmp_tr_obj[i].w))+" x "+std::to_string((int)(mm_per_pix * tmp_tr_obj[i].h)),
                                cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
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
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
#if (IMSHOW_SEG > 0)
                cv::imshow("Segmented image", marker*(255/marker_count));
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
#if (VERBOSE > 3)
                for (size_t i = 0; i < input_objects_size; i++) {
                    if (i > 0)
                        std::cout << "Label " << i << " - pos: (" << m_input_objects[i].x << "," << m_input_objects[i].y << ") center: (" << m_input_objects[i].cx << ","
                                  << m_input_objects[i].cy << ") size: " << m_input_objects[i].w << " x " << m_input_objects[i].h << " area: " << m_input_objects[i].area << std::endl;
                    else
                        std::cout << "Background - pos: (" << m_input_objects[i].x << "," << m_input_objects[i].y << ") center: (" << m_input_objects[i].cx << ","
                                  << m_input_objects[i].cy << ") size: " << m_input_objects[i].w << " x " << m_input_objects[i].h << " area: " << m_input_objects[i].area << std::endl;
                }
#endif
                // Track objects

#if (VERBOSE > 0)
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
                        std::cout << "New all objects: idx "<< i << " #" << m_tracked_objects->back().unique_id << std::endl;
                    }
                }
                else if (tracked_objects_size > 0 && input_objects_size == 0)
                {
                    // Deregister all tracked objects
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        m_tracked_objects->at(i).lost_ctr++;
                        if (m_tracked_objects->at(i).lost_ctr >= m_max_disappeared)
                        {
                            lost_objects.push_back(i);
                            std::cout << "Lost all objects: " << lost_objects.back() << std::endl;
                        }
                    }
                }
                else {
                    // Compute centroid distances
                    distances = centroidDistances(m_tracked_objects, m_input_objects, input_objects_size);
#if (VERBOSE > 0)
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
                    //  std::sort(col_min_distances.begin(), col_min_distances.end());
                    std::sort(max_distances.begin(), max_distances.end(), std::greater<std::tuple<double,int,int>>());
#if (VERBOSE > 0)
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
                    // Update current objects
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        size_t obj_idx = static_cast<size_t>(std::get<1>(row_min_distances[i]))/*row*/;
                        size_t inp_idx = static_cast<size_t>(std::get<2>(row_min_distances[i]))/*col*/;
                        auto tmp_obj = m_input_objects[inp_idx];
                        tmp_obj.unique_id = m_tracked_objects->at(obj_idx).unique_id; // !!!
                        m_tracked_objects->at(obj_idx) = tmp_obj;
                        std::cout << "Updated object: idx " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                                  << " x " << m_tracked_objects->at(obj_idx).cx
                                  << " y " << m_tracked_objects->at(obj_idx).cy << std::endl;
                    }

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
                                std::cout << "Lost object: idx " << lost_objects.back() << " #" << m_tracked_objects->at(obj_idx).unique_id
                                          << " x " << m_tracked_objects->at(obj_idx).cx
                                          << " y " << m_tracked_objects->at(obj_idx).cy << std::endl;
                            }
                        }
                    }
                    else if (tracked_objects_size < input_objects_size)
                    {
                        // Register specific objects
                        size_t to_add = input_objects_size - tracked_objects_size;
                        std::cout << "Debug to_add: " << to_add << std::endl;
                        for (size_t i = 0; i < to_add; i++)
                        {
                            size_t inp_idx = static_cast<size_t>(std::get<2>(max_distances[i]))/*col or row?*/;
                            std::cout << "Debug input object add: idx col " << inp_idx << " idx row " << std::get<1>(max_distances[i]) << " #" << m_id_ctr << std::endl;
                            auto tmp_obj = m_input_objects[inp_idx]; // !!!
                            bool skip = false;
                            tmp_obj.unique_id = m_id_ctr++;

                            // Check if same point already exists
                            for (size_t j = 0; j < tracked_objects_size; j++)
                            {
                                if ( areSame(tmp_obj.x, m_tracked_objects->at(j).x) && areSame(tmp_obj.y, m_tracked_objects->at(j).y) )
                                {
                                    std::cerr << "Point match: idx "<< j << " #" << m_tracked_objects->at(j).unique_id << std::endl;
                                    skip = true;
                                }
                            }
                            //                            if (skip)
                            //                            {
                            //                                std::cerr << "Point skipped" << std::endl;
                            //                            }
                            //                            else
                            //                            {
                            m_tracked_objects->push_back(tmp_obj);
                            std::cout << "New object: idx " << m_tracked_objects->size()-1 << " #" << m_tracked_objects->back().unique_id
                                      << " x " << m_tracked_objects->back().cx
                                      << " y " << m_tracked_objects->back().cy << std::endl;
                            //                            }

                        }
                    }
                }



                //                Current object centroids: 4
                //                idx 0 #0 (525.459,325.29)+
                //                idx 1 #1 (70.3242,27.4778)+
                //                idx 2 #2 (198.21,201.209)+
                //                idx 3 #3 (718.24,389.078)+
                //                Current input centroids: 6
                //                idx 0 #-1 (536.979,310.492)+
                //                idx 1 #-1 (63.7887,24.2529)+
                //                idx 2 #-1 (197.095,204.604)+
                //                idx 3 #-1 (722.737,390.796)+
                //                idx 4 #-1 (487.376,115.384)
                //                idx 5 #-1 (474.907,103.549)
                //                Distance matrix ( 4 x 6 ):
                //                [18.75329241444043, 551.146519046812, 349.8400092875225, 207.8691012090353, 213.3327111073276, 227.4306348644634;
                //                 545.7686948549622, 7.287858780345206, 217.8172278279624, 746.7541937474931, 426.2153070189767, 411.6723279120752;
                //                 355.9589650869916, 222.2221410672822, 3.573056757721439, 557.7373018397565, 301.6333191883944, 293.4258814482594;
                //                 197.5635926765162, 749.2688337513224, 552.8317494093509, 4.813626468653316, 358.0598635052168, 375.1507217167562]
                //                Minima rows: 4
                //                3.57306 at row,col: (2,2)=
                //                4.81363 at row,col: (3,3)=
                //                7.28786 at row,col: (1,1)=
                //                18.7533 at row,col: (0,0)=
                //                Minima cols: 6
                //                18.7533 at row,col: (0,0)=
                //                7.28786 at row,col: (1,1)=
                //                3.57306 at row,col: (2,2)=
                //                4.81363 at row,col: (3,3)=
                //                213.333 at row,col: (0,4)
                //                227.431 at row,col: (0,5)
                //                Maxima: 4
                //                749.269 at row,col: (3,1)
                //                746.754 at row,col: (1,3)
                //                557.737 at row,col: (2,3)
                //                551.147 at row,col: (0,1)
                //                Updated object: idx 2 #2 x 197.095 y 204.604
                //                Updated object: idx 3 #3 x 722.737 y 390.796
                //                Updated object: idx 1 #1 x 63.7887 y 24.2529
                //                Updated object: idx 0 #0 x 536.979 y 310.492
                //                Debug to_add: 2
                //                Debug input object add: idx col 1 idx row 3 #4
                //                New object: idx 4 #4 x 722.737 y 390.796 // changed back to col after this debug
                //                Debug input object add: idx col 3 idx row 1 #5
                //                New object: idx 5 #5 x 63.7887 y 24.2529
                //                (OpenCV tracking) Total took: 2 ms
                //                (OpenCV segmentation) Increased read index: 6 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                Point match: idx 1 #1
                //                Point match: idx 2 #2
                //                Point match: idx 3 #3
                //                Point match: idx 1 #1
                //                Point match: idx 2 #2
                //                Point match: idx 3 #3
                //                (OpenCV segmentation) Increased queue size: 1 size: array of 5
                //                (OpenCV segmentation) Total took: 11 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 5
                //                Current object centroids: 6
                //                idx 0 #0 (536.979,310.492)
                //                idx 1 #1 (63.7887,24.2529)
                //                idx 2 #2 (197.095,204.604)
                //                idx 3 #3 (722.737,390.796)
                //                idx 4 #4 (722.737,390.796)
                //                idx 5 #5 (63.7887,24.2529)




                //                Current object centroids: 4
                //                idx 0 #0 (606.825,330.88)+
                //                idx 1 #1 (651.406,365.704)+
                //                idx 2 #2 (1102.09,51.093)+
                //                idx 3 #4 (327.414,213.368)+
                //                Current input centroids: 5
                //                idx 0 #-1 (633.966,314.625)+
                //                idx 1 #-1 (646.71,369.032)+
                //                idx 2 #-1 (1063.13,49.1031)   ---------
                //                idx 3 #-1 (1100.31,59.1559)+
                //                idx 4 #-1 (330.81,214.288)+
                //                Distance matrix ( 4 x 5 ):
                //                [31.63628625654063, 55.19413620798102, 536.2971307529453, 563.3474089135461, 299.6299721668431;
                //                 53.97447862283411, 5.755576878113144, 519.3801318612116, 543.5867613354541, 354.5538409559917;
                //                 537.2052972351736, 555.3893722994709, 39.00875191803778, 8.257333020416855, 788.3569155438075;
                //                 322.8430082897389, 355.2204294695402, 753.834176313242, 788.1303407538394, 3.518941318206198]
                //                Minima rows: 4
                //                3.51894 at row,col: (3,4)=
                //                5.75558 at row,col: (1,1)=
                //                8.25733 at row,col: (2,3)=
                //                31.6363 at row,col: (0,0)=
                //                Minima cols: 5
                //                31.6363 at row,col: (0,0)=
                //                5.75558 at row,col: (1,1)=
                //                39.0088 at row,col: (2,2)   --------------
                //                8.25733 at row,col: (2,3)=
                //                3.51894 at row,col: (3,4)=
                //                Maxima: 4
                //                788.357 at row,col: (2,4)
                //                788.13 at row,col: (3,3)
                //                563.347 at row,col: (0,3)
                //                543.587 at row,col: (1,3)
                //                Updated object: idx 3 #4 x 330.81 y 214.288
                //                Updated object: idx 1 #1 x 646.71 y 369.032
                //                Updated object: idx 2 #2 x 1100.31 y 59.1559
                //                Updated object: idx 0 #0 x 633.966 y 314.625
                //                Debug to_add: 1
                //                Debug input object add: idx col 4 idx row 2 #5
                //                New object: idx 4 #5 x 330.81 y 214.288 // changed to row after this debug
                //                (OpenCV tracking) Total took: 2 ms
                //                Point match: idx 3 #4
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 4
                //                Current object centroids: 5
                //                idx 0 #0 (633.966,314.625)
                //                idx 1 #1 (646.71,369.032)
                //                idx 2 #2 (1100.31,59.1559)
                //                idx 3 #4 (330.81,214.288)
                //                idx 4 #5 (330.81,214.288) -----------




                // bug 1: assigning the same point several times, change something with the minima


                //                Current object centroids: 4
                //                idx 0 #0 (642.895,342.345)
                //                idx 1 #1 (640.306,361.452)
                //                idx 2 #2 (972.904,192.83)
                //                idx 3 #3 (527.216,244.983)
                //                Current input centroids: 3
                //                idx 0 #-1 (629.097,349.966)
                //                idx 1 #-1 (641.035,360.909)
                //                idx 2 #-1 (526.734,255.955)
                //                Distance matrix ( 4 x 3 ):
                //                [15.76210300218083, 18.65773113366837, 144.7634625011049;
                //                 16.04849575235883, 0.9092481883047626, 155.0102960404363;
                //                 378.0143227356846, 372.0053011725211, 450.6133705993814;
                //                 146.2916217362687, 162.4613085045341, 10.98237217747797]
                //                Minima: 4
                //                0.909248 at row,col: (1,1)
                //                10.9824 at row,col: (3,2)
                //                15.7621 at row,col: (0,0)
                //                372.005 at row,col: (2,1)
                //                Maxima: 4
                //                450.613 at row,col: (2,2)
                //                162.461 at row,col: (3,1)
                //                155.01 at row,col: (1,2)
                //                144.763 at row,col: (0,2)
                //                Updated object: idx 1 #1 x 641.035 y 360.909
                //                Updated object: idx 3 #3 x 526.734 y 255.955
                //                Updated object: idx 0 #0 x 629.097 y 349.966
                //                Updated object: idx 2 #2 x 641.035 y 360.909
                //                Lost object: idx 2 #2 x 641.035 y 360.909
                //                deleting object idx 2 #2
                //                (OpenCV tracking) Total took: 1 ms
                //                (OpenCV capture) Increased write index: 5 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV capture) Total took: 12 ms
                //                (OpenCV segmentation) Increased read index: 5 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV segmentation) Increased queue size: 1 size: array of 4
                //                (OpenCV segmentation) Total took: 10 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 4
                //                Current object centroids: 3
                //                idx 0 #0 (629.097,349.966)+
                //                idx 1 #1 (641.035,360.909)+
                //                idx 2 #3 (526.734,255.955)+
                //                Current input centroids: 4
                //                idx 0 #-1 (643.973,343.209)
                //                idx 1 #-1 (639.951,361.471)++
                //                idx 2 #-1 (962.405,204.13)
                //                idx 3 #-1 (515.926,259.406)+
                //                Distance matrix ( 3 x 4 ):
                //                [16.33834355871208, 15.8165526228977, 363.8161856882104, 144.9443938364356;
                //                 17.94289681362182, 1.221347123331795, 357.57329077322, 161.1061430884407;
                //                 146.144515404077, 154.7634622992736, 438.7427072104057, 11.3460058478314]
                //                Minima: 3
                //                1.22135 at row,col: (1,1)
                //                11.346 at row,col: (2,3)
                //                15.8166 at row,col: (0,1)
                //                Maxima: 3
                //                438.743 at row,col: (2,2)
                //                363.816 at row,col: (0,2)
                //                357.573 at row,col: (1,2)
                //                Updated object: idx 1 #1 x 639.951 y 361.471
                //                Updated object: idx 2 #3 x 515.926 y 259.406
                //                Updated object: idx 0 #0 x 639.951 y 361.471
                //                Debug to_add: 1
                //                Debug input object add: idx col 2 idx row 2 #4
                //                New object: idx 3 #4 x 962.405 y 204.13
                //                (OpenCV tracking) Total took: 3 ms
                //                (OpenCV capture) Increased write index: 6 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV capture) Total took: 16 ms
                //                (OpenCV segmentation) Increased read index: 6 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV segmentation) Increased queue size: 1 size: array of 5
                //                (OpenCV segmentation) Total took: 11 ms
                //                (OpenCV capture) Increased write index: 7 size0 [1280 x 720] type0 16 size1 [1280 x 720] type1 4
                //                (OpenCV capture) Total took: 15 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 5
                //                Current object centroids: 4
                //                idx 0 #0 (639.951,361.471)
                //                idx 1 #1 (639.951,361.471)
                //                idx 2 #3 (515.926,259.406)
                //                idx 3 #4 (962.405,204.13)













                // bug 2: adding the wrong input index


                //                Current object centroids: 4
                //                idx 0 #0 (683.369,378.302)+
                //                idx 1 #1 (596.699,341.753)+
                //                idx 2 #2 (305.96,270.547)+
                //                idx 3 #5 (1044.91,521.188)+
                //                Current input centroids: 4
                //                idx 0 #-1 (689.564,380.065)+
                //                idx 1 #-1 (597.221,341.643)+
                //                idx 2 #-1 (299.493,275.638)+
                //                idx 3 #-1 (1046.57,522.775)+
                //                Distance matrix ( 4 x 4 ):
                //                [6.440979648931084, 93.62263293071025, 397.366759898874, 390.8837606246364;
                //                 100.4573369694134, 0.5342091928115595, 304.4707157521901, 484.9289418761837;
                //                 398.9313622654873, 299.8135159369417, 8.230292074281216, 782.385932411932;
                //                 382.3448898413069, 482.3508936393918, 784.8203840105554, 2.298144033958473]
                //                Minima: 4
                //                0.534209 at row,col: (1,1)
                //                2.29814 at row,col: (3,3)
                //                6.44098 at row,col: (0,0)
                //                8.23029 at row,col: (2,2)
                //                Maxima: 4
                //                784.82 at row,col: (3,2)
                //                782.386 at row,col: (2,3)
                //                484.929 at row,col: (1,3)
                //                397.367 at row,col: (0,2)
                //                Updated object: idx 1 #1 x 597.221 y 341.643
                //                Updated object: idx 3 #5 x 1046.57 y 522.775
                //                Updated object: idx 0 #0 x 689.564 y 380.065
                //                Updated object: idx 2 #2 x 299.493 y 275.638
                //                (OpenCV tracking) Total took: 1 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 5

                //                Current object centroids: 4
                //                idx 0 #0 (689.564,380.065) +
                //                idx 1 #1 (597.221,341.643) +
                //                idx 2 #2 (299.493,275.638) +
                //                idx 3 #5 (1046.57,522.775) +
                //                Current input centroids: 5
                //                idx 0 #-1 (693.074,372.85) +
                //                idx 1 #-1 (596.486,342.16) +
                //                idx 2 #-1 (752.388,185.913) --------------
                //                idx 3 #-1 (300.599,277.261) +
                //                idx 4 #-1 (1047.09,523.816) +
                //                Distance matrix ( 4 x 5 ):
                //                [8.023649847120954, 100.4998894864852, 204.063611153311, 402.3214457564323, 385.3408156432844;
                //                 100.8044304801655, 0.8987935618389953, 219.8381232425886, 303.5296767492395, 485.3515736979858;
                //                 405.4084523895476, 304.3517385048092, 461.6976826801945, 1.963576622034126, 787.7115368693632;
                //                 383.9784432067437, 484.9746541605048, 447.2369625778045, 785.3379523750431, 1.161151367919249]
                //                Minima: 4
                //                0.898794 at row,col: (1,1)
                //                1.16115 at row,col: (3,4)
                //                1.96358 at row,col: (2,3)
                //                8.02365 at row,col: (0,0)
                //                Maxima: 4
                //                787.712 at row,col: ( -> 2 <- ,4)
                //                785.338 at row,col: (3,3)
                //                485.352 at row,col: (1,4)
                //                402.321 at row,col: (0,3)
                //                Updated object: idx 1 #1 x 596.486 y 342.16
                //                Updated object: idx 3 #5 x 1047.09 y 523.816
                //                Updated object: idx 2 #2 x 300.599 y 277.261
                //                Updated object: idx 0 #0 x 693.074 y 372.85
                //                Debug to_add: 1
                //                Debug input object add: idx 4 #7
                //                New object: idx 4 #6 x 1047.09 y 523.816 --------------------- (752.388,185.913)
                //                (OpenCV tracking) Total took: 1 ms
                //                Point match: idx 3 #5 ------------------------------------------------
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 4

                //                Current object centroids: 5
                //                idx 0 #0 (693.074,372.85)
                //                idx 1 #1 (596.486,342.16)+
                //                idx 2 #2 (300.599,277.261)+
                //                idx 3 #5 (1047.09,523.816)
                //                idx 4 #6 (1047.09,523.816)
                //                Current input centroids: 4
                //                idx 0 #-1 (691.808,381.238)
                //                idx 1 #-1 (596.932,341.273)+
                //                idx 2 #-1 (302.586,279.061)+
                //                idx 3 #-1 (1049.26,525.654)
                //                Distance matrix ( 5 x 4 ):
                //                [8.483095579151582, 101.1950153623298, 401.5931352878609, 387.5755965576175;
                //                 103.0206901545194, 0.9923673592300829, 300.5971058419241, 488.5393973747424;
                //                 404.7909033038528, 303.1680023494777, 2.681627301914399, 788.7884398581305;
                //                 382.8214668711114, 485.759259738593, 783.7008445035273, 2.842863212023687;
                //                 382.8214668711114, 485.759259738593, 783.7008445035273, 2.842863212023687]
                //                Minima: 5
                //                0.992367 at row,col: (1,1)
                //                2.68163 at row,col: (2,2)
                //                2.84286 at row,col: (3,3)
                //                2.84286 at row,col: (4,3)
                //                8.4831 at row,col: (0,0)
                //                Maxima: 5
                //                788.788 at row,col: (2,3)
                //                783.701 at row,col: (4,2)
                //                783.701 at row,col: (3,2)
                //                488.539 at row,col: (1,3)
                //                401.593 at row,col: (0,2)
                //                Updated object: idx 1 #1 x 596.932 y 341.273
                //                Updated object: idx 2 #2 x 302.586 y 279.061
                //                Updated object: idx 3 #5 x 1049.26 y 525.654
                //                Updated object: idx 4 #6 x 1049.26 y 525.654
                //                Updated object: idx 0 #0 x 691.808 y 381.238
                //                Lost object: idx 2 #2 x 302.586 y 279.061
                //                deleting object idx 2 #2
                //                (OpenCV tracking) Total took: 1 ms
                //                (OpenCV tracking) Reduced queue size: 0 size: array of 5

                //                Current object centroids: 4
                //                idx 0 #0 (691.808,381.238)
                //                idx 1 #1 (596.932,341.273)
                //                idx 2 #5 (1049.26,525.654)
                //                idx 3 #6 (1049.26,525.654)


                // Deregister lost objects
                if ( !lost_objects.empty() ) {
                    deregisterObjects(&lost_objects, m_tracked_objects);

                }

                tracked_objects_size = m_tracked_objects->size();

#if (IMSHOW_TRA > 0)
                cv::Mat tracking_mat = cv::Mat::zeros(FRAME_HEIGHT_CV, FRAME_WIDTH_CV, CV_8U);
                // Draw markers on cv window
                for (size_t i = 0; i < tracked_objects_size; i++){
                    if (i == 0)
                    {
                        cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                             m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(100), -1);
                        cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                           roundToInt(m_tracked_objects->at(i).cy)), 10, cv::Scalar(255), 2);
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
                cv::imshow("Tracking", tracking_mat);

#endif

#if (VERBOSE > 0)
                std::cout << "(OpenCV tracking) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_tra).count() << " ms" << std::endl;
#endif
#if (IMSHOW_TRA > 0)
#endif
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TRAC));
        }
        std::cout << "Tracking thread is exiting" << std::endl;
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
